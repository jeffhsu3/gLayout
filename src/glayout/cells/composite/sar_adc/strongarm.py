"""Matched, fail-closed StrongARM comparator generator for SKY130.

The implementation is intentionally independent of the old experimental
``strongarm_latch`` composite.  Functional devices are built from parallel,
equal-area fingers and placed in mirrored ABBA rows.  Layout connectivity and
the canonical schematic netlist are generated from the same immutable device
table, but the canonical netlist is emitted directly so layout-routing failures
can never produce an empty or altered schematic.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.comp_utils import evaluate_bbox
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.primitives.fet import nmos, pmos
from glayout.primitives.guardring import tapring
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist


STRONGARM_PINS: tuple[str, ...] = ("VDD", "X", "Y", "CLK", "VIN2", "VIN1")
LAYOUT_PINS: tuple[str, ...] = STRONGARM_PINS + ("VSS",)


@dataclass(frozen=True)
class StrongArmDevice:
    name: str
    kind: str
    drain: str
    gate: str
    source: str
    bulk: str
    width_um: float
    length_um: float = 0.15


FUNCTIONAL_DEVICES: tuple[StrongArmDevice, ...] = (
    StrongArmDevice("S1", "pfet", "P", "CLK", "VDD", "VDD", 1.0),
    StrongArmDevice("S3", "pfet", "X", "CLK", "VDD", "VDD", 1.0),
    StrongArmDevice("M5", "pfet", "X", "Y", "VDD", "VDD", 4.0),
    StrongArmDevice("M6", "pfet", "Y", "X", "VDD", "VDD", 4.0),
    StrongArmDevice("S4", "pfet", "Y", "CLK", "VDD", "VDD", 1.0),
    StrongArmDevice("S2", "pfet", "Q", "CLK", "VDD", "VDD", 1.0),
    StrongArmDevice("M4", "nfet", "Y", "X", "Q", "VSS", 4.0),
    StrongArmDevice("M3", "nfet", "X", "Y", "P", "VSS", 4.0),
    StrongArmDevice("M1", "nfet", "P", "VIN1", "TAIL", "VSS", 8.0),
    StrongArmDevice("M2", "nfet", "Q", "VIN2", "TAIL", "VSS", 8.0),
    StrongArmDevice("M7", "nfet", "TAIL", "CLK", "VSS", "VSS", 2.0),
)


def _scaled_devices(
    device_scale: int,
    input_pair_scale: int | None = None,
    input_pair_length_scale: int = 1,
) -> tuple[StrongArmDevice, ...]:
    if not isinstance(device_scale, int) or device_scale < 1:
        raise ValueError("device_scale must be a positive integer")
    if input_pair_scale is None:
        input_pair_scale = device_scale
    if not isinstance(input_pair_scale, int) or input_pair_scale < 1:
        raise ValueError("input_pair_scale must be a positive integer")
    if not isinstance(input_pair_length_scale, int) or input_pair_length_scale < 1:
        raise ValueError("input_pair_length_scale must be a positive integer")
    return tuple(
        StrongArmDevice(
            name=device.name,
            kind=device.kind,
            drain=device.drain,
            gate=device.gate,
            source=device.source,
            bulk=device.bulk,
            width_um=device.width_um
            * (input_pair_scale if device.name in {"M1", "M2"} else device_scale),
            length_um=device.length_um
            * (input_pair_length_scale if device.name in {"M1", "M2"} else 1),
        )
        for device in FUNCTIONAL_DEVICES
    )


def _canonical_netlist(
    *,
    devices: tuple[StrongArmDevice, ...] = FUNCTIONAL_DEVICES,
    include_vss_pin: bool = False,
    include_layout_dummies: bool = False,
    layout_dummies: list[dict[str, object]] | None = None,
) -> Netlist:
    nodes = list(STRONGARM_PINS)
    if include_vss_pin:
        nodes.append("VSS")
    ground = "VSS" if include_vss_pin else "0"
    lines = [f".subckt strongarm_comparator {' '.join(nodes)}"]
    for device in devices:
        bulk = ground if device.bulk == "VSS" else device.bulk
        source = ground if device.source == "VSS" else device.source
        lines.append(
            f"X{device.name} {device.drain} {device.gate} {source} {bulk} "
            f"sky130_fd_pr__{device.kind}_01v8 L={device.length_um:g} "
            f"W={device.width_um:g} nf=1 mult=1"
        )
    if include_layout_dummies:
        if not include_vss_pin:
            raise ValueError("layout dummies require the explicit VSS LVS interface")
        if layout_dummies is None:
            layout_dummies = [
                {"kind": "pfet", "body": "VDD", "width_um": 4.0, "length_um": 0.15},
                {"kind": "nfet", "body": "VSS", "width_um": 6.0, "length_um": 0.15},
            ]
        for index, dummy in enumerate(layout_dummies):
            body = str(dummy["body"])
            lines.append(
                f"XDUMMY_{index} {body} {body} {body} {body} "
                f"sky130_fd_pr__{dummy['kind']}_01v8 "
                f"L={float(dummy['length_um']):g} "
                f"W={float(dummy['width_um']):g} nf=1 mult=1"
            )
    lines.append(".ends strongarm_comparator")
    return Netlist(
        circuit_name="strongarm_comparator",
        nodes=nodes,
        source_netlist="\n".join(lines),
    )


def _abba_units(
    left: StrongArmDevice, right: StrongArmDevice, finger_width_um: float = 1.0
) -> list[StrongArmDevice]:
    fingers = left.width_um / finger_width_um
    if left.width_um != right.width_um or int(fingers) != fingers or int(fingers) % 2:
        raise ValueError("ABBA devices must have equal positive even finger counts")
    repeats = int(fingers) // 2
    pattern: list[StrongArmDevice] = []
    for _ in range(repeats):
        pattern.extend((left, right, right, left))
    return pattern


def _unit_rows(
    devices: tuple[StrongArmDevice, ...] = FUNCTIONAL_DEVICES,
    finger_widths: dict[str, float] | None = None,
) -> tuple[tuple[str, str, list[StrongArmDevice]], ...]:
    devices_by_name = {device.name: device for device in devices}
    if finger_widths is None:
        finger_widths = {device.name: 1.0 for device in devices}
    reset_scale = int(
        devices_by_name["S1"].width_um / finger_widths[devices_by_name["S1"].name]
    )
    if any(
        devices_by_name[name].width_um / finger_widths[name] != reset_scale
        for name in ("S1", "S2", "S3", "S4")
    ):
        raise ValueError("the four reset devices must have equal integral widths")
    reset = [
        devices_by_name[name]
        for _ in range(reset_scale)
        for name in ("S1", "S3", "S4", "S2")
    ]
    return (
        ("reset", "pfet", reset),
        (
            "latch_p",
            "pfet",
            _abba_units(
                devices_by_name["M5"],
                devices_by_name["M6"],
                finger_widths["M5"],
            ),
        ),
        (
            "latch_n",
            "nfet",
            _abba_units(
                devices_by_name["M3"],
                devices_by_name["M4"],
                finger_widths["M3"],
            ),
        ),
        (
            "input",
            "nfet",
            _abba_units(
                devices_by_name["M1"],
                devices_by_name["M2"],
                finger_widths["M1"],
            ),
        ),
        (
            "tail",
            "nfet",
            [devices_by_name["M7"]]
            * int(devices_by_name["M7"].width_um / finger_widths["M7"]),
        ),
    )


def _add_rect(
    component: Component,
    layer: tuple[int, int],
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> None:
    if xmax <= xmin or ymax <= ymin:
        raise ValueError("route rectangle must have positive area")
    component.add_polygon(
        [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)],
        layer=layer,
    )


def _snap(pdk: MappedPDK, value: float) -> float:
    """Snap route centres before placing fixed-size cut arrays.

    A half-grid centre makes a legal 0.15 um cut round to 0.155 um when the
    GDS is quantized, which the SKY130 deck correctly rejects.
    """

    return float(pdk.snap_to_2xgrid(value))


def _metal_stub_to_track(
    component: Component,
    *,
    pdk: MappedPDK,
    port,
    branch_x: float,
    track_y: float,
) -> None:
    """Connect one M2 terminal to an M3 row track without hidden fallbacks."""

    if pdk.layer_to_glayer(port.layer) != "met2":
        raise ValueError(f"expected an M2 device terminal, got {port.layer}")
    m2 = pdk.get_glayer("met2")
    width = pdk.get_grule("met2")["min_width"]
    x0, y0 = float(port.center[0]), float(port.center[1])
    branch_x = _snap(pdk, branch_x)
    track_y = _snap(pdk, track_y)
    _add_rect(
        component,
        m2,
        min(x0, branch_x) - width / 2,
        y0 - width / 2,
        max(x0, branch_x) + width / 2,
        y0 + width / 2,
    )
    _add_rect(
        component,
        m2,
        branch_x - width / 2,
        min(y0, track_y) - width / 2,
        branch_x + width / 2,
        max(y0, track_y) + width / 2,
    )
    transition = component << via_stack(pdk, "met2", "met3")
    transition.move((branch_x, track_y))


def _ring_port(reference, side: str):
    candidates = [
        port
        for name, port in reference.ports.items()
        if name.startswith(f"{side}_") and "top_met" in name
    ]
    if not candidates:
        raise RuntimeError(f"tap ring has no usable {side} metal port")
    return min(candidates, key=lambda port: abs(float(port.center[0])))


def strongarm_comparator(
    pdk: MappedPDK,
    *,
    unit_width_um: float = 1.0,
    length_um: float = 0.15,
    device_scale: int = 1,
    input_pair_scale: int | None = None,
    input_pair_length_scale: int = 1,
) -> Component:
    """Generate the exact 11-device StrongARM comparator.

    The layout exposes an additional VSS physical pin.  ``info['netlist']`` is
    the legacy six-pin subcircuit with ground node 0; ``info['lvs_netlist']`` is
    the equivalent explicit-seven-pin form used for physical signoff.
    """

    if unit_width_um != 1.0:
        raise ValueError("this matched implementation requires 1 um unit fingers")
    if length_um != 0.15:
        raise ValueError("this SKY130 implementation is qualified only at L=0.15 um")
    functional_devices = _scaled_devices(
        device_scale, input_pair_scale, input_pair_length_scale
    )
    resolved_input_pair_scale = (
        device_scale if input_pair_scale is None else input_pair_scale
    )
    input_finger_width_um = (
        unit_width_um if input_pair_scale is None else unit_width_um * input_pair_scale
    )
    finger_widths = {
        device.name: (
            input_finger_width_um if device.name in {"M1", "M2"} else unit_width_um
        )
        for device in functional_devices
    }
    finger_lengths = {device.name: device.length_um for device in functional_devices}
    pdk.activate()
    top = Component(name="strongarm_comparator")
    unit_by_geometry = {}
    primitive_specs = sorted(
        {
            (device.kind, finger_widths[device.name], finger_lengths[device.name])
            for device in functional_devices
        }
    )
    for kind, width, primitive_length in primitive_specs:
        generator = nmos if kind == "nfet" else pmos
        options = {
            "width": width,
            "length": primitive_length,
            "fingers": 1,
            "multipliers": 1,
            "with_tie": False,
            "with_dummy": False,
            "with_substrate_tap": False,
        }
        if kind == "nfet":
            options["with_dnwell"] = False
        unit = generator(pdk, **options)
        # The generic mapped PDK's ``pwell`` GDS purpose (64/44) is not
        # imported by the SKY130 Magic deck. Ordinary 1.8 V NMOS devices
        # belong directly in the p-substrate.
        if kind == "nfet":
            unit.remove_layers([pdk.get_glayer("pwell")])
        unit_by_geometry[(kind, width, primitive_length)] = unit
    unit_pitch = pdk.snap_to_2xgrid(
        max(evaluate_bbox(unit)[0] for unit in unit_by_geometry.values()) + 0.8
    )
    row_y = {
        "reset": 42.0,
        "latch_p": 24.0,
        "latch_n": 6.0,
        "input": -14.0,
        "tail": -34.0,
    }

    row_refs: dict[str, list[tuple[object, StrongArmDevice | None]]] = {}
    all_functional_refs: list[tuple[object, StrongArmDevice]] = []
    dummy_specs: list[dict[str, object]] = []
    unit_rows = _unit_rows(functional_devices, finger_widths)
    for row_name, kind, units in unit_rows:
        entries: list[tuple[object, StrongArmDevice | None]] = []
        count = len(units) + 2
        row_unit_width = input_finger_width_um if row_name == "input" else unit_width_um
        row_unit_length = (
            length_um * input_pair_length_scale if row_name == "input" else length_um
        )
        for index in range(count):
            ref = top << unit_by_geometry[(kind, row_unit_width, row_unit_length)]
            ref.move(
                (
                    _snap(pdk, (index - (count - 1) / 2) * unit_pitch),
                    _snap(pdk, row_y[row_name]),
                )
            )
            if index in (0, count - 1):
                body = "VDD" if kind == "pfet" else "VSS"
                entries.append((ref, None))
                dummy_specs.append(
                    {
                        "row": row_name,
                        "kind": kind,
                        "body": body,
                        "width_um": row_unit_width,
                        "length_um": row_unit_length,
                    }
                )
            else:
                device = units[index - 1]
                entries.append((ref, device))
                all_functional_refs.append((ref, device))
        row_refs[row_name] = entries

    core_xmin = min(ref.xmin for entries in row_refs.values() for ref, _ in entries)
    core_xmax = max(ref.xmax for entries in row_refs.values() for ref, _ in entries)
    core_half = max(abs(core_xmin), abs(core_xmax))
    spine_x = {
        "X": _snap(pdk, -(core_half + 4.0)),
        "Y": _snap(pdk, core_half + 4.0),
        "P": _snap(pdk, -(core_half + 6.0)),
        "Q": _snap(pdk, core_half + 6.0),
        "VIN1": _snap(pdk, -(core_half + 8.0)),
        "VIN2": _snap(pdk, core_half + 8.0),
        "CLK": _snap(pdk, -(core_half + 10.0)),
        "TAIL": _snap(pdk, core_half + 10.0),
        "VDD": _snap(pdk, -(core_half + 12.0)),
        "VSS": _snap(pdk, core_half + 12.0),
    }
    met3 = pdk.get_glayer("met3")
    met4 = pdk.get_glayer("met4")
    m3_width = pdk.get_grule("met3")["min_width"]
    m4_width = pdk.get_grule("met4")["min_width"]
    # A met3-to-met4 stack is 1.5 um square in SKY130.  Two-micron pitch
    # prevents neighbouring stacks from merging into over-width cuts.
    track_step = 2.0
    track_extents: list[float] = []

    for row_name, _, _ in unit_rows:
        entries = row_refs[row_name]
        gate_groups: dict[str, list[tuple[object, str]]] = defaultdict(list)
        sd_groups: dict[str, list[tuple[object, str]]] = defaultdict(list)
        for ref, device in entries:
            if device is None:
                body = "VDD" if row_name in ("reset", "latch_p") else "VSS"
                for terminal in ("gate", "source", "drain"):
                    (gate_groups if terminal == "gate" else sd_groups)[body].append(
                        (ref, terminal)
                    )
                continue
            gate_groups[device.gate].append((ref, "gate"))
            sd_groups[device.source].append((ref, "source"))
            sd_groups[device.drain].append((ref, "drain"))

        row_height = max(ref.ymax - ref.ymin for ref, _ in entries)
        for direction, groups in ((-1, gate_groups), (1, sd_groups)):
            ordered_nets = sorted(groups, key=lambda net: (abs(spine_x[net]), net))
            for index, net in enumerate(ordered_nets):
                track_y = _snap(
                    pdk,
                    row_y[row_name]
                    + direction * (row_height / 2 + 2.0 + index * track_step),
                )
                track_extents.append(track_y)
                branch_xs: list[float] = []
                for ref, terminal in groups[net]:
                    if terminal == "gate":
                        port = ref.ports["gate_S"]
                        branch_x = _snap(pdk, float(port.center[0]))
                    elif terminal == "source":
                        port = ref.ports["source_W"]
                        branch_x = _snap(pdk, float(port.center[0]) - 0.35)
                    else:
                        port = ref.ports["drain_E"]
                        branch_x = _snap(pdk, float(port.center[0]) + 0.35)
                    _metal_stub_to_track(
                        top,
                        pdk=pdk,
                        port=port,
                        branch_x=branch_x,
                        track_y=track_y,
                    )
                    branch_xs.append(branch_x)

                _add_rect(
                    top,
                    met3,
                    min(min(branch_xs), spine_x[net]),
                    track_y - m3_width / 2,
                    max(max(branch_xs), spine_x[net]),
                    track_y + m3_width / 2,
                )
                spine_via = top << via_stack(pdk, "met3", "met4")
                spine_via.move((spine_x[net], track_y))

    spine_bottom = _snap(pdk, min(track_extents) - 2.0)
    # Reserve a quiet body-tap channel above every reset signal escape track.
    spine_top = _snap(pdk, max(track_extents) + 10.0)
    for net, x in spine_x.items():
        _add_rect(
            top,
            met4,
            x - m4_width / 2,
            spine_bottom,
            x + m4_width / 2,
            spine_top,
        )

    # One continuous n-well covers all PMOS units and the n+ well-tap ring.
    pmos_refs = [
        ref for row_name in ("reset", "latch_p") for ref, _ in row_refs[row_name]
    ]
    pmos_xmin = min(ref.xmin for ref in pmos_refs) - 2.0
    pmos_xmax = max(ref.xmax for ref in pmos_refs) + 2.0
    pmos_ymin = min(ref.ymin for ref in pmos_refs) - 2.0
    nwell_tap = top << tapring(
        pdk,
        enclosed_rectangle=(pmos_xmax - pmos_xmin, 2.0),
        sdlayer="n+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
        # A bottom well-tap rail would cross the M2 gate drops from the PMOS
        # rows and silently short X/Y/CLK to VDD.  A continuous north tap gives
        # the common n-well an unambiguous body contact without crossing any
        # signal escape route.
        sides=(False, True, False, False),
    )
    nwell_tap.move(
        (
            _snap(pdk, (pmos_xmin + pmos_xmax) / 2),
            _snap(pdk, max(track_extents) + 4.0),
        )
    )
    nwell_margin = pdk.get_grule("nwell", "active_tap")["min_enclosure"]
    _add_rect(
        top,
        pdk.get_glayer("nwell"),
        nwell_tap.xmin - nwell_margin,
        pmos_ymin - nwell_margin,
        nwell_tap.xmax + nwell_margin,
        nwell_tap.ymax + nwell_margin,
    )

    # A p+ substrate guard ring surrounds devices and signal spines.
    guard_width = 2 * (core_half + 10.0)
    guard_height = spine_top - spine_bottom + 8.0
    substrate_tap = top << tapring(
        pdk,
        enclosed_rectangle=(guard_width, guard_height),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    substrate_tap.move((0.0, (spine_bottom + spine_top) / 2))

    # Tie both rings into their explicit supply spines through M3 branches.
    for ring, side, net, offset in (
        (nwell_tap, "N", "VDD", 0.0),
        (substrate_tap, "S", "VSS", 0.0),
    ):
        port = _ring_port(ring, side)
        track_y = _snap(pdk, float(port.center[1]) + offset)
        _metal_stub_to_track(
            top,
            pdk=pdk,
            port=port,
            branch_x=float(port.center[0]),
            track_y=track_y,
        )
        _add_rect(
            top,
            met3,
            min(float(port.center[0]), spine_x[net]),
            track_y - m3_width / 2,
            max(float(port.center[0]), spine_x[net]),
            track_y + m3_width / 2,
        )
        via = top << via_stack(pdk, "met3", "met4")
        via.move((spine_x[net], track_y))
        _add_rect(
            top,
            met4,
            spine_x[net] - m4_width / 2,
            min(track_y, spine_bottom),
            spine_x[net] + m4_width / 2,
            max(track_y, spine_top),
        )

    for pin in LAYOUT_PINS:
        x = spine_x[pin]
        orientation = 90
        centre = (x, spine_top)
        top.add_port(
            name=pin,
            center=centre,
            width=m4_width,
            orientation=orientation,
            layer=met4,
        )
        top.add_label(text=pin, position=centre, layer=met4)

    top.info["netlist"] = _canonical_netlist(
        devices=functional_devices, include_vss_pin=False
    )
    top.info["lvs_netlist"] = _canonical_netlist(
        devices=functional_devices,
        include_vss_pin=True,
        include_layout_dummies=True,
        layout_dummies=dummy_specs,
    )
    top.info["functional_devices"] = [device.__dict__ for device in functional_devices]
    top.info["device_scale"] = device_scale
    top.info["input_pair_scale"] = resolved_input_pair_scale
    top.info["input_pair_length_scale"] = input_pair_length_scale
    top.info["input_pair_area_scale"] = (
        resolved_input_pair_scale * input_pair_length_scale
    )
    top.info["unit_fingers"] = {
        device.name: int(device.width_um / finger_widths[device.name])
        for device in functional_devices
    }
    top.info["finger_width_um"] = finger_widths
    top.info["finger_length_um"] = finger_lengths
    top.info["row_patterns"] = {
        row_name: "".join(
            "A" if device.name == units[0].name else "B" for device in units
        )
        for row_name, _, units in unit_rows
        if row_name in {"input", "latch_n", "latch_p"}
    }
    top.info["dummy_devices"] = dummy_specs
    return component_snap_to_grid(top)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    strongarm_comparator(sky130_mapped_pdk).write_gds("strongarm_comparator.gds")
