"""Matched two-stage clocked dynamic preamplifier for the SKY130 SAR ADC.

Each gain stage combines diode PMOS loads, bounded cross-coupled PMOS gain,
clock-low PMOS output reset, a differential NMOS input pair, and a clocked
NMOS tail.  Reset removes inter-bit state while the two stages isolate the
CDAC and provide enough gain to swamp the downstream StrongARM offset.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.primitives.fet import nmos, pmos
from glayout.primitives.guardring import tapring
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist
from glayout.util.comp_utils import evaluate_bbox
from glayout.util.snap_to_grid import component_snap_to_grid

from .strongarm import _add_rect, _metal_stub_to_track, _ring_port, _snap


DYNAMIC_PREAMP_PINS: tuple[str, ...] = (
    "VDD",
    "OUTP",
    "OUTN",
    "CLK",
    "VINP",
    "VINN",
    "VSS",
)


@dataclass(frozen=True)
class PreampDevice:
    name: str
    kind: str
    drain: str
    gate: str
    source: str
    bulk: str
    width_um: float
    length_um: float


def preamp_devices(
    *,
    input_width_um: float = 2.0,
    load_width_um: float = 2.0,
    feedback_width_um: float = 1.0,
    reset_width_um: float = 1.0,
    tail_width_um: float = 1.0,
    length_um: float = 0.5,
) -> tuple[PreampDevice, ...]:
    """Return the canonical devices for both resettable gain stages."""

    dimensions = (
        input_width_um,
        load_width_um,
        feedback_width_um,
        reset_width_um,
        tail_width_um,
        length_um,
    )
    if any(value <= 0 for value in dimensions):
        raise ValueError("preamplifier device dimensions must be positive")
    if any(width % 0.5 for width in dimensions[:-1]):
        raise ValueError("preamplifier widths must use integral 0.5 um units")
    if int(input_width_um) % 2 or int(load_width_um) % 2:
        raise ValueError("matched input and load widths must have even finger counts")
    devices: list[PreampDevice] = []
    for stage, (vinp, vinn, outp, outn) in enumerate(
        (("VINP", "VINN", "MIDP", "MIDN"), ("MIDP", "MIDN", "OUTP", "OUTN")),
        start=1,
    ):
        tail = f"TAIL{stage}"
        prefix = f"S{stage}_"
        devices.extend(
            (
                PreampDevice(prefix + "LP", "pfet", outp, outp, "VDD", "VDD", load_width_um, length_um),
                PreampDevice(prefix + "LN", "pfet", outn, outn, "VDD", "VDD", load_width_um, length_um),
                PreampDevice(prefix + "CP", "pfet", outp, outn, "VDD", "VDD", feedback_width_um, length_um),
                PreampDevice(prefix + "CN", "pfet", outn, outp, "VDD", "VDD", feedback_width_um, length_um),
                PreampDevice(prefix + "RP", "pfet", outp, "CLK", "VDD", "VDD", reset_width_um, length_um),
                PreampDevice(prefix + "RN", "pfet", outn, "CLK", "VDD", "VDD", reset_width_um, length_um),
                # Cross each inverting drain pair so the named stage is non-inverting.
                PreampDevice(prefix + "INP", "nfet", outn, vinp, tail, "VSS", input_width_um, length_um),
                PreampDevice(prefix + "INN", "nfet", outp, vinn, tail, "VSS", input_width_um, length_um),
                PreampDevice(prefix + "TAIL", "nfet", tail, "CLK", "VSS", "VSS", tail_width_um, length_um),
            )
        )
    return tuple(devices)


def _canonical_netlist(
    devices: tuple[PreampDevice, ...],
    *,
    include_layout_dummies: bool = False,
    layout_dummies: list[dict[str, object]] | None = None,
) -> Netlist:
    lines = [f".subckt clocked_dynamic_preamp {' '.join(DYNAMIC_PREAMP_PINS)}"]
    for device in devices:
        lines.append(
            f"X{device.name} {device.drain} {device.gate} {device.source} "
            f"{device.bulk} sky130_fd_pr__{device.kind}_01v8 "
            f"L={device.length_um:g} W={device.width_um:g} nf=1 mult=1"
        )
    if include_layout_dummies:
        for index, dummy in enumerate(layout_dummies or []):
            body = str(dummy["body"])
            lines.append(
                f"XDUMMY_{index} {body} {body} {body} {body} "
                f"sky130_fd_pr__{dummy['kind']}_01v8 "
                f"L={float(dummy['length_um']):g} "
                f"W={float(dummy['width_um']):g} nf=1 mult=1"
            )
    lines.append(".ends clocked_dynamic_preamp")
    return Netlist(
        circuit_name="clocked_dynamic_preamp",
        nodes=list(DYNAMIC_PREAMP_PINS),
        source_netlist="\n".join(lines),
    )


def _abba(left: PreampDevice, right: PreampDevice) -> list[PreampDevice]:
    fingers = int(left.width_um)
    if left.width_um != right.width_um or fingers != left.width_um or fingers % 2:
        raise ValueError("ABBA devices must have equal positive even finger counts")
    return [device for _ in range(fingers // 2) for device in (left, right, right, left)]


def _centred_units(
    devices: tuple[PreampDevice, ...], *, unit_width_um: float
) -> list[PreampDevice]:
    """Place half of every device then mirror it for zero first moment."""

    half: list[PreampDevice] = []
    for device in devices:
        fingers = round(device.width_um / unit_width_um)
        if fingers <= 0 or fingers % 2:
            raise ValueError("centred devices require a positive even unit count")
        half.extend([device] * (fingers // 2))
    return half + list(reversed(half))


def clocked_dynamic_preamp(
    pdk: MappedPDK,
    *,
    input_width_um: float = 2.0,
    load_width_um: float = 2.0,
    feedback_width_um: float = 1.0,
    reset_width_um: float = 1.0,
    tail_width_um: float = 1.0,
    length_um: float = 0.5,
) -> Component:
    """Generate the resettable two-stage differential preamplifier."""

    devices = preamp_devices(
        input_width_um=input_width_um,
        load_width_um=load_width_um,
        feedback_width_um=feedback_width_um,
        reset_width_um=reset_width_um,
        tail_width_um=tail_width_um,
        length_um=length_um,
    )
    by_name = {device.name: device for device in devices}
    rows = (
        (
            "s1_load",
            "pfet",
            0.5,
            _centred_units(
                tuple(by_name[f"S1_{name}"] for name in ("LP", "LN", "CP", "CN", "RP", "RN")),
                unit_width_um=0.5,
            ),
        ),
        (
            "s2_load",
            "pfet",
            0.5,
            _centred_units(
                tuple(by_name[f"S2_{name}"] for name in ("LP", "LN", "CP", "CN", "RP", "RN")),
                unit_width_um=0.5,
            ),
        ),
        ("s1_input", "nfet", 1.0, _abba(by_name["S1_INP"], by_name["S1_INN"])),
        ("s1_tail", "nfet", 0.5, [by_name["S1_TAIL"]] * round(tail_width_um / 0.5)),
        ("s2_input", "nfet", 1.0, _abba(by_name["S2_INP"], by_name["S2_INN"])),
        ("s2_tail", "nfet", 0.5, [by_name["S2_TAIL"]] * round(tail_width_um / 0.5)),
    )
    row_y = {
        "s1_load": 32.0,
        "s2_load": 14.0,
        "s1_input": -8.0,
        "s1_tail": -26.0,
        "s2_input": -46.0,
        "s2_tail": -64.0,
    }

    pdk.activate()
    top = Component(name="clocked_dynamic_preamp")
    primitives = {}
    for row_name, kind, unit_width, _ in rows:
        generator = nmos if kind == "nfet" else pmos
        options = {
            "width": unit_width,
            "length": length_um,
            "fingers": 1,
            "multipliers": 1,
            "with_tie": False,
            "with_dummy": False,
            "with_substrate_tap": False,
        }
        if kind == "nfet":
            options["with_dnwell"] = False
        primitive = generator(pdk, **options)
        if kind == "nfet":
            primitive.remove_layers([pdk.get_glayer("pwell")])
        primitives[row_name] = primitive
    pitch = pdk.snap_to_2xgrid(
        max(evaluate_bbox(primitive)[0] for primitive in primitives.values()) + 0.8
    )

    row_refs: dict[str, list[tuple[object, PreampDevice | None]]] = {}
    dummy_specs: list[dict[str, object]] = []
    for row_name, kind, unit_width, units in rows:
        entries: list[tuple[object, PreampDevice | None]] = []
        count = len(units) + 2
        for index in range(count):
            ref = top << primitives[row_name]
            ref.move(
                (
                    _snap(pdk, (index - (count - 1) / 2) * pitch),
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
                        "width_um": unit_width,
                        "length_um": length_um,
                    }
                )
            else:
                entries.append((ref, units[index - 1]))
        row_refs[row_name] = entries

    core_xmin = min(ref.xmin for entries in row_refs.values() for ref, _ in entries)
    core_xmax = max(ref.xmax for entries in row_refs.values() for ref, _ in entries)
    core_half = max(abs(core_xmin), abs(core_xmax))
    spine_x = {
        "OUTP": _snap(pdk, -(core_half + 4.0)),
        "OUTN": _snap(pdk, core_half + 4.0),
        "VINP": _snap(pdk, -(core_half + 6.0)),
        "VINN": _snap(pdk, core_half + 6.0),
        "CLK": _snap(pdk, -(core_half + 8.0)),
        "TAIL1": _snap(pdk, core_half + 8.0),
        "MIDP": _snap(pdk, -(core_half + 10.0)),
        "MIDN": _snap(pdk, core_half + 10.0),
        "TAIL2": _snap(pdk, core_half + 12.0),
        "VDD": _snap(pdk, -(core_half + 12.0)),
        "VSS": _snap(pdk, core_half + 14.0),
    }
    met3 = pdk.get_glayer("met3")
    met4 = pdk.get_glayer("met4")
    m3_width = pdk.get_grule("met3")["min_width"]
    m4_width = pdk.get_grule("met4")["min_width"]
    track_extents: list[float] = []

    for row_name, _, _, _ in rows:
        entries = row_refs[row_name]
        gate_groups: dict[str, list[tuple[object, str]]] = defaultdict(list)
        sd_groups: dict[str, list[tuple[object, str]]] = defaultdict(list)
        for ref, device in entries:
            if device is None:
                body = "VDD" if row_name.endswith("load") else "VSS"
                gate_groups[body].append((ref, "gate"))
                sd_groups[body].extend(((ref, "source"), (ref, "drain")))
            else:
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
                    + direction * (row_height / 2 + 2.0 + index * 2.0),
                )
                track_extents.append(track_y)
                branch_xs = []
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
                transition = top << via_stack(pdk, "met3", "met4")
                transition.move((spine_x[net], track_y))

    spine_bottom = _snap(pdk, min(track_extents) - 2.0)
    spine_top = _snap(pdk, max(track_extents) + 10.0)
    for x in spine_x.values():
        _add_rect(
            top,
            met4,
            x - m4_width / 2,
            spine_bottom,
            x + m4_width / 2,
            spine_top,
        )

    pmos_refs = [
        ref
        for row_name in ("s1_load", "s2_load")
        for ref, _ in row_refs[row_name]
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
        sides=(False, True, False, False),
    )
    nwell_tap.move((0.0, _snap(pdk, max(track_extents) + 4.0)))
    nwell_margin = pdk.get_grule("nwell", "active_tap")["min_enclosure"]
    _add_rect(
        top,
        pdk.get_glayer("nwell"),
        nwell_tap.xmin - nwell_margin,
        pmos_ymin - nwell_margin,
        nwell_tap.xmax + nwell_margin,
        nwell_tap.ymax + nwell_margin,
    )

    substrate_tap = top << tapring(
        pdk,
        enclosed_rectangle=(2 * (core_half + 8.0), spine_top - spine_bottom + 8.0),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    substrate_tap.move((0.0, (spine_bottom + spine_top) / 2))
    for ring, side, net in (
        (nwell_tap, "N", "VDD"),
        (substrate_tap, "S", "VSS"),
    ):
        port = _ring_port(ring, side)
        track_y = _snap(pdk, float(port.center[1]))
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
        transition = top << via_stack(pdk, "met3", "met4")
        transition.move((spine_x[net], track_y))
        _add_rect(
            top,
            met4,
            spine_x[net] - m4_width / 2,
            min(track_y, spine_bottom),
            spine_x[net] + m4_width / 2,
            max(track_y, spine_top),
        )

    for pin in DYNAMIC_PREAMP_PINS:
        centre = (spine_x[pin], spine_top)
        top.add_port(
            name=pin,
            center=centre,
            width=m4_width,
            orientation=90,
            layer=met4,
        )
        top.add_label(text=pin, position=centre, layer=met4)

    top.info["netlist"] = _canonical_netlist(devices)
    top.info["lvs_netlist"] = _canonical_netlist(
        devices,
        include_layout_dummies=True,
        layout_dummies=dummy_specs,
    )
    top.info["functional_devices"] = [device.__dict__ for device in devices]
    top.info["device_count"] = len(devices)
    top.info["clock_gated"] = True
    top.info["idle_static_path"] = False
    top.info["polarity"] = "OUTP_gt_OUTN_when_VINP_gt_VINN"
    top.info["input_gate_area_um2"] = input_width_um * length_um
    top.info["input_width_um"] = input_width_um
    top.info["load_width_um"] = load_width_um
    top.info["feedback_width_um"] = feedback_width_um
    top.info["reset_width_um"] = reset_width_um
    top.info["tail_width_um"] = tail_width_um
    top.info["length_um"] = length_um
    top.info["stage_count"] = 2
    top.info["clock_low_output_reset"] = True
    top.info["bounded_positive_feedback"] = True
    top.info["dummy_devices"] = dummy_specs
    top.info["row_patterns"] = {
        "s1_load": "zero_first_moment_mirrored",
        "s2_load": "zero_first_moment_mirrored",
        "s1_input": "ABBA",
        "s2_input": "ABBA",
    }
    return component_snap_to_grid(top)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    clocked_dynamic_preamp(sky130_mapped_pdk).write_gds(
        "clocked_dynamic_preamp.gds"
    )
