"""Clock-gated differential current trim for the SAR StrongARM comparator.

The trim cell connects a binary-weighted NMOS sink bank to each StrongARM
output.  A selected branch conducts only while ``CLK`` is high, so static
trim codes add no direct-current path during precharge or idle.  Equal X/Y
banks keep the unselected parasitics symmetric; system calibration drives one
bank at a time and stores the resulting signed magnitude code.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.primitives.fet import nmos
from glayout.primitives.guardring import tapring
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist
from glayout.util.snap_to_grid import component_snap_to_grid


DEFAULT_TRIM_BITS = 4
TRIM_UNIT_WIDTH_UM = 0.42
TRIM_SELECTOR_LENGTH_UM = 100.0
TRIM_FOOTER_WIDTH_UM = 8.0
TRIM_FOOTER_LENGTH_UM = 0.15


def trim_pins(bits: int = DEFAULT_TRIM_BITS) -> tuple[str, ...]:
    if not isinstance(bits, int) or bits < 1:
        raise ValueError("trim bits must be a positive integer")
    order = tuple(reversed(range(bits)))
    return (
        "VSS",
        "CLK",
        "X",
        "Y",
        *(f"TRIM_X{bit}" for bit in order),
        *(f"TRIM_Y{bit}" for bit in order),
    )


TRIM_PINS = trim_pins()


@dataclass(frozen=True)
class TrimDevice:
    """One transistor in the canonical trim-DAC schematic."""

    name: str
    drain: str
    gate: str
    source: str
    bulk: str
    width_um: float
    length_um: float
    side: str
    bit: int | None
    role: str


def trim_devices(
    *,
    bits: int = DEFAULT_TRIM_BITS,
    unit_width_um: float = TRIM_UNIT_WIDTH_UM,
    selector_length_um: float = TRIM_SELECTOR_LENGTH_UM,
    footer_width_um: float = TRIM_FOOTER_WIDTH_UM,
    footer_length_um: float = TRIM_FOOTER_LENGTH_UM,
) -> tuple[TrimDevice, ...]:
    trim_pins(bits)
    if (
        min(
            unit_width_um,
            selector_length_um,
            footer_width_um,
            footer_length_um,
        )
        <= 0
    ):
        raise ValueError("trim transistor dimensions must be positive")
    devices: list[TrimDevice] = []
    for side in ("X", "Y"):
        tail = f"TRIM_{side}_TAIL"
        for bit in range(bits):
            devices.append(
                TrimDevice(
                    name=f"{side}{bit}",
                    drain=side,
                    gate=f"TRIM_{side}{bit}",
                    source=tail,
                    bulk="VSS",
                    width_um=unit_width_um * (1 << bit),
                    length_um=selector_length_um,
                    side=side,
                    bit=bit,
                    role="selector",
                )
            )
        devices.append(
            TrimDevice(
                name=f"CLK_{side}",
                drain=tail,
                gate="CLK",
                source="VSS",
                bulk="VSS",
                width_um=footer_width_um,
                length_um=footer_length_um,
                side=side,
                bit=None,
                role="footer",
            )
        )
    return tuple(devices)


def _canonical_netlist(
    *,
    bits: int = DEFAULT_TRIM_BITS,
    unit_width_um: float = TRIM_UNIT_WIDTH_UM,
    selector_length_um: float = TRIM_SELECTOR_LENGTH_UM,
    footer_width_um: float = TRIM_FOOTER_WIDTH_UM,
    footer_length_um: float = TRIM_FOOTER_LENGTH_UM,
) -> Netlist:
    pins = trim_pins(bits)
    devices = trim_devices(
        bits=bits,
        unit_width_um=unit_width_um,
        selector_length_um=selector_length_um,
        footer_width_um=footer_width_um,
        footer_length_um=footer_length_um,
    )
    lines = [f".subckt strongarm_offset_trim {' '.join(pins)}"]
    for device in devices:
        lines.append(
            f"X{device.name} {device.drain} {device.gate} "
            f"{device.source} {device.bulk} sky130_fd_pr__nfet_01v8 "
            f"L={device.length_um:g} W={device.width_um:g} nf=1 mult=1"
        )
    lines.append(".ends strongarm_offset_trim")
    return Netlist(
        circuit_name="strongarm_offset_trim",
        nodes=list(pins),
        source_netlist="\n".join(lines),
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
    return float(pdk.snap_to_2xgrid(value))


def _terminal_to_m3_track(
    component: Component,
    *,
    pdk: MappedPDK,
    port,
    branch_x: float,
    track_y: float,
) -> float:
    """Connect one physical M1 terminal to an abstract-M3 row track."""

    if pdk.layer_to_glayer(port.layer) != "met2":
        raise ValueError(f"expected an abstract met2 terminal, got {port.layer}")
    met2 = pdk.get_glayer("met2")
    width = pdk.get_grule("met2")["min_width"]
    x0, y0 = map(float, port.center)
    branch_x = _snap(pdk, branch_x)
    track_y = _snap(pdk, track_y)
    _add_rect(
        component,
        met2,
        min(x0, branch_x) - width / 2,
        y0 - width / 2,
        max(x0, branch_x) + width / 2,
        y0 + width / 2,
    )
    _add_rect(
        component,
        met2,
        branch_x - width / 2,
        min(y0, track_y) - width / 2,
        branch_x + width / 2,
        max(y0, track_y) + width / 2,
    )
    transition = component << via_stack(pdk, "met2", "met3")
    transition.move((branch_x, track_y))
    return branch_x


def _route_group(
    top: Component,
    *,
    pdk: MappedPDK,
    references: dict[str, object],
    devices: tuple[TrimDevice, ...],
    net: str,
    terminals: list[tuple[TrimDevice, str]],
    track_y: float,
    spine_x: float,
) -> None:
    branch_xs: list[float] = []
    for device, terminal in terminals:
        reference = references[device.name]
        if terminal == "gate":
            port = reference.ports["gate_S"]
            branch_x = float(port.center[0])
        elif terminal == "source":
            port = reference.ports["source_W"]
            branch_x = float(port.center[0]) - 0.45
        else:
            port = reference.ports["drain_E"]
            branch_x = float(port.center[0]) + 0.45
        branch_xs.append(
            _terminal_to_m3_track(
                top,
                pdk=pdk,
                port=port,
                branch_x=branch_x,
                track_y=track_y,
            )
        )
    met3 = pdk.get_glayer("met3")
    width = pdk.get_grule("met3")["min_width"]
    _add_rect(
        top,
        met3,
        min(*branch_xs, spine_x) - width / 2,
        track_y - width / 2,
        max(*branch_xs, spine_x) + width / 2,
        track_y + width / 2,
    )
    transition = top << via_stack(pdk, "met3", "met4")
    transition.move((_snap(pdk, spine_x), _snap(pdk, track_y)))


def _ring_port(reference, side: str):
    candidates = [
        port
        for name, port in reference.ports.items()
        if name.startswith(f"{side}_") and "top_met" in name
    ]
    if not candidates:
        raise RuntimeError(f"tap ring has no usable {side} metal port")
    return min(candidates, key=lambda port: abs(float(port.center[0])))


def strongarm_offset_trim(
    pdk: MappedPDK,
    *,
    bits: int = DEFAULT_TRIM_BITS,
    unit_width_um: float = TRIM_UNIT_WIDTH_UM,
    selector_length_um: float = TRIM_SELECTOR_LENGTH_UM,
    footer_width_um: float = TRIM_FOOTER_WIDTH_UM,
    footer_length_um: float = TRIM_FOOTER_LENGTH_UM,
) -> Component:
    """Generate the zero-static-power StrongARM differential trim bank."""

    devices = trim_devices(
        bits=bits,
        unit_width_um=unit_width_um,
        selector_length_um=selector_length_um,
        footer_width_um=footer_width_um,
        footer_length_um=footer_length_um,
    )
    pdk.activate()
    top = Component(name="strongarm_offset_trim")

    templates: dict[tuple[float, float], Component] = {}
    for width, length in sorted(
        {(device.width_um, device.length_um) for device in devices}
    ):
        template = nmos(
            pdk,
            width=width,
            length=length,
            fingers=1,
            multipliers=1,
            with_tie=False,
            with_dummy=False,
            with_substrate_tap=False,
            with_dnwell=False,
        )
        template.remove_layers([pdk.get_glayer("pwell")])
        templates[(width, length)] = template

    references: dict[str, object] = {}
    long_channel_layout = selector_length_um >= 20.0
    selector_pitch = 4.5
    selector_y = 8.0
    for device in devices:
        reference = top << templates[(device.width_um, device.length_um)]
        if device.role == "selector":
            if long_channel_layout:
                bit = int(device.bit)
                x = -18.0 + bit * 4.0 if device.side == "X" else 18.0 - bit * 4.0
                row = 2 * bit + (0 if device.side == "X" else 1)
                y = selector_y + row * 8.0
            else:
                distance = 6.0 + int(device.bit) * selector_pitch
                x = -distance if device.side == "X" else distance
                y = selector_y
        else:
            x = -8.0 if device.side == "X" else 8.0
            y = -10.0
        reference.move((_snap(pdk, x), _snap(pdk, y)))
        references[device.name] = reference

    if long_channel_layout:
        selector_references = [
            references[device.name] for device in devices if device.role == "selector"
        ]
        selector_left = min(float(reference.xmin) for reference in selector_references)
        spine_x = {
            "TRIM_X_TAIL": _snap(pdk, selector_left - 8.0),
            "TRIM_Y_TAIL": _snap(pdk, selector_left - 4.0),
            "CLK": -2.0,
            "VSS": 2.0,
            "X": 24.0,
            "Y": 28.0,
        }
    else:
        spine_x = {
            "X": -24.0,
            "TRIM_X_TAIL": -22.0,
            "CLK": -2.0,
            "VSS": 2.0,
            "TRIM_Y_TAIL": 22.0,
            "Y": 24.0,
        }
    for side in ("X", "Y"):
        for bit in range(bits):
            gate_port = references[f"{side}{bit}"].ports["gate_S"]
            spine_x[f"TRIM_{side}{bit}"] = _snap(pdk, float(gate_port.center[0]))

    terminal_groups: dict[str, list[tuple[TrimDevice, str]]] = defaultdict(list)
    for device in devices:
        terminal_groups[device.gate].append((device, "gate"))
        terminal_groups[device.source].append((device, "source"))
        terminal_groups[device.drain].append((device, "drain"))

    selector_devices = [device for device in devices if device.role == "selector"]
    footer_devices = [device for device in devices if device.role == "footer"]
    track_y = {
        "X": 16.0,
        "Y": 16.0,
        "TRIM_X_TAIL_SELECTOR": 13.0,
        "TRIM_Y_TAIL_SELECTOR": 13.0,
        "TRIM_X_TAIL_FOOTER": -2.0,
        "TRIM_Y_TAIL_FOOTER": 0.0 if long_channel_layout else -2.0,
        "VSS": -5.0,
        "CLK": -18.0,
    }
    for side in ("X", "Y"):
        tail = f"TRIM_{side}_TAIL"
        side_selectors = [device for device in selector_devices if device.side == side]
        if long_channel_layout:
            for device in side_selectors:
                reference = references[device.name]
                row_y = float(reference.ports["drain_E"].center[1])
                _route_group(
                    top,
                    pdk=pdk,
                    references=references,
                    devices=devices,
                    net=side,
                    terminals=[(device, "drain")],
                    track_y=row_y,
                    spine_x=spine_x[side],
                )
                _route_group(
                    top,
                    pdk=pdk,
                    references=references,
                    devices=devices,
                    net=tail,
                    terminals=[(device, "source")],
                    track_y=row_y,
                    spine_x=spine_x[tail],
                )
        else:
            _route_group(
                top,
                pdk=pdk,
                references=references,
                devices=devices,
                net=side,
                terminals=[(device, "drain") for device in side_selectors],
                track_y=track_y[side],
                spine_x=spine_x[side],
            )
            _route_group(
                top,
                pdk=pdk,
                references=references,
                devices=devices,
                net=tail,
                terminals=[(device, "source") for device in side_selectors],
                track_y=track_y[f"{tail}_SELECTOR"],
                spine_x=spine_x[tail],
            )
        footer = next(device for device in footer_devices if device.side == side)
        _route_group(
            top,
            pdk=pdk,
            references=references,
            devices=devices,
            net=tail,
            terminals=[(footer, "drain")],
            track_y=track_y[f"{tail}_FOOTER"],
            spine_x=spine_x[tail],
        )

    _route_group(
        top,
        pdk=pdk,
        references=references,
        devices=devices,
        net="VSS",
        terminals=[(device, "source") for device in footer_devices],
        track_y=track_y["VSS"],
        spine_x=spine_x["VSS"],
    )
    _route_group(
        top,
        pdk=pdk,
        references=references,
        devices=devices,
        net="CLK",
        terminals=[(device, "gate") for device in footer_devices],
        track_y=track_y["CLK"],
        spine_x=spine_x["CLK"],
    )

    met4 = pdk.get_glayer("met4")
    met4_width = pdk.get_grule("met4")["min_width"]
    handoff_y = 1.0
    for side in ("X", "Y"):
        for bit in range(bits):
            net = f"TRIM_{side}{bit}"
            port = references[f"{side}{bit}"].ports["gate_S"]
            x, y = map(float, port.center)
            if abs(x - spine_x[net]) > 1e-6:
                raise RuntimeError(f"{net} gate is not aligned to its trim spine")
            if long_channel_layout:
                selector = next(
                    device for device in devices if device.name == f"{side}{bit}"
                )
                _route_group(
                    top,
                    pdk=pdk,
                    references=references,
                    devices=devices,
                    net=net,
                    terminals=[(selector, "gate")],
                    track_y=y,
                    spine_x=x,
                )
            else:
                _add_rect(
                    top,
                    pdk.get_glayer("met2"),
                    x - pdk.get_grule("met2")["min_width"] / 2,
                    min(y, handoff_y),
                    x + pdk.get_grule("met2")["min_width"] / 2,
                    max(y, handoff_y),
                )
                transition_23 = top << via_stack(pdk, "met2", "met3")
                transition_23.move((x, handoff_y))
                transition_34 = top << via_stack(pdk, "met3", "met4")
                transition_34.move((x, handoff_y))

    device_xmin = min(float(reference.xmin) for reference in references.values())
    device_xmax = max(float(reference.xmax) for reference in references.values())
    device_ymin = min(float(reference.ymin) for reference in references.values())
    device_ymax = max(float(reference.ymax) for reference in references.values())
    ring_width = max(50.0, 2 * max(abs(device_xmin), abs(device_xmax)) + 8.0)
    ring_height = max(44.0, device_ymax - device_ymin + 8.0)
    ring_center_y = (device_ymin + device_ymax) / 2
    substrate_tap = top << tapring(
        pdk,
        enclosed_rectangle=(ring_width, ring_height),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    substrate_tap.move((0.0, _snap(pdk, ring_center_y)))
    ring_port = _ring_port(substrate_tap, "S")
    ring_track_y = _snap(pdk, float(ring_port.center[1]))
    ring_branch = _terminal_to_m3_track(
        top,
        pdk=pdk,
        port=ring_port,
        branch_x=float(ring_port.center[0]),
        track_y=ring_track_y,
    )
    _add_rect(
        top,
        pdk.get_glayer("met3"),
        min(ring_branch, spine_x["VSS"]),
        ring_track_y - pdk.get_grule("met3")["min_width"] / 2,
        max(ring_branch, spine_x["VSS"]),
        ring_track_y + pdk.get_grule("met3")["min_width"] / 2,
    )
    ring_transition = top << via_stack(pdk, "met3", "met4")
    ring_transition.move((spine_x["VSS"], ring_track_y))

    spine_top = _snap(pdk, max(26.0, device_ymax + 10.0))
    spine_bottom = {
        "X": track_y["X"],
        "Y": track_y["Y"],
        "CLK": track_y["CLK"],
        "VSS": ring_track_y,
        "TRIM_X_TAIL": track_y["TRIM_X_TAIL_FOOTER"],
        "TRIM_Y_TAIL": track_y["TRIM_Y_TAIL_FOOTER"],
    }
    for side in ("X", "Y"):
        tail = f"TRIM_{side}_TAIL"
        tail_top = track_y[f"{tail}_SELECTOR"]
        if long_channel_layout:
            tail_top = max(
                float(references[device.name].ports["source_W"].center[1])
                for device in selector_devices
                if device.side == side
            )
        _add_rect(
            top,
            met4,
            spine_x[tail] - met4_width / 2,
            track_y[f"{tail}_FOOTER"],
            spine_x[tail] + met4_width / 2,
            tail_top,
        )
        for bit in range(bits):
            if long_channel_layout:
                spine_bottom[f"TRIM_{side}{bit}"] = float(
                    references[f"{side}{bit}"].ports["gate_S"].center[1]
                )
            else:
                spine_bottom[f"TRIM_{side}{bit}"] = handoff_y

        if long_channel_layout:
            side_rows = [
                float(references[device.name].ports["drain_E"].center[1])
                for device in selector_devices
                if device.side == side
            ]
            spine_bottom[side] = min(side_rows)
            spine_bottom[tail] = track_y[f"{tail}_FOOTER"]

    for net in trim_pins(bits):
        x = spine_x[net]
        _add_rect(
            top,
            met4,
            x - met4_width / 2,
            spine_bottom[net],
            x + met4_width / 2,
            spine_top,
        )
        top.add_port(
            name=net,
            center=(x, spine_top),
            width=met4_width,
            orientation=90,
            layer=met4,
        )
        # Keep the Magic port label inside the terminal metal.  A label
        # exactly on the top edge can quantize onto empty space and produce an
        # otherwise-correct extracted subcircuit with no public pins.
        top.add_label(text=net, position=(x, spine_top - 1.0), layer=met4)

    netlist = _canonical_netlist(
        bits=bits,
        unit_width_um=unit_width_um,
        selector_length_um=selector_length_um,
        footer_width_um=footer_width_um,
        footer_length_um=footer_length_um,
    )
    top.info["netlist"] = netlist
    top.info["lvs_netlist"] = netlist
    top.info["trim_bits_per_side"] = bits
    top.info["signed_code_range"] = [-(1 << bits) + 1, (1 << bits) - 1]
    top.info["unit_width_um"] = unit_width_um
    top.info["selector_length_um"] = selector_length_um
    top.info["footer_width_um"] = footer_width_um
    top.info["footer_length_um"] = footer_length_um
    top.info["device_count"] = len(devices)
    top.info["device_kinds"] = dict(Counter("nfet" for _ in devices))
    top.info["functional_devices"] = [device.__dict__ for device in devices]
    top.info["idle_static_path"] = False
    top.info["calibration_contract"] = "drive only one binary-weighted side at a time"
    return component_snap_to_grid(top)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    strongarm_offset_trim(sky130_mapped_pdk).write_gds("strongarm_offset_trim.gds")
