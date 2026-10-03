"""Physical bottom-plate switch matrix for the 8-/10-bit SKY130 SAR CDAC.

The passive MIM array is intentionally a separate hard macro.  This block
implements the active circuitry between its ``BP9 .. BP0, BPT`` pins and the
sample/reference rails:

* one shared ``PHI_S`` inverter produces ``PHI_SB``;
* one slice per bit selects VIN_S while sampling, then VREF or VSS while converting;
* the termination slice samples VIN_S and returns to VSS during conversion.

Every physical device comes from the immutable schematic table below.  M2
terminal branches cross orthogonal M3 local tracks and global buses without
shorting; bit-control and bottom-plate pins terminate at their local M3 tracks.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.primitives.fet import nmos, pmos
from glayout.primitives.guardring import tapring
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist


DEFAULT_SAR_BITS = 10
SUPPORTED_SAR_BITS = (8, 10)


def bit_order(bits: int = DEFAULT_SAR_BITS) -> tuple[int, ...]:
    """Return MSB-first bit indices for a supported switch matrix."""

    if bits not in SUPPORTED_SAR_BITS:
        raise ValueError(f"bits must be one of {SUPPORTED_SAR_BITS}; got {bits}")
    return tuple(range(bits - 1, -1, -1))


def switch_matrix_pins(bits: int = DEFAULT_SAR_BITS) -> tuple[str, ...]:
    """Return the public switch-matrix pin order for ``bits`` resolution."""

    order = bit_order(bits)
    return (
        ("VDD", "VSS", "VREF", "VIN_S", "PHI_S", "PHI_SB")
        + tuple(f"SW{bit}" for bit in order)
        + tuple(f"BP{bit}" for bit in order)
        + ("BPT",)
    )


def _switch_matrix_name(bits: int) -> str:
    return (
        "sar_cdac_switch_matrix"
        if bits == DEFAULT_SAR_BITS
        else f"sar_cdac_switch_matrix_{bits}bit"
    )


BIT_ORDER: tuple[int, ...] = bit_order()
SWITCH_MATRIX_PINS: tuple[str, ...] = switch_matrix_pins()


@dataclass(frozen=True)
class SwitchDevice:
    """One transistor in the canonical switch-matrix schematic."""

    name: str
    kind: str
    drain: str
    gate: str
    source: str
    bulk: str
    width_um: float
    length_um: float = 0.15
    bit: int | None = None
    slot: str = ""


def _canonical_devices(
    bits: int = DEFAULT_SAR_BITS,
) -> tuple[SwitchDevice, ...]:
    order = bit_order(bits)
    devices = [
        SwitchDevice(
            "INVPHI_N",
            "nfet",
            "PHI_SB",
            "PHI_S",
            "VSS",
            "VSS",
            1.0,
            slot="phase",
        ),
        SwitchDevice(
            "INVPHI_P",
            "pfet",
            "PHI_SB",
            "PHI_S",
            "VDD",
            "VDD",
            2.0,
            slot="phase",
        ),
    ]
    for bit in order:
        bp = f"BP{bit}"
        sw = f"SW{bit}"
        sw_b = f"SW{bit}_B"
        mid = f"MID{bit}"
        devices.extend(
            (
                SwitchDevice(
                    f"MINV{bit}_N",
                    "nfet",
                    sw_b,
                    sw,
                    "VSS",
                    "VSS",
                    1.0,
                    bit=bit,
                    slot="invert",
                ),
                SwitchDevice(
                    f"MINV{bit}_P",
                    "pfet",
                    sw_b,
                    sw,
                    "VDD",
                    "VDD",
                    2.0,
                    bit=bit,
                    slot="invert",
                ),
                SwitchDevice(
                    f"MSW{bit}_REF",
                    "pfet",
                    bp,
                    sw_b,
                    "VREF",
                    "VDD",
                    2.0,
                    bit=bit,
                    slot="reference",
                ),
                SwitchDevice(
                    f"MSW{bit}_GND1",
                    "nfet",
                    bp,
                    sw_b,
                    mid,
                    "VSS",
                    1.0,
                    bit=bit,
                    slot="reference",
                ),
                SwitchDevice(
                    f"MSW{bit}_GND2",
                    "nfet",
                    mid,
                    "PHI_SB",
                    "VSS",
                    "VSS",
                    1.0,
                    bit=bit,
                    slot="ground",
                ),
                SwitchDevice(
                    f"MSW{bit}_SMPN",
                    "nfet",
                    bp,
                    "PHI_S",
                    "VIN_S",
                    "VSS",
                    4.0,
                    bit=bit,
                    slot="sample",
                ),
                SwitchDevice(
                    f"MSW{bit}_SMPP",
                    "pfet",
                    bp,
                    "PHI_SB",
                    "VIN_S",
                    "VDD",
                    4.0,
                    bit=bit,
                    slot="sample",
                ),
            )
        )
    devices.extend(
        (
            SwitchDevice(
                "MSWT_GND",
                "nfet",
                "BPT",
                "PHI_SB",
                "VSS",
                "VSS",
                1.0,
                slot="termination_ground",
            ),
            SwitchDevice(
                "MSWT_SMPN",
                "nfet",
                "BPT",
                "PHI_S",
                "VIN_S",
                "VSS",
                4.0,
                slot="termination_sample",
            ),
            SwitchDevice(
                "MSWT_SMPP",
                "pfet",
                "BPT",
                "PHI_SB",
                "VIN_S",
                "VDD",
                4.0,
                slot="termination_sample",
            ),
        )
    )
    return tuple(devices)


SWITCH_DEVICES: tuple[SwitchDevice, ...] = _canonical_devices()


def _canonical_netlist(bits: int = DEFAULT_SAR_BITS) -> Netlist:
    circuit_name = _switch_matrix_name(bits)
    pins = switch_matrix_pins(bits)
    lines = [f".subckt {circuit_name} {' '.join(pins)}"]
    for device in _canonical_devices(bits):
        lines.append(
            f"X{device.name} {device.drain} {device.gate} "
            f"{device.source} {device.bulk} sky130_fd_pr__{device.kind}_01v8 "
            f"L={device.length_um:g} W={device.width_um:g} nf=1 mult=1"
        )
    lines.append(f".ends {circuit_name}")
    return Netlist(
        circuit_name=circuit_name,
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


def _metal_stub_to_track(
    component: Component,
    *,
    pdk: MappedPDK,
    port,
    branch_x: float,
    track_y: float,
) -> float:
    """Connect one M1 device terminal to an M3 track and return its branch x.

    Glayout's device ports are on its abstract ``met2`` layer (SKY130 M1).
    Only the short, outward-facing terminal stub remains on that layer.  The
    vertical branch rises to abstract ``met3`` (SKY130 M2), then meets the
    horizontal abstract ``met4`` (SKY130 M3) track through a second via.  This
    orthogonal layer assignment avoids both M1 spacing errors at the device and
    unintended shorts where branches cross unrelated tracks.
    """

    if pdk.layer_to_glayer(port.layer) != "met2":
        raise ValueError(f"expected an M2 device terminal, got {port.layer}")
    met2 = pdk.get_glayer("met2")
    met3 = pdk.get_glayer("met3")
    met2_width = pdk.get_grule("met2")["min_width"]
    met3_width = pdk.get_grule("met3")["min_width"]
    x0, y0 = map(float, port.center)
    branch_x = _snap(pdk, branch_x)
    track_y = _snap(pdk, track_y)
    _add_rect(
        component,
        met2,
        min(x0, branch_x) - met2_width / 2,
        y0 - met2_width / 2,
        max(x0, branch_x) + met2_width / 2,
        y0 + met2_width / 2,
    )
    lower_transition = component << via_stack(pdk, "met2", "met3")
    lower_transition.move((branch_x, y0))
    _add_rect(
        component,
        met3,
        branch_x - met3_width / 2,
        min(y0, track_y) - met3_width / 2,
        branch_x + met3_width / 2,
        max(y0, track_y) + met3_width / 2,
    )
    upper_transition = component << via_stack(pdk, "met3", "met4")
    upper_transition.move((branch_x, track_y))
    return branch_x


def _ring_port(reference, side: str):
    candidates = [
        port
        for name, port in reference.ports.items()
        if name.startswith(f"{side}_") and "top_met" in name
    ]
    if not candidates:
        raise RuntimeError(f"tap ring has no usable {side} metal port")
    return min(candidates, key=lambda port: abs(float(port.center[0])))


def _device_target(device: SwitchDevice, bits: int) -> tuple[float, float]:
    """Return the deterministic placement centre for a schematic device."""

    p_y = 11.0
    n_y = -11.0
    if device.slot == "phase":
        return 0.0, p_y if device.kind == "pfet" else n_y

    first_slice_x = 16.0
    slice_pitch = 14.0
    if device.bit is None:
        centre_x = first_slice_x + bits * slice_pitch
    else:
        centre_x = first_slice_x + (bits - 1 - device.bit) * slice_pitch
    offsets = {
        "invert": -4.5,
        "reference": -1.5,
        "ground": 1.5,
        "sample": 4.5,
        "termination_ground": 1.5,
        "termination_sample": 4.5,
    }
    return centre_x + offsets[device.slot], p_y if device.kind == "pfet" else n_y


def _local_track(net: str) -> float | None:
    if net.startswith("BP"):
        return 0.0
    if net.startswith("SW") and net.endswith("_B"):
        return 3.5
    if net.startswith("MID"):
        return -3.5
    if net.startswith("SW"):
        return -6.5
    return None


def _external_spine_x(net: str, bits: int) -> float:
    first_slice_x = 16.0
    slice_pitch = 14.0
    if net == "BPT":
        return first_slice_x + bits * slice_pitch + 5.0
    if net.startswith("BP"):
        bit = int(net[2:])
        return first_slice_x + (bits - 1 - bit) * slice_pitch + 5.0
    if net.startswith("SW") and not net.endswith("_B"):
        bit = int(net[2:])
        return first_slice_x + (bits - 1 - bit) * slice_pitch - 5.0
    raise ValueError(f"{net} has no external M4 spine")


def sar_cdac_switch_matrix(
    pdk: MappedPDK,
    *,
    bits: int = DEFAULT_SAR_BITS,
    length_um: float = 0.15,
) -> Component:
    """Generate the exact active switch bank for an 8- or 10-bit CDAC.

    The 10-bit default retains its 75-device and pin-compatible implementation.
    Passing ``bits=8`` produces a 61-device macro named
    ``sar_cdac_switch_matrix_8bit`` with ``SW7..SW0`` and ``BP7..BP0``.
    ``PHI_SB`` remains exposed for the bootstrap and top-plate reset macros.
    """

    if length_um != 0.15:
        raise ValueError("this SKY130 switch matrix is qualified only at L=0.15 um")
    devices = _canonical_devices(bits)
    pdk.activate()
    top = Component(name=_switch_matrix_name(bits))

    widths = sorted({device.width_um for device in devices})
    templates: dict[tuple[str, float], Component] = {}
    for kind in ("nfet", "pfet"):
        factory = nmos if kind == "nfet" else pmos
        for width in widths:
            device_options = {
                "width": width,
                "length": length_um,
                "fingers": 1,
                "multipliers": 1,
                "with_tie": False,
                "with_dummy": False,
                "with_substrate_tap": False,
            }
            if kind == "nfet":
                device_options["with_dnwell"] = False
            else:
                device_options["dnwell"] = False
            template = factory(
                pdk,
                **device_options,
            )
            if kind == "nfet":
                # Standard 1.8 V NMOS devices sit directly in p-substrate.
                template.remove_layers([pdk.get_glayer("pwell")])
            templates[(kind, width)] = template

    references: dict[str, object] = {}
    pmos_references = []
    for device in devices:
        reference = top << templates[(device.kind, device.width_um)]
        reference.move(
            tuple(_snap(pdk, value) for value in _device_target(device, bits))
        )
        references[device.name] = reference
        if device.kind == "pfet":
            pmos_references.append(reference)

    core_xmin = min(reference.xmin for reference in references.values())
    core_xmax = max(reference.xmax for reference in references.values())
    core_centre_x = (core_xmin + core_xmax) / 2

    # One continuous n-well and north n+ tap cover every PMOS switch.
    pmos_xmin = min(reference.xmin for reference in pmos_references) - 2.0
    pmos_xmax = max(reference.xmax for reference in pmos_references) + 2.0
    pmos_ymin = min(reference.ymin for reference in pmos_references) - 2.0
    nwell_tap = top << tapring(
        pdk,
        enclosed_rectangle=(pmos_xmax - pmos_xmin, 2.0),
        sdlayer="n+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
        sides=(False, True, False, False),
    )
    nwell_tap.move((_snap(pdk, (pmos_xmin + pmos_xmax) / 2), 30.0))
    nwell_margin = pdk.get_grule("nwell", "active_tap")["min_enclosure"]
    _add_rect(
        top,
        pdk.get_glayer("nwell"),
        nwell_tap.xmin - nwell_margin,
        pmos_ymin - nwell_margin,
        nwell_tap.xmax + nwell_margin,
        nwell_tap.ymax + nwell_margin,
    )

    # The p+ guard ring supplies a real VSS substrate contact around the bank.
    substrate_tap = top << tapring(
        pdk,
        enclosed_rectangle=(core_xmax - core_xmin + 18.0, 70.0),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    substrate_tap.move((_snap(pdk, core_centre_x), 0.0))

    met4 = pdk.get_glayer("met4")
    m4_width = pdk.get_grule("met4")["min_width"]
    global_tracks = {
        "VDD": 25.0,
        "VREF": 22.0,
        "VIN_S": 19.0,
        "PHI_SB": 16.0,
        "PHI_S": -19.0,
        "VSS": -23.0,
    }

    terminals: dict[str, list[tuple[object, str]]] = defaultdict(list)
    for device in devices:
        reference = references[device.name]
        terminals[device.gate].append((reference, "gate"))
        terminals[device.source].append((reference, "source"))
        terminals[device.drain].append((reference, "drain"))

    global_left = float(substrate_tap.xmin - 2.0)
    global_right = float(substrate_tap.xmax + 2.0)
    for net, net_terminals in terminals.items():
        track_y = global_tracks.get(net, _local_track(net))
        if track_y is None:
            raise RuntimeError(f"no physical track assigned to {net}")
        branches = []
        for reference, terminal in net_terminals:
            if terminal == "gate":
                port = reference.ports["gate_S"]
                # P/N sample gates occupy the same slice x coordinate but
                # carry opposite phases.  Offset every gate branch by device
                # polarity so those M2 branches remain physically distinct.
                polarity_offset = 0.55 if float(port.center[1]) > 0 else -0.55
                branch_x = float(port.center[0]) + polarity_offset
            elif terminal == "source":
                port = reference.ports["source_W"]
                branch_x = float(port.center[0]) - 0.6
            else:
                port = reference.ports["drain_E"]
                branch_x = float(port.center[0]) + 0.6
            branches.append(
                _metal_stub_to_track(
                    top,
                    pdk=pdk,
                    port=port,
                    branch_x=branch_x,
                    track_y=track_y,
                )
            )

        if net in global_tracks:
            xmin, xmax = global_left, global_right
        else:
            xmin, xmax = min(branches), max(branches)
            if net.startswith("BP") or (
                net.startswith("SW") and not net.endswith("_B")
            ):
                spine_x = _external_spine_x(net, bits)
                xmin, xmax = min(xmin, spine_x), max(xmax, spine_x)
                end_y = track_y
                top.add_port(
                    name=net,
                    center=(spine_x, end_y),
                    width=m4_width,
                    orientation=0 if net.startswith("BP") else 180,
                    layer=met4,
                )
                top.add_label(text=net, position=(spine_x, end_y), layer=met4)
        _add_rect(
            top,
            met4,
            xmin,
            track_y - m4_width / 2,
            xmax,
            track_y + m4_width / 2,
        )

    # Connect the n-well and substrate taps to their global buses.
    for ring, side, net in (
        (nwell_tap, "N", "VDD"),
        (substrate_tap, "S", "VSS"),
    ):
        port = _ring_port(ring, side)
        branch_x = _metal_stub_to_track(
            top,
            pdk=pdk,
            port=port,
            branch_x=float(port.center[0]),
            track_y=global_tracks[net],
        )
        _add_rect(
            top,
            met4,
            min(branch_x, global_left),
            global_tracks[net] - m4_width / 2,
            max(branch_x, global_left),
            global_tracks[net] + m4_width / 2,
        )

    # Global buses enter from alternating sides to keep labels unambiguous.
    for index, (net, y) in enumerate(global_tracks.items()):
        right = bool(index % 2)
        x = global_right if right else global_left
        top.add_port(
            name=net,
            center=(x, y),
            width=m4_width,
            orientation=0 if right else 180,
            layer=met4,
        )
        # Keep the extraction label inside the macro.  Magic's ``port
        # makeall`` does not promote labels exactly on this cell's outermost
        # geometry edge, even though it still uses them to name the net.
        top.add_label(text=net, position=(core_centre_x, y), layer=met4)

    top.info["netlist"] = _canonical_netlist(bits)
    top.info["lvs_netlist"] = _canonical_netlist(bits)
    top.info["functional_devices"] = [device.__dict__ for device in devices]
    top.info["device_count"] = len(devices)
    top.info["device_kinds"] = dict(Counter(d.kind for d in devices))
    top.info["bit_order"] = bit_order(bits)
    top.info["resolution_bits"] = bits
    top.info["includes_phase_inverter"] = True
    top.info["includes_top_plate_reset"] = False
    return component_snap_to_grid(top)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    sar_cdac_switch_matrix(sky130_mapped_pdk).write_gds("sar_cdac_switch_matrix.gds")
