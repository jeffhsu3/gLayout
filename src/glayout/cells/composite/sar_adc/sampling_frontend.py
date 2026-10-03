"""Sampling-front-end generators for the 1.2 V SKY130 SAR ADC.

The passive CDAC remains a strictly passive, independently signed-off macro.
This module owns the active circuits that touch its input and top plate:

* :func:`sar_bootstrapped_switch` implements the floating-well bootstrap;
* :func:`sar_top_plate_reset` implements the reset TG and charge dummy;
* :func:`sar_sampling_frontend` combines both signed-off cells.

The four bootstrap PMOS devices share one continuous n-well tied to the
dynamic ``NWELL`` node.  It is deliberately not tied to VDD: that physical
detail is the body-bias fix that prevents the boosted nodes from forward
biasing a source/drain-to-well diode.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.primitives.fet import nmos, pmos
from glayout.primitives.guardring import tapring
from glayout.primitives.mimcap import mimcap
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist


BOOTSTRAP_PINS: tuple[str, ...] = ("VDD", "VSS", "PHI_SB", "VIN_S", "VIN")
TOP_RESET_PINS: tuple[str, ...] = (
    "VDD",
    "VSS",
    "VDAC",
    "VCM",
    "PHI_S",
    "PHI_SB",
)
SAMPLING_FRONTEND_PINS: tuple[str, ...] = (
    "VDD",
    "VSS",
    "VIN",
    "VIN_S",
    "VDAC",
    "VCM",
    "PHI_S",
    "PHI_SB",
)
BOOT_CAP_SIZE_UM = 15.6
TOP_DUMMY_WIDTH_UM = 1.65


@dataclass(frozen=True)
class FrontendDevice:
    """One MOSFET in a canonical sampling-front-end schematic."""

    name: str
    kind: str
    drain: str
    gate: str
    source: str
    bulk: str
    width_um: float
    length_um: float = 0.15


BOOTSTRAP_DEVICES: tuple[FrontendDevice, ...] = (
    FrontendDevice("Q0", "nfet", "VIN", "NET1", "VIN_S", "VSS", 1.0),
    FrontendDevice("Q5", "nfet", "NET1", "PHI_SB", "VSS", "VSS", 1.0),
    FrontendDevice("Q4", "nfet", "VIN", "NET1", "NET2", "VSS", 1.0),
    FrontendDevice("WA", "pfet", "NWELL", "NET1", "VDD", "NWELL", 1.0),
    FrontendDevice("WB", "pfet", "NWELL", "VDD", "NET1", "NWELL", 1.0),
    FrontendDevice("Q1", "pfet", "VDD", "NET1", "NET3", "NWELL", 2.0),
    FrontendDevice("Q3", "nfet", "NET2", "PHI_SB", "VSS", "VSS", 1.0),
    FrontendDevice("Q2", "pfet", "NET3", "PHI_SB", "NET1", "NWELL", 1.0),
)

TOP_RESET_DEVICES: tuple[FrontendDevice, ...] = (
    FrontendDevice("TOP_N", "nfet", "VDAC", "PHI_S", "VCM", "VSS", 4.0),
    FrontendDevice("TOP_P", "pfet", "VDAC", "PHI_SB", "VCM", "VDD", 4.0),
    FrontendDevice(
        "TOP_DUMMY_N",
        "nfet",
        "VDAC",
        "PHI_SB",
        "VDAC",
        "VSS",
        TOP_DUMMY_WIDTH_UM,
    ),
)


def _canonical_netlist(
    circuit_name: str,
    pins: tuple[str, ...],
    devices: tuple[FrontendDevice, ...],
    *,
    include_boot_cap: bool = False,
) -> Netlist:
    lines = [f".subckt {circuit_name} {' '.join(pins)}"]
    for device in devices:
        lines.append(
            f"X{device.name} {device.drain} {device.gate} "
            f"{device.source} {device.bulk} sky130_fd_pr__{device.kind}_01v8 "
            f"L={device.length_um:g} W={device.width_um:g} nf=1 mult=1"
        )
    if include_boot_cap:
        lines.append(
            "XC_BOOT NET3 NET2 sky130_fd_pr__cap_mim_m3_1 "
            f"w={BOOT_CAP_SIZE_UM:g} l={BOOT_CAP_SIZE_UM:g}"
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


def _terminal_to_track(
    component: Component,
    *,
    pdk: MappedPDK,
    port,
    branch_x: float,
    track_y: float,
) -> float:
    """Route a device M1 terminal through an M2 drop to an M3 track."""

    if pdk.layer_to_glayer(port.layer) != "met2":
        raise ValueError(f"expected an abstract met2 terminal, got {port.layer}")
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
    lower_via = component << via_stack(pdk, "met2", "met3")
    lower_via.move((branch_x, y0))
    _add_rect(
        component,
        met3,
        branch_x - met3_width / 2,
        min(y0, track_y) - met3_width / 2,
        branch_x + met3_width / 2,
        max(y0, track_y) + met3_width / 2,
    )
    upper_via = component << via_stack(pdk, "met3", "met4")
    upper_via.move((branch_x, track_y))
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


def _fet_templates(
    pdk: MappedPDK, devices: tuple[FrontendDevice, ...]
) -> dict[tuple[str, float], Component]:
    templates: dict[tuple[str, float], Component] = {}
    for kind, width in sorted({(device.kind, device.width_um) for device in devices}):
        options = {
            "width": width,
            "length": 0.15,
            "fingers": 1,
            "multipliers": 1,
            "with_tie": False,
            "with_dummy": False,
            "with_substrate_tap": False,
        }
        if kind == "nfet":
            options["with_dnwell"] = False
            template = nmos(pdk, **options)
            template.remove_layers([pdk.get_glayer("pwell")])
        else:
            options["dnwell"] = False
            template = pmos(pdk, **options)
        templates[(kind, width)] = template
    return templates


def _place_devices(
    top: Component,
    *,
    pdk: MappedPDK,
    devices: tuple[FrontendDevice, ...],
    targets: dict[str, tuple[float, float]],
) -> dict[str, object]:
    templates = _fet_templates(pdk, devices)
    references: dict[str, object] = {}
    for device in devices:
        reference = top << templates[(device.kind, device.width_um)]
        reference.move(tuple(_snap(pdk, value) for value in targets[device.name]))
        references[device.name] = reference
    return references


def _add_wells_and_guard(
    top: Component,
    *,
    pdk: MappedPDK,
    references: dict[str, object],
    devices: tuple[FrontendDevice, ...],
    extra_xmax: float,
) -> tuple[object, object, float, float]:
    pmos_references = [
        references[device.name] for device in devices if device.kind == "pfet"
    ]
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
    nwell_tap.move((_snap(pdk, (pmos_xmin + pmos_xmax) / 2), 24.0))
    nwell_margin = pdk.get_grule("nwell", "active_tap")["min_enclosure"]
    _add_rect(
        top,
        pdk.get_glayer("nwell"),
        nwell_tap.xmin - nwell_margin,
        pmos_ymin - nwell_margin,
        nwell_tap.xmax + nwell_margin,
        nwell_tap.ymax + nwell_margin,
    )

    core_xmin = min(reference.xmin for reference in references.values())
    core_xmax = max(
        extra_xmax,
        max(reference.xmax for reference in references.values()),
    )
    core_centre_x = (core_xmin + core_xmax) / 2
    substrate_tap = top << tapring(
        pdk,
        enclosed_rectangle=(core_xmax - core_xmin + 14.0, 48.0),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    substrate_tap.move((_snap(pdk, core_centre_x), 0.0))
    return (
        nwell_tap,
        substrate_tap,
        float(substrate_tap.xmin - 2.0),
        float(substrate_tap.xmax + 2.0),
    )


def _route_devices(
    top: Component,
    *,
    pdk: MappedPDK,
    devices: tuple[FrontendDevice, ...],
    references: dict[str, object],
    tracks: dict[str, float],
    extensions: dict[str, tuple[float, float]] | None = None,
) -> dict[str, tuple[float, float]]:
    terminals: dict[str, list[tuple[FrontendDevice, object, str]]] = defaultdict(list)
    for device in devices:
        reference = references[device.name]
        terminals[device.gate].append((device, reference, "gate"))
        terminals[device.source].append((device, reference, "source"))
        terminals[device.drain].append((device, reference, "drain"))

    met4 = pdk.get_glayer("met4")
    width = pdk.get_grule("met4")["min_width"]
    spans: dict[str, tuple[float, float]] = {}
    for net, net_terminals in terminals.items():
        if net not in tracks:
            raise RuntimeError(f"no track assigned to {net}")
        branches = []
        for device, reference, terminal in net_terminals:
            if terminal == "gate":
                port = reference.ports["gate_S"]
                # TOP_P's long PHI_SB drop sits between its source and drain
                # branches.  The default east offset leaves only 65 nm to the
                # drain branch on M2; the centred drop clears both terminals
                # while remaining separated from TOP_N's west-offset PHI_S.
                if device.name == "TOP_P":
                    offset = 0.0
                else:
                    offset = 0.55 if float(port.center[1]) > 0 else -0.55
                branch_x = float(port.center[0]) + offset
            elif terminal == "source":
                port = reference.ports["source_W"]
                # Q0's source branch travels below the device to VIN_S.  Pull
                # it farther west so it clears Q0's long NET1 gate branch on
                # physical M2; the generic offset leaves only 55 nm.
                source_offset = 1.1 if device.name == "Q0" and net == "VIN_S" else 0.6
                branch_x = float(port.center[0]) - source_offset
            else:
                port = reference.ports["drain_E"]
                branch_x = float(port.center[0]) + 0.6
            branches.append(
                _terminal_to_track(
                    top,
                    pdk=pdk,
                    port=port,
                    branch_x=branch_x,
                    track_y=tracks[net],
                )
            )
        xmin, xmax = min(branches), max(branches)
        if extensions and net in extensions:
            xmin = min(xmin, extensions[net][0])
            xmax = max(xmax, extensions[net][1])
        if xmax <= xmin:
            xmin -= width / 2
            xmax += width / 2
        _add_rect(
            top,
            met4,
            xmin,
            tracks[net] - width / 2,
            xmax,
            tracks[net] + width / 2,
        )
        spans[net] = (xmin, xmax)
    return spans


def _connect_ring(
    top: Component,
    *,
    pdk: MappedPDK,
    ring,
    side: str,
    net: str,
    track_y: float,
) -> float:
    port = _ring_port(ring, side)
    return _terminal_to_track(
        top,
        pdk=pdk,
        port=port,
        branch_x=float(port.center[0]),
        track_y=track_y,
    )


def _extend_track(
    top: Component,
    *,
    pdk: MappedPDK,
    y: float,
    current_span: tuple[float, float],
    target_x: float,
) -> tuple[float, float]:
    met4 = pdk.get_glayer("met4")
    width = pdk.get_grule("met4")["min_width"]
    xmin = min(current_span[0], target_x)
    xmax = max(current_span[1], target_x)
    _add_rect(top, met4, xmin, y - width / 2, xmax, y + width / 2)
    return xmin, xmax


def _add_external_ports(
    top: Component,
    *,
    pdk: MappedPDK,
    tracks: dict[str, float],
    sides: dict[str, str],
    left: float,
    right: float,
) -> None:
    met4 = pdk.get_glayer("met4")
    width = pdk.get_grule("met4")["min_width"]
    for net, side in sides.items():
        x = left if side == "left" else right
        orientation = 180 if side == "left" else 0
        top.add_port(
            name=net,
            center=(x, tracks[net]),
            width=width,
            orientation=orientation,
            layer=met4,
        )
        # Keep labels one micron inside the routed boundary.  A common centre
        # coordinate can fall beyond a short one-sided net (VIN in particular)
        # and then Magic creates a disconnected top-level pin.
        label_x = x + 1.0 if side == "left" else x - 1.0
        top.add_label(text=net, position=(label_x, tracks[net]), layer=met4)


def sar_bootstrapped_switch(
    pdk: MappedPDK,
    *,
    boot_cap_size_um: float = BOOT_CAP_SIZE_UM,
    length_um: float = 0.15,
) -> Component:
    """Generate the floating-well bootstrapped sampling switch."""

    if length_um != 0.15:
        raise ValueError("the bootstrap is qualified only at L=0.15 um")
    if boot_cap_size_um != BOOT_CAP_SIZE_UM:
        raise ValueError("the bootstrap is qualified only with a 15.6 um MIM cap")
    pdk.activate()
    top = Component(name="sar_bootstrapped_switch")
    targets = {
        "WA": (-6.0, 9.0),
        "WB": (-2.0, 9.0),
        "Q1": (2.0, 9.0),
        "Q2": (6.0, 9.0),
        "Q5": (-6.0, -9.0),
        "Q3": (-2.0, -9.0),
        "Q4": (2.0, -9.0),
        "Q0": (6.0, -9.0),
    }
    references = _place_devices(
        top,
        pdk=pdk,
        devices=BOOTSTRAP_DEVICES,
        targets=targets,
    )
    cap = top << mimcap(pdk, (boot_cap_size_um, boot_cap_size_um), with_extension=False)
    cap.move((20.0, 0.0))
    nwell_tap, substrate_tap, route_left, route_right = _add_wells_and_guard(
        top,
        pdk=pdk,
        references=references,
        devices=BOOTSTRAP_DEVICES,
        extra_xmax=float(cap.xmax),
    )
    tracks = {
        "VDD": 18.0,
        "NWELL": 14.0,
        "NET3": 11.1,
        "NET1": 4.0,
        "NET2": 0.0,
        "VIN": -4.0,
        # Keep the sample-output track clear of the wide M3 bootstrap plate.
        # A track at -8 um clips the plate's -7.94 um boundary after width is
        # included, which both violates the wide-metal spacing rule and shorts
        # VIN_S to NET2 in extraction.
        "VIN_S": -10.0,
        "PHI_SB": -14.0,
        "VSS": -18.0,
    }

    cap_top = cap.ports["top_met_E"]
    # The top-plate transition must sit wholly outside the M3 bottom plate;
    # otherwise an ordinary M3/M4 via would short the bootstrap capacitor.
    cap_via_x = _snap(pdk, float(cap.xmax) + 2.0)
    spans = _route_devices(
        top,
        pdk=pdk,
        devices=BOOTSTRAP_DEVICES,
        references=references,
        tracks=tracks,
        extensions={
            "NET3": (cap_via_x, cap_via_x),
        },
    )

    # Escape the M3 bottom plate through M2.  The inner via is completely
    # enclosed by the plate; the outer via observes wide-M3 spacing.  This
    # avoids a minimum-width M3 neck at the edge of the >3 um plate.
    bottom_inner_x = _snap(pdk, float(cap.xmin) + 1.0)
    bottom_outer_x = _snap(pdk, float(cap.xmin) - 1.6)
    bottom_y = float(cap.center[1])
    spans["NET2"] = _extend_track(
        top,
        pdk=pdk,
        y=tracks["NET2"],
        current_span=spans["NET2"],
        target_x=bottom_outer_x,
    )
    for x in (bottom_outer_x, bottom_inner_x):
        bottom_transition = top << via_stack(pdk, "met3", "met4")
        bottom_transition.move((x, bottom_y))
    bottom_bridge_width = 0.58
    _add_rect(
        top,
        pdk.get_glayer("met3"),
        bottom_outer_x,
        bottom_y - bottom_bridge_width / 2,
        bottom_inner_x,
        bottom_y + bottom_bridge_width / 2,
    )

    met5 = pdk.get_glayer("met5")
    m5_escape_width = 3.2
    _add_rect(
        top,
        met5,
        min(float(cap_top.center[0]), cap_via_x),
        float(cap_top.center[1]) - m5_escape_width / 2,
        max(float(cap_top.center[0]), cap_via_x),
        float(cap_top.center[1]) + m5_escape_width / 2,
    )
    _add_rect(
        top,
        met5,
        cap_via_x - m5_escape_width / 2,
        min(float(cap_top.center[1]), tracks["NET3"]),
        cap_via_x + m5_escape_width / 2,
        max(float(cap_top.center[1]), tracks["NET3"]) + m5_escape_width / 2,
    )
    cap_transition = top << via_stack(pdk, "met4", "met5")
    cap_transition.move((cap_via_x, tracks["NET3"]))

    well_x = _connect_ring(
        top,
        pdk=pdk,
        ring=nwell_tap,
        side="N",
        net="NWELL",
        track_y=tracks["NWELL"],
    )
    spans["NWELL"] = _extend_track(
        top,
        pdk=pdk,
        y=tracks["NWELL"],
        current_span=spans["NWELL"],
        target_x=well_x,
    )
    substrate_x = _connect_ring(
        top,
        pdk=pdk,
        ring=substrate_tap,
        side="S",
        net="VSS",
        track_y=tracks["VSS"],
    )
    spans["VSS"] = _extend_track(
        top,
        pdk=pdk,
        y=tracks["VSS"],
        current_span=spans["VSS"],
        target_x=substrate_x,
    )

    for net in BOOTSTRAP_PINS:
        spans[net] = _extend_track(
            top,
            pdk=pdk,
            y=tracks[net],
            current_span=spans[net],
            target_x=route_left if net == "VIN" else route_right,
        )
    _add_external_ports(
        top,
        pdk=pdk,
        tracks=tracks,
        sides={
            "VDD": "right",
            "VSS": "right",
            "PHI_SB": "right",
            "VIN_S": "right",
            "VIN": "left",
        },
        left=route_left,
        right=route_right,
    )

    netlist = _canonical_netlist(
        "sar_bootstrapped_switch",
        BOOTSTRAP_PINS,
        BOOTSTRAP_DEVICES,
        include_boot_cap=True,
    )
    top.info["netlist"] = netlist
    top.info["lvs_netlist"] = netlist
    top.info["functional_devices"] = [device.__dict__ for device in BOOTSTRAP_DEVICES]
    top.info["device_count"] = len(BOOTSTRAP_DEVICES)
    top.info["device_kinds"] = dict(
        Counter(device.kind for device in BOOTSTRAP_DEVICES)
    )
    top.info["mimcap_count"] = 1
    top.info["boot_cap_size_um"] = boot_cap_size_um
    top.info["floating_well_devices"] = ("WA", "WB", "Q1", "Q2")
    return component_snap_to_grid(top)


def sar_top_plate_reset(
    pdk: MappedPDK,
    *,
    dummy_width_um: float = TOP_DUMMY_WIDTH_UM,
    length_um: float = 0.15,
) -> Component:
    """Generate the VCM reset TG and complementary NMOS charge dummy."""

    if length_um != 0.15:
        raise ValueError("the top-plate reset is qualified only at L=0.15 um")
    if dummy_width_um != TOP_DUMMY_WIDTH_UM:
        raise ValueError("the top-plate dummy is qualified only at W=1.65 um")
    pdk.activate()
    top = Component(name="sar_top_plate_reset")
    targets = {
        "TOP_P": (0.0, 9.0),
        "TOP_N": (0.0, -9.0),
        "TOP_DUMMY_N": (5.0, -9.0),
    }
    references = _place_devices(
        top,
        pdk=pdk,
        devices=TOP_RESET_DEVICES,
        targets=targets,
    )
    nwell_tap, substrate_tap, route_left, route_right = _add_wells_and_guard(
        top,
        pdk=pdk,
        references=references,
        devices=TOP_RESET_DEVICES,
        extra_xmax=max(reference.xmax for reference in references.values()),
    )
    tracks = {
        "VDD": 18.0,
        "VDAC": 6.0,
        "VCM": 2.0,
        "PHI_S": -8.0,
        "PHI_SB": -14.0,
        "VSS": -18.0,
    }
    spans = _route_devices(
        top,
        pdk=pdk,
        devices=TOP_RESET_DEVICES,
        references=references,
        tracks=tracks,
    )
    well_x = _connect_ring(
        top,
        pdk=pdk,
        ring=nwell_tap,
        side="N",
        net="VDD",
        track_y=tracks["VDD"],
    )
    reset_track_width = pdk.get_grule("met4")["min_width"]
    spans["VDD"] = (
        well_x - reset_track_width / 2,
        well_x + reset_track_width / 2,
    )
    spans["VDD"] = _extend_track(
        top,
        pdk=pdk,
        y=tracks["VDD"],
        current_span=spans["VDD"],
        target_x=well_x,
    )
    substrate_x = _connect_ring(
        top,
        pdk=pdk,
        ring=substrate_tap,
        side="S",
        net="VSS",
        track_y=tracks["VSS"],
    )
    spans["VSS"] = (
        substrate_x - reset_track_width / 2,
        substrate_x + reset_track_width / 2,
    )
    spans["VSS"] = _extend_track(
        top,
        pdk=pdk,
        y=tracks["VSS"],
        current_span=spans["VSS"],
        target_x=substrate_x,
    )
    sides = {
        "VDD": "left",
        "VSS": "left",
        "PHI_SB": "left",
        "VDAC": "right",
        "VCM": "right",
        "PHI_S": "right",
    }
    for net in TOP_RESET_PINS:
        spans[net] = _extend_track(
            top,
            pdk=pdk,
            y=tracks[net],
            current_span=spans[net],
            target_x=route_left if sides[net] == "left" else route_right,
        )
    _add_external_ports(
        top,
        pdk=pdk,
        tracks=tracks,
        sides=sides,
        left=route_left,
        right=route_right,
    )

    netlist = _canonical_netlist(
        "sar_top_plate_reset",
        TOP_RESET_PINS,
        TOP_RESET_DEVICES,
    )
    top.info["netlist"] = netlist
    top.info["lvs_netlist"] = netlist
    top.info["functional_devices"] = [device.__dict__ for device in TOP_RESET_DEVICES]
    top.info["device_count"] = len(TOP_RESET_DEVICES)
    top.info["device_kinds"] = dict(
        Counter(device.kind for device in TOP_RESET_DEVICES)
    )
    top.info["dummy_width_um"] = dummy_width_um
    top.info["dummy_source_drain_shorted"] = True
    return component_snap_to_grid(top)


def sar_sampling_frontend(pdk: MappedPDK) -> Component:
    """Combine the bootstrap and reset cells without absorbing the CDAC."""

    pdk.activate()
    top = Component(name="sar_sampling_frontend")
    bootstrap_component = sar_bootstrapped_switch(pdk)
    reset_component = sar_top_plate_reset(pdk)
    bootstrap = top << bootstrap_component
    bootstrap.movex(_snap(pdk, -float(bootstrap.xmin)))
    reset = top << reset_component
    reset.movex(_snap(pdk, float(bootstrap.xmax - reset.xmin + 12.0)))

    met4 = pdk.get_glayer("met4")
    width = pdk.get_grule("met4")["min_width"]
    for net in ("VDD", "VSS", "PHI_SB"):
        left_port = bootstrap.ports[net]
        right_port = reset.ports[net]
        if abs(float(left_port.center[1] - right_port.center[1])) > 1e-6:
            raise RuntimeError(f"sampling-front-end {net} tracks are not aligned")
        y = float(left_port.center[1])
        _add_rect(
            top,
            met4,
            min(float(left_port.center[0]), float(right_port.center[0])),
            y - width / 2,
            max(float(left_port.center[0]), float(right_port.center[0])),
            y + width / 2,
        )
        centre_x = (float(left_port.center[0]) + float(right_port.center[0])) / 2
        top.add_port(
            name=net,
            center=(centre_x, y),
            width=width,
            orientation=90,
            layer=met4,
        )
        top.add_label(text=net, position=(centre_x, y), layer=met4)

    exposed = {
        "VIN": bootstrap.ports["VIN"],
        "VIN_S": bootstrap.ports["VIN_S"],
        "VDAC": reset.ports["VDAC"],
        "VCM": reset.ports["VCM"],
        "PHI_S": reset.ports["PHI_S"],
    }
    for name, port in exposed.items():
        top.add_port(name=name, port=port)
        # Reference ports lie exactly on the child boundary.  Magic can assign
        # a label on that edge to empty space instead of the child conductor,
        # so promote the pin with a parent-level label one micron inward.
        if float(port.orientation) == 180.0:
            label_x = float(port.center[0]) + 1.0
        elif float(port.orientation) == 0.0:
            label_x = float(port.center[0]) - 1.0
        else:
            raise RuntimeError(f"expected a horizontal {name} port")
        top.add_label(
            text=name,
            position=(label_x, float(port.center[1])),
            layer=port.layer,
        )

    combined_devices = BOOTSTRAP_DEVICES + TOP_RESET_DEVICES
    netlist = _canonical_netlist(
        "sar_sampling_frontend",
        SAMPLING_FRONTEND_PINS,
        combined_devices,
        include_boot_cap=True,
    )
    top.info["netlist"] = netlist
    top.info["lvs_netlist"] = netlist
    top.info["functional_devices"] = [device.__dict__ for device in combined_devices]
    top.info["device_count"] = len(combined_devices)
    top.info["device_kinds"] = dict(Counter(device.kind for device in combined_devices))
    top.info["mimcap_count"] = 1
    top.info["boot_cap_size_um"] = BOOT_CAP_SIZE_UM
    top.info["dummy_width_um"] = TOP_DUMMY_WIDTH_UM
    top.info["subblocks"] = (
        "sar_bootstrapped_switch",
        "sar_top_plate_reset",
    )
    top.info["passive_cdac_included"] = False
    return component_snap_to_grid(top)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    sar_sampling_frontend(sky130_mapped_pdk).write_gds("sar_sampling_frontend.gds")
