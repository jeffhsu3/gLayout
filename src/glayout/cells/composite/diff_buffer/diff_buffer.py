"""diff_buffer — pseudo-differential unity-gain buffer for the analog Conv1D delay line.

Two LOW-POWER OpenFASOC two-stage opamps (one per rail), each wired as a unity-gain
follower, buffer a differential tap while preserving BOTH the differential signal and the
common mode -- the fix for the delay-line CM droop (cascaded source-followers collapsed it).
Validated in ngspice (analog_ref/opamp_lp/casc.spice): diff preserved exactly, CM held at
1.649V across a 4-stage cascade, ~66uW/opamp (528uW for a 4-tap buffer). DRC-clean.

Each opamp: OPAMP_TWO_STAGE  VDD GND DIFFPAIR_BIAS VP VN CS_BIAS VOUT. Follower wiring (VP is
the INVERTING input, verified empirically): VP shorted to VOUT (feedback), signal into VN.
Output node = port `commonsource_output_E` (no pin_output when add_output_stage=False).
Bias pins are CURRENT inputs (diode-connected mirror refs); the two opamps SHARE VDP_BIAS /
VCS_BIAS (tied) for rail matching -- the top-level bias gen sources the total current.

Ports: VINP VINN VOUTP VOUTN VDP_BIAS VCS_BIAS VDD VSS.
"""

import re
from dataclasses import dataclass
from glayout.backend import Component, cell, rectangle
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.routing.straight_route import straight_route
from glayout.primitives.via_gen import via_stack
from glayout.spice.netlist import Netlist
from glayout.cells.composite.opamp.opamp import opamp

# low-power sizing (matches analog_ref/opamp_lp/gen_opamp_lp.py): ~66uW/opamp.
# This used to also pass inter_finger_topmet="met1" and tie_layers=("met1","met1"):
# keeping the per-finger S/D via arrays on met1 so their met2 patches don't sit
# ~0.25um from the met2 S/D rails (gf180 M2.2a needs 0.28). Both are the library
# default now, so the explicit opt-in is gone -- the geometry is unchanged.
# Verified: LP opamp gf180 magic DRC 106 -> 16 (M2.2a 186 -> 17); sky130 unchanged.
LP = dict(
    half_diffpair_params=(2, 1, 2),
    diffpair_bias=(2, 4, 2),
    half_common_source_params=(3, 1, 4, 2),
    half_common_source_bias=(2, 4, 2, 2),
    half_pload=(2, 1, 4),
    mim_cap_size=(6, 6),
    mim_cap_rows=1,
    rmult=1,
    with_antenna_diode_on_diffinputs=2,
    add_output_stage=False,
    # widen the diff pair's plus/minus gate-bar gap (default abuts at min_sep
    # ~0.3um for ~20um): with the 4um input pin-rail lift in opamp_twostage
    # this cuts the hold<->VOUT coupling that limit-cycled the S/H chain at ff
    # (chipforge PEX_RC_SIMS.md sec 6)
    diffpair_plus_minus_sep=1.5,
)


def _cs_output(ref_) -> str:
    """Name of the opamp's common-source output port on this reference.

    opamp_twostage adds it as ``commonsource_output_E``, but the opamp runs
    ``rename_ports_by_orientation`` afterwards, which re-derives the compass
    suffix from the port's actual facing -- and that facing depends on how the
    n-to-p output c_route lands, which is not the same in every configuration.
    Match on the stem so the buffer keeps working whichever way it points.
    """
    matches = [n for n in ref_.ports if n.startswith("commonsource_output_")]
    if not matches:
        raise KeyError(
            "opamp reference exposes no commonsource_output_* port; "
            f"got {sorted(n for n in ref_.ports if 'output' in n)}"
        )
    return matches[0]


@dataclass(frozen=True)
class DiffBufferPhysicalConfig:
    """Layout-only knobs for compacting :func:`diff_buffer`.

    Defaults reproduce the established hand-routed layout.  Keeping these knobs
    separate from ``LP`` prevents a physical search from silently changing the
    circuit sizing or generated netlist.
    """

    opamp_gap: float = 10.0
    supply_overhang: float = 1.0
    vcs_route_clearance: float = 2.0
    vinn_route_clearance: float = 4.0
    signal_width: float = 1.0
    supply_width: float = 2.0

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")


def units_fix(netlist: str) -> str:
    """Append 'u' to bare numeric l=/w= (generator omits the unit -> ngspice reads meters)."""
    return "\n".join(
        re.sub(r"\b([lw])=(\d+\.?\d*)(?![u\d\.])", r"\1=\2u", ln)
        for ln in netlist.splitlines()
    )


def _diff_buffer_netlist(opa: Component) -> Netlist:
    nodes = ["VINP", "VINN", "VOUTP", "VOUTN", "VDP_BIAS", "VCS_BIAS", "VDD", "VSS"]
    net = Netlist(circuit_name="diff_buffer", nodes=nodes)
    # p-rail follower: VP=VOUTP (feedback), VN=VINP, VOUT=VOUTP
    net.connect_netlist(
        opa.info["netlist"],
        [
            ("VDD", "VDD"),
            ("GND", "VSS"),
            ("DIFFPAIR_BIAS", "VDP_BIAS"),
            ("VP", "VOUTP"),
            ("VN", "VINP"),
            ("CS_BIAS", "VCS_BIAS"),
            ("VOUT", "VOUTP"),
        ],
    )
    # n-rail follower: VP=VOUTN (feedback), VN=VINN, VOUT=VOUTN
    net.connect_netlist(
        opa.info["netlist"],
        [
            ("VDD", "VDD"),
            ("GND", "VSS"),
            ("DIFFPAIR_BIAS", "VDP_BIAS"),
            ("VP", "VOUTN"),
            ("VN", "VINN"),
            ("CS_BIAS", "VCS_BIAS"),
            ("VOUT", "VOUTN"),
        ],
    )
    return net


@cell
def diff_buffer(
    pdk: MappedPDK,
    gap: float = 10.0,
    physical_config: DiffBufferPhysicalConfig | None = None,
) -> Component:
    pdk.activate()
    cfg = physical_config or DiffBufferPhysicalConfig(opamp_gap=gap)
    opa = opamp(pdk, **LP)  # one body, placed twice (identical -> good rail matching)
    top = Component()
    pref = top << opa  # p-rail (left)
    nref = top << opa  # n-rail (right)
    nref.movex(pref.xmax + cfg.opamp_gap - nref.xmin)

    # EXPLICIT routing only -- smart_route repeatedly plowed through the opamp
    # interior here (merged nets, LVS "netlists do not match"). All paths below are
    # controlled: same-layer straight_routes between copied ports + explicit
    # via_stacks (the auto-layer inference of L_route/straight_route is avoided
    # for the same reason).
    routed, failed = [], []

    # (1)(2) unity-feedback FB: netlist node VP = the feedback side in the ngspice-
    # validated follower = LAYOUT port pin_minus (netgen pin binding on the
    # standalone opamp; feeding back to pin_plus would be POSITIVE feedback).
    # Path: met3 east at the pin_minus y (this band holds only the minus-input's
    # own routing and the VOUT-net n-to-p cons -- same nets), then an explicit
    # met3->met5 via landing on the VOUT riser.
    # ... on MET4: the met3 band at the pin_minus y also carries the PLUS input's
    # met3 riser (x~8-11) -- a met3 run shorted VOUT to VINP. Both inputs' met4
    # antenna hops are same-net or at a different y, so met4 is clear.
    for ref_ in (pref, nref):
        fb_lift = top << via_stack(pdk, "met3", "met4")
        fb_lift.movex(ref_.ports["pin_minus_E"].center[0] - 0.6 - fb_lift.center[0])
        fb_lift.movey(ref_.ports["pin_minus_E"].center[1] - fb_lift.center[1])
        fbv = top << via_stack(pdk, "met4", "met5")
        fbv.movex(ref_.ports[_cs_output(ref_)].center[0] - fbv.center[0])
        fbv.movey(ref_.ports["pin_minus_E"].center[1] - fbv.center[1])
        fb_a = fb_lift.ports["top_met_E"].copy()
        fb_b = fb_a.copy()
        fb_b.center = (fbv.center[0], fb_a.center[1])
        top << straight_route(
            pdk,
            fb_a,
            fb_b,
            glayer1="met4",
            glayer2="met4",
            width=cfg.signal_width,
        )
    routed += ["FB_P", "FB_N"]

    # (3) VDD OVER THE TOP on MET5: met4 is blocked twice up here -- a straight run
    # at the pin y crossed the mimcap-connection legs (comp node), and the summit
    # carries the fullbottom met4 VOUT drain-con bar spanning x=+-34.8 directly
    # above the vdd pin (a met4 vertical merged VDD with VOUT). met5 is empty at
    # the summit: via met4->met5 inside each vdd pin, met5 up/over/down.
    # raw rectangles for the met5 runs -- straight_route misdrew here (wrong layer
    # and direction), the sixth auto-router failure in this cell.
    def _bar(x0, y0, x1, y1, glayer):
        _r = top << rectangle(
            size=(round(x1 - x0, 3), round(y1 - y0, 3)),
            layer=pdk.get_glayer(glayer),
            centered=True,
        )
        _r.movex((x0 + x1) / 2 - _r.center[0]).movey((y0 + y1) / 2 - _r.center[1])

    _y_high = max(pref.ymax, nref.ymax) + cfg.supply_overhang
    _vups = []
    for ref_ in (pref, nref):
        _pin_c = ref_.ports["pin_vdd_N"].copy()
        _pin_c.center = (
            _pin_c.center[0],
            _pin_c.center[1] - 1.5,
        )  # inside the 5x3 pin rect
        _vv = top << via_stack(pdk, "met4", "met5")
        _vv.movex(_pin_c.center[0] - _vv.center[0]).movey(
            _pin_c.center[1] - _vv.center[1]
        )
        _vups.append(_vv)
        _bar(_vv.center[0] - 0.5, _vv.center[1], _vv.center[0] + 0.5, _y_high, "met5")
    _bar(
        _vups[0].center[0] - 0.5,
        _y_high - 1.0,
        _vups[1].center[0] + 0.5,
        _y_high,
        "met5",
    )
    # (4) VSS on MET3: a met4 run crossed the mimcap-connection c_route's fullbottom
    # met4 legs at x~56-71 (shorting VSS into the compensation node and breaking the
    # cap extraction). The met3 band here only crosses the opamp's own gnd legs
    # (same net). Explicit met3->met4 vias inside each gnd pin rect.
    _gva = top << via_stack(pdk, "met3", "met4")
    _gva.movex(pref.ports["pin_gnd_E"].center[0] - 0.6 - _gva.center[0])
    _gva.movey(pref.ports["pin_gnd_E"].center[1] - _gva.center[1])
    _gvb = top << via_stack(pdk, "met3", "met4")
    _gvb.movex(nref.ports["pin_gnd_W"].center[0] + 0.6 - _gvb.center[0])
    _gvb.movey(nref.ports["pin_gnd_W"].center[1] - _gvb.center[1])
    _ga = _gva.ports["bottom_met_E"].copy()
    _gb = _ga.copy()
    _gb.center = (_gvb.center[0], _ga.center[1])
    top << straight_route(
        pdk, _ga, _gb, glayer1="met3", glayer2="met3", width=cfg.supply_width
    )
    # (5) VDP_BIAS: met3 rects at the same y -- straight met3 (crosses the met4
    # vbias2 pin and cap plates on a different layer).
    top << straight_route(
        pdk,
        pref.ports["pin_diffpairibias_E"],
        nref.ports["pin_diffpairibias_W"],
        glayer1="met3",
        glayer2="met3",
        width=cfg.supply_width,
    )
    routed += ["VDD", "VSS", "VDP_BIAS"]

    # (6) VCS_BIAS: the pins are met4 at the caps' y -- a met4 run would short the
    # mimcap met4 bottom plates (and violate MIMTM.1). Drop each pin to met3 and
    # run met3 BELOW both cells (clear of the met3 vbias1/dpbias rects), then up.
    # vertical drop on MET4 (a met3 vertical would cross the VDP_BIAS met3 rail at
    # the same y); the long horizontal runs on met3 BELOW both cells.
    _y_low = min(pref.ymin, nref.ymin) - cfg.vcs_route_clearance
    _lowvias = []
    for ref_ in (pref, nref):
        _pin = ref_.ports["pin_commonsourceibias_S"]
        _d = _pin.copy()
        _d.center = (_pin.center[0], _y_low)
        top << straight_route(
            pdk,
            _pin,
            _d,
            glayer1="met4",
            glayer2="met4",
            width=cfg.signal_width,
        )
        _lv = top << via_stack(pdk, "met3", "met4")
        _lv.movex(_pin.center[0] - _lv.center[0]).movey(_y_low + 0.4 - _lv.center[1])
        _lowvias.append(_lv)
    _ha = _lowvias[0].ports["bottom_met_E"].copy()
    _hb = _ha.copy()
    _hb.center = (_lowvias[1].center[0], _ha.center[1])
    top << straight_route(
        pdk, _ha, _hb, glayer1="met3", glayer2="met3", width=cfg.signal_width
    )
    routed += ["VCS_BIAS"]

    # (7) VINN egress. nref's plus input lands at nref's WEST edge -- i.e. the MIDDLE of
    # the buffer (in the inter-opamp gap) -- so a left-side consumer (the delay stage
    # drives BOTH inputs from the west) can't reach VINN without the router plowing
    # across the top through VINP, shorting the two held nodes. Bring VINN out to the
    # buffer's SW corner on met2 (free in the gap and below the cells): met3 west into
    # the gap, drop to met2, DOWN the clear gap column, then WEST below both opamps to a
    # low pin at the left edge -- well clear of VINP (west edge, y~22) so the two stage
    # inputs present at separated heights and route without collision.
    _vinn_src = nref.ports["pin_plus_W"]  # met3, faces W, at nref's west edge
    _gap_x = (pref.xmax + nref.xmin) / 2.0  # centre of the inter-opamp gap
    _vinn_v = top << via_stack(pdk, "met2", "met3")
    _vinn_v.movex(_gap_x - _vinn_v.center[0]).movey(
        _vinn_src.center[1] - _vinn_v.center[1]
    )
    top << straight_route(
        pdk,
        _vinn_src,
        _vinn_v.ports["top_met_E"],
        glayer1="met3",
        glayer2="met3",
        width=cfg.signal_width,
    )
    _y_bot = min(pref.ymin, nref.ymin) - cfg.vinn_route_clearance
    _down_a = _vinn_v.ports["bottom_met_S"]
    _down_b = _down_a.copy()
    _down_b.center = (_gap_x, _y_bot)
    _down_b.orientation = 90
    top << straight_route(
        pdk,
        _down_a,
        _down_b,
        glayer1="met2",
        glayer2="met2",
        width=cfg.signal_width,
    )
    _x_edge = pref.xmin
    _west_a = _down_b.copy()
    _west_a.orientation = 180
    _west_b = _down_b.copy()
    _west_b.center = (_x_edge, _y_bot)
    _west_b.orientation = 0
    top << straight_route(
        pdk,
        _west_a,
        _west_b,
        glayer1="met2",
        glayer2="met2",
        width=cfg.signal_width,
    )
    routed += ["VINN_egress"]
    _vinn_pin = _west_b.copy()
    _vinn_pin.orientation = 180  # W-facing input pin at the edge

    # expose clean-named buffer pins (composable + labelable for LVS)
    top.add_port(name="VINP", port=pref.ports["pin_plus_W"])
    top.add_port(name="VINN", port=_vinn_pin)
    top.add_port(name="VOUTP", port=pref.ports[_cs_output(pref)])
    top.add_port(name="VOUTN", port=nref.ports[_cs_output(nref)])
    top.add_port(name="VDP_BIAS", port=pref.ports["pin_diffpairibias_W"])
    top.add_port(name="VCS_BIAS", port=pref.ports["pin_commonsourceibias_W"])
    top.add_port(name="VDD", port=pref.ports["pin_vdd_W"])
    top.add_port(name="VSS", port=pref.ports["pin_gnd_W"])

    comp = component_snap_to_grid(top)
    comp.info["netlist"] = _diff_buffer_netlist(opa)
    comp.info["routed"] = routed
    comp.info["route_failed"] = failed
    comp.info["physical_config"] = cfg
    return comp


if __name__ == "__main__":
    from glayout.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk

    c = diff_buffer(gf180_mapped_pdk)
    c.write_gds("diff_buffer.gds")
    open("diff_buffer.spice", "w").write(
        units_fix(c.info["netlist"].generate_netlist())
    )
    print("GDS:", c.name, "bbox=", c.bbox)
    print("routed:", c.info.get("routed"), "FAILED:", c.info.get("route_failed"))
