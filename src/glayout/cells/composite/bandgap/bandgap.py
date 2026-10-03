"""Brokaw-style CMOS bandgap voltage reference core (gf180).

Topology
--------
A self-biased beta-multiplier bandgap core after the principle in
D. M. Colombo, "Bandgap Voltage References in submicrometer CMOS
technology" (2009, ``refs/bandgap/000697738.pdf``): two NPNs with
different emitter areas (Q1 1x, Q2 8x) force a PTAT voltage
``dVBE = VT*ln(8)`` across R1, and the PTAT branch current is
mirrored into R2 stacked on a third 1x NPN so that ::

    VREF = VBE3 + (R2/R1) * dVBE  ~=  1.2 V

The loop needs no opamp: the PMOS mirror (M1/M2, gates on node Y)
and the NMOS mirror (M3/M4, gates on node X) force all three legs
to carry the same current, and the shared NMOS gate then enforces
``I*R1 = VBE1 - VBE2``.  Legs ::

    left:   VDD -> M1 -> X -> M3 -> VE1 -> Q1 (1x) -> VSS-side
    right:  VDD -> M2 -> Y -> M4 -> R1 -> VE2 -> Q2 (8x) -> VSS-side
    output: VDD -> M6 -> VREF -> R2 -> VE3 -> Q3 (1x) -> VSS-side

with M2 diode-connected (PG = Y) and M3 diode-connected (NG = X).

Technology mapping notes (gf180 only)
--------------------------------------
* The NPNs use the ``(5.0, 5.0)`` ``draw_bjt`` unit.  The no-dnwell
  NPN shares one pwell for its emitter/base/collector diffusions, so
  extraction merges the collector with the emitter: every unit is
  therefore used (and modelled) with C strapped to E, and the base
  returned to the VSS rail.  The reference netlist instantiates the
  extracted ``npn_05p00x05p00`` device cell directly (``X`` instances),
  which is what magic extracts and what the gf180 netgen setup
  matches on both sides.
* R1/R2 are the foundry-agnostic MOS-diode ``resistor`` primitive, so
  the cell stays inside the LVS-proven device set.  At equal branch
  currents the series count ratio sets the bandgap gain exactly
  (``V_R2/V_R1 = N2/N1``); the absolute tempco of a MOS diode is
  worse than poly, so a future revision may swap in real resistors.
* No dedicated startup circuit is included: like the thesis
  reference (Fig. 3.3), startup is a separate block.  The zero-current
  state of the self-biased loop is stable, so the cell needs an
  external kick (or a startup leg) to leave it.
* sky130 has no valid BJT sizes in the mapped PDK, so the cell raises
  ``NotImplementedError`` there (same precedent as the sky130-only
  ``vt_ref`` cell, mirrored).
"""
from __future__ import annotations

import os
from contextlib import contextmanager
from typing import Optional

from pydantic import validate_arguments

from glayout.backend import Component, rectangle
from glayout.pdk.mappedpdk import MappedPDK
from glayout.primitives.bjt import draw_bjt
from glayout.primitives.fet import fet_netlist, nmos, pmos
from glayout.primitives.guardring import tapring
from glayout.primitives.resistor import resistor, resistor_netlist
from glayout.primitives.via_gen import via_stack
from glayout.routing import c_route, L_route, straight_route
from glayout.spice import Netlist
from glayout.util.comp_utils import (
    align_comp_to_port,
    evaluate_bbox,
    move,
    prec_ref_center,
)
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.util.snap_to_grid import component_snap_to_grid


@contextmanager
def _no_sub_cell_pin_labels():
    """Build children without their standalone gf180 pin labels.

    Same idiom as ``sample_hold_cell`` / ``low_voltage_cmirror``: inner
    labels would otherwise flatten into this cell's GDS and extract as
    extra top-level pins.
    """
    prev = os.environ.get("GLAYOUT_NO_PIN_LABELS")
    os.environ["GLAYOUT_NO_PIN_LABELS"] = "1"
    try:
        yield
    finally:
        if prev is None:
            os.environ.pop("GLAYOUT_NO_PIN_LABELS", None)
        else:
            os.environ["GLAYOUT_NO_PIN_LABELS"] = prev


def bjt_netlist() -> Netlist:
    """Reference leaf for one ``npn_05p00x05p00`` unit device.

    The leaf carries no source template: both the extracted layout and
    this schematic instantiate the *undefined* ``npn_05p00x05p00`` cell,
    which netgen matches as a black-box device class (same mechanism
    the gf180 setup file's NPN entries rely on).  Terminal order is
    (C, B, E, S); parents strap C to the emitter net to mirror what
    extraction produces for the shared-well structure.
    """
    return Netlist(
        circuit_name="npn_05p00x05p00",
        nodes=["C", "B", "E", "S"],
    )


def bandgap_netlist(
    pdk: MappedPDK,
    mirror_width: float = 5.0,
    mirror_length: float = 1.0,
    res_width: float = 3.0,
    res_length: float = 1.0,
    r1_series: int = 1,
    r2_series: int = 9,
    q2_units: int = 8,
    n_dummies: int = 2,
) -> Netlist:
    """Reference netlist for :func:`bandgap`.

    Nodes are ``[VDD, VREF, B]``: the pwell (BJT bases, NFET bulks) is
    the same physical net as the substrate in extraction, so there is
    no separate VSS pin -- ``B`` is the ground/substrate reference.
    Every BJT ties C to its emitter net; each dummy straps C/B/E onto
    its own floating ``DUMi`` net while its substrate terminal stays
    on ``B``.
    """
    netlist = Netlist(circuit_name="BANDGAP", nodes=["VDD", "VREF", "B"])

    def _add_fet(model: str, d: str, g: str, s: str, b: str) -> None:
        leaf = fet_netlist(
            pdk=pdk,
            circuit_name="BANDGAP_PFET" if model == pmos_model else "BANDGAP_NFET",
            model=model,
            width=mirror_width,
            length=mirror_length,
            fingers=1,
            multipliers=1,
            with_dummy=False,
        )
        # DUM maps to B (this cell never builds FET dummies, so the DUM
        # node would otherwise dangle device-free on one side only).
        netlist.connect_netlist(leaf, [("D", d), ("G", g), ("S", s), ("B", b), ("DUM", "B")])

    pmos_model = pdk.models["pfet"]
    nmos_model = pdk.models["nfet"]

    # PMOS mirror trio (common gate PG = node Y)
    _add_fet(pmos_model, "X", "Y", "VDD", "VDD")      # M1
    _add_fet(pmos_model, "Y", "Y", "VDD", "VDD")      # M2 (diode ref)
    _add_fet(pmos_model, "VREF", "Y", "VDD", "VDD")   # M6 (output leg)
    # NMOS mirror pair (common gate NG = node X); bulks on B (pwell is
    # substrate in extraction, so no separate VSS exists).
    _add_fet(nmos_model, "X", "X", "VE1", "B")        # M3 (diode ref)
    _add_fet(nmos_model, "Y", "X", "VR1T", "B")       # M4

    # Resistors (MOS-diode stacks)
    # NOTE: the layout stacks series units bottom-up with unit 0 (the P
    # side of the diode chain) at the bottom, so P maps to the lower
    # potential net and N to the upper one.
    r1 = resistor_netlist(pdk, width=res_width, length=res_length,
                          num_series=r1_series, multipliers=1)
    netlist.connect_netlist(r1, [("P", "VE2"), ("N", "VR1T"), ("B", "VDD")])
    r2 = resistor_netlist(pdk, width=res_width, length=res_length,
                          num_series=r2_series, multipliers=1)
    netlist.connect_netlist(r2, [("P", "VE3"), ("N", "VREF"), ("B", "VDD")])

    # Signal BJTs: Q1 + Q3 (1x) and Q2 (q2_units x); bases on B (pwell
    # is substrate in extraction).
    netlist.connect_netlist(bjt_netlist(), [("C", "VE1"), ("B", "B"), ("E", "VE1"), ("S", "B")])
    for _ in range(q2_units):
        netlist.connect_netlist(bjt_netlist(), [("C", "VE2"), ("B", "B"), ("E", "VE2"), ("S", "B")])
    netlist.connect_netlist(bjt_netlist(), [("C", "VE3"), ("B", "B"), ("E", "VE3"), ("S", "B")])
    # Dummy (edge-termination) BJTs: each straps E/B/C locally (its own
    # DUMi net), mirroring the layout's per-dummy E-B met2 strap.
    for i in range(n_dummies):
        dum = f"DUM{i}"
        netlist.connect_netlist(bjt_netlist(), [("C", dum), ("B", dum), ("E", dum), ("S", "B")])
    return netlist


def _require_gf180(pdk: MappedPDK) -> None:
    if pdk.name.lower() != "gf180":
        raise NotImplementedError(
            "The bandgap core needs vertical NPNs; the mapped "
            f"{pdk.name!r} PDK has no valid BJT sizes."
        )


def _via_at(parent: Component, pdk: MappedPDK, x: float, y: float,
            bottom: str = "met1", top: str = "met2"):
    """Drop a via stack centred at (x, y); return its reference."""
    ref = parent << via_stack(pdk, bottom, top)
    ref.move((float(x), float(y)))
    return ref


def _bar(parent: Component, pdk: MappedPDK, cx: float, cy: float,
         w: float, h: float, glayer: str = "met2"):
    """Axis-aligned bus bar centred at (cx, cy); return its reference."""
    ref = parent << rectangle(layer=pdk.get_glayer(glayer), size=(w, h), centered=True)
    ref.move((float(cx), float(cy)))
    return ref


def _hbox(parent: Component, pdk: MappedPDK, x0: float, x1: float,
          y: float, glayer: str = "met2", width: float = 0.6):
    """Horizontal bus bar on `glayer`; return its reference."""
    return _bar(parent, pdk, (float(x0) + float(x1)) / 2, float(y),
                abs(float(x1 - x0)), width, glayer)


def _vbar(parent: Component, pdk: MappedPDK, x: float, y0: float,
          y1: float, glayer: str = "met2", width: float = 0.6):
    """Vertical bus bar on `glayer`; return its reference."""
    return _bar(parent, pdk, float(x), (float(y0) + float(y1)) / 2,
                width, abs(float(y1 - y0)), glayer)


@validate_arguments
def bandgap(
    pdk: MappedPDK,
    mirror_width: float = 5.0,
    mirror_length: float = 1.0,
    res_width: float = 3.0,
    res_length: float = 1.0,
    r1_series: int = 1,
    r2_series: int = 9,
    q2_units: int = 8,
) -> Component:
    """Build the bandgap core layout; see module docstring for topology."""
    _require_gf180(pdk)
    n_dummies = 2
    top = Component()

    with _no_sub_cell_pin_labels():
        unit_proto = draw_bjt(pdk, (5.0, 5.0), "npn",
                              draw_dnwell=False, with_labels=False)
        unit_w = float(evaluate_bbox(unit_proto)[0])
        fet_kwargs = dict(width=mirror_width, fingers=1, multipliers=1,
                          length=mirror_length, with_dummy=False,
                          with_tie=True, with_substrate_tap=False,
                          sd_route_topmet="met2", gate_route_topmet="met2")
        pmos_proto = pmos(pdk, **fet_kwargs)
        nmos_proto = nmos(pdk, with_dnwell=False, **fet_kwargs)
        res_kwargs = dict(width=res_width, fingers=1, multipliers=1,
                          length=res_length, with_dummy=False,
                          with_tie=False, with_substrate_tap=False,
                          sd_route_topmet="met2", gate_route_topmet="met2")
        res_proto = pmos(pdk, **res_kwargs)

    # --- BJT row -----------------------------------------------------
    # Order: DUM | Q2 x4 | Q1 | Q3 | Q2 x4 | DUM  (Q1/Q3 share the centre)
    roles: list[str] = (
        ["DUM"] + ["Q2"] * (q2_units // 2) + ["Q1", "Q3"]
        + ["Q2"] * (q2_units - q2_units // 2) + ["DUM"]
    )
    pitch = unit_w + 2.4
    xs = [round((i - (len(roles) - 1) / 2) * pitch, 3) for i in range(len(roles))]
    with _no_sub_cell_pin_labels():
        for x in xs:
            ref = top << draw_bjt(pdk, (5.0, 5.0), "npn",
                                  draw_dnwell=False, with_labels=False)
            ref.move((x, 0.0))
    # role -> list of unit indices (roles repeat: Q2, DUM)
    role_idx: dict[str, list[int]] = {}
    for i, r in enumerate(roles):
        role_idx.setdefault(r, []).append(i)
    x_q1 = xs[role_idx["Q1"][0]]
    x_q3 = xs[role_idx["Q3"][0]]
    x_q2 = [xs[i] for i in role_idx["Q2"]]
    x_dum = [xs[i] for i in role_idx["DUM"]]

    # Per-unit base drops: a via on each unit's south base segment with a
    # met2 bar down to the VSS rail.  (A continuous met1 bus is illegal
    # here: it would cross every unit's collector ring bars and short the
    # VSS net to the emitter nets.  The base rings are p+ in the
    # shared-well structure, so these drops also tie each pwell to VSS.)
    # Dummy units instead strap E-to-B locally, giving each its own DUMi
    # net (mirrored in the reference netlist).
    base_y = -3.03
    vss_y = -9.5
    _hbox(top, pdk, xs[0] - 6.0, xs[-1] + 6.0, vss_y)
    dum_nets = 0
    for i, x in enumerate(xs):
        _via_at(top, pdk, x, 0.0)  # emitter via (met1 -> met2)
        _via_at(top, pdk, x, base_y)  # base via (met1 -> met2)
        if roles[i] == "DUM":
            _vbar(top, pdk, x, base_y, 0.0)  # local E-B strap on met2
            dum_nets += 1
        else:
            _vbar(top, pdk, x, vss_y, base_y)  # base drop to VSS rail

    # --- FET block ---------------------------------------------------
    # PMOS trio (M1/M2/M6) above, NMOS pair (M3/M4) below it.  Right leg
    # is vertical (M2 over M4); S/D ports face up on both primitives.
    with _no_sub_cell_pin_labels():
        pmos_units = [top << pmos(pdk, **fet_kwargs) for _ in range(3)]
        nmos_units = [top << nmos(pdk, with_dnwell=False, **fet_kwargs)
                      for _ in range(2)]
    m1, m2, m6 = pmos_units
    m3, m4 = nmos_units
    y_pmos = 26.0
    y_nmos = 12.0
    x_m1, x_m2, x_m6 = x_q1 - 10.0, x_q1 + 10.0, x_q3 + 10.0
    x_m3, x_m4 = x_q1, x_q1 + 10.0
    for ref, x in zip((m1, m2, m6), (x_m1, x_m2, x_m6)):
        ref.move((x, y_pmos))
    for ref, x in zip((m3, m4), (x_m3, x_m4)):
        ref.move((x, y_nmos))

    # Rails and mirror-gate buses.
    vdd_y = y_pmos + 8.0
    y_pg = 20.0  # PG = node Y bus
    y_ng = 6.0   # NG = node X bus
    _hbox(top, pdk, x_m1 - 6.0, 73.4, vdd_y)                # VDD rail
    # (east end reaches the R1-tie riser at x=73.4)
    _hbox(top, pdk, x_m1 - 6.0, x_m6 + 6.0, y_pg)            # PG bus
    _hbox(top, pdk, x_m1 - 2.0, x_m4 + 3.4, y_ng)            # NG bus
    # PMOS sources up to VDD via the SIDE ports (source_E/W): a centred
    # vertical vbar would overlap the drain pad one level down (same
    # x-range, 0.3um pad gap) -- an invisible VDD-to-drain short.
    # M1 exits east, M2/M6 exit west (their Y-dogleg and VREF jog own
    # the east sides).
    for ref, side in ((m1, 1.0), (m2, -1.0), (m6, -1.0)):
        port = ref.ports["source_E" if side > 0 else "source_W"]
        px, py = float(port.center[0]), float(port.center[1])
        _hbox(top, pdk, px, px + side * 0.8, py - 0.2)
        _vbar(top, pdk, px + side * 0.8, py - 0.2, vdd_y)
    for ref in pmos_units:  # PMOS nwell ties up to VDD
        port = ref.ports["tie_N_top_met_N"]
        _via_at(top, pdk, float(port.center[0]), float(port.center[1]),
                bottom="met1", top="met2")
        _vbar(top, pdk, float(port.center[0]), float(port.center[1]), vdd_y)
    for ref in pmos_units:  # PMOS gates to PG
        port = ref.ports["gate_E"]
        _vbar(top, pdk, float(port.center[0]), y_pg,
              float(port.center[1]))

    # NMOS pwell ties: met1 drops (crossing the met2 NG bus safely) via'd
    # onto the met2 tie bus, which runs west into the VSS rail.
    tie_bus_y = 5.0
    x_tiebus = xs[0] - 5.0
    _hbox(top, pdk, x_tiebus, x_m4 + 6.0, tie_bus_y)
    for ref in nmos_units:
        port = ref.ports["tie_S_top_met_S"]
        px, py = float(port.center[0]), float(port.center[1])
        _vbar(top, pdk, px, tie_bus_y, py + 0.5, glayer="met1")
        _via_at(top, pdk, px, tie_bus_y, bottom="met1", top="met2")
    _vbar(top, pdk, x_tiebus, vss_y, tie_bus_y)
    _hbox(top, pdk, x_tiebus, x_m3 - 6.0, tie_bus_y)

    # NMOS gates to NG: M3 drops straight; M4 jogs west first.
    _vbar(top, pdk, float(m3.ports["gate_E"].center[0]), y_ng,
          float(m3.ports["gate_E"].center[1]))
    m4gx, m4gy = float(m4.ports["gate_E"].center[0]), float(m4.ports["gate_E"].center[1])
    _hbox(top, pdk, x_m4 - 3.0, m4gx, m4gy)
    _vbar(top, pdk, x_m4 - 3.0, y_ng, m4gy)

    # Y net: M2 drain + M4 drain join the PG bus.  The S/D top pads sit
    # only ~0.3um apart in y, so any due-east stub violates met2 spacing
    # to the (other-net) source pad; dogleg north first, then east, then
    # run one joint drop into the bus.
    x_ydog = float(m2.ports["drain_N"].center[0]) + 0.95
    x_ydrop = x_m4 + 2.5
    y_dog = []
    for ref in (m2, m4):
        dp = ref.ports["drain_N"]
        px, py = float(dp.center[0]), float(dp.center[1])
        _vbar(top, pdk, x_ydog, py - 0.1, py + 1.2)
        _hbox(top, pdk, x_ydog, x_ydrop, py + 0.7)
        y_dog.append(py + 0.7)
    _vbar(top, pdk, x_ydrop, y_dog[1], y_dog[0])

    # X net: M1 drain and M3 drain drop to the NG bus on met3 (they must
    # cross the PG/VSS met2 buses, so met2 is illegal here).
    # NOTE: M1's via is offset west of the drain pad centre: a centred
    # via pad overlaps M1's own source vbar (VDD) -- an invisible short.
    # M3's drop is offset EAST (+1.5): x_m3 == x_q1, so a centred drop
    # would land exactly on the VE1 met3 highway -- an invisible X-VE1
    # short (DRC-clean, LVS-fatal).
    for ref, x, xoff in ((m1, x_m1, -1.2), (m3, x_m3, 1.5)):
        port = ref.ports["drain_N"]
        px, py = float(port.center[0]), float(port.center[1])
        _via_at(top, pdk, px + xoff, py, bottom="met2", top="met3")
        if xoff < 0:
            # Stop 0.7 east of the via: reaching the pad centre would
            # overlap M1's source vbar (VDD) one pad over.
            _hbox(top, pdk, px + xoff, px - 0.7, py, glayer="met2")
        else:
            # Start on the drain pad (same net) and run east to the via.
            _hbox(top, pdk, px, px + xoff, py, glayer="met2")
        _vbar(top, pdk, px + xoff, y_ng, py, glayer="met3")
        _via_at(top, pdk, px + xoff, y_ng, bottom="met2", top="met3")

    # VE1 (M3 source -> Q1 emitter): leave the source pad from its EAST
    # side port, 0.3 below pad-centre level (the drain bar hangs low;
    # pad-band level would violate spacing to it), then met3 highway
    # down, stacked onto the emitter via.
    ps = m3.ports["source_E"]
    sx, sy = float(ps.center[0]), float(ps.center[1])
    ey = sy - 0.3
    _hbox(top, pdk, x_q1, sx, ey, glayer="met2")
    _via_at(top, pdk, x_q1, ey, bottom="met2", top="met3")
    _vbar(top, pdk, x_q1, 0.0, ey, glayer="met3")
    _via_at(top, pdk, x_q1, 0.0, bottom="met2", top="met3")

    # --- Q2 emitter bus (VE2) -----------------------------------------
    # NOTE: the bus sits at y=4.0 and the stubs span only y -0.1..4.3.
    # Taller stubs would cross the met2 tie bus (y=5) and the NG bus
    # (y=6) -- same-layer overlaps that DRC cannot see but that short
    # VE2 to VSS and X in extraction.
    y_q2bus = 4.0
    for x in x_q2:
        _vbar(top, pdk, x, -0.1, y_q2bus + 0.3)
    _hbox(top, pdk, min(x_q2) - 3.3, 68.2, y_q2bus)

    # --- R1 (single MOS diode, east of the array) ----------------------
    with _no_sub_cell_pin_labels():
        r1ref = top << pmos(pdk, **{**res_kwargs, "with_tie": True})
    x_r1, y_r1 = 70.0, 2.0
    r1ref.move((x_r1, y_r1))
    # NOTE: cglayer="met2" keeps the strap on one metal (plain rects, no
    # vias).  The default met3 jog drops via stacks whose bottom mcon can
    # land on field and short the strap net to substrate -- invisible to
    # DRC, fatal to LVS.
    top << c_route(pdk, r1ref.ports["gate_W"], r1ref.ports["drain_W"],
                   cglayer="met2")
    # VR1T (M4 source -> R1 source): leave from the EAST side port 0.3
    # below pad-centre level (same low-hanging drain bar), run east,
    # drop south clear of every R1 bar, land on the source bar.
    ps4 = m4.ports["source_E"]
    ry = float(ps4.center[1]) - 0.3
    _hbox(top, pdk, float(ps4.center[0]) - 0.3, 72.0, ry)
    _vbar(top, pdk, 72.0, 3.83, ry)
    _hbox(top, pdk, 70.0, 72.0, 3.83)
    # VE2 (R1 drain/gate -> Q2 bus): met3 hop west, then south on met2.
    # The south run sits at x=68.2 so it OVERLAPS the diode strap's west
    # end (same net) instead of near-missing it.
    pd_e = r1ref.ports["drain_E"]
    pdx, pdy = float(pd_e.center[0]), float(pd_e.center[1])
    _via_at(top, pdk, pdx, pdy, bottom="met2", top="met3")
    _hbox(top, pdk, 68.2, pdx, pdy, glayer="met3")
    _via_at(top, pdk, 68.2, pdy, bottom="met2", top="met3")
    _vbar(top, pdk, 68.2, float(r1ref.ports["gate_S"].center[1]), pdy)
    _hbox(top, pdk, 68.2, float(r1ref.ports["gate_S"].center[0]),
          float(r1ref.ports["gate_S"].center[1]))
    # R1 nwell tie -> VDD: north stub onto the tie via, spur EAST to
    # x=71, met3 hop OVER the VR1T south vbar (x=72, same layer would
    # short VDD to VR1T -- DRC-clean, LVS-fatal), then a riser up to
    # the VDD rail (extended east to meet it).
    pt_n = r1ref.ports["tie_N_top_met_N"]
    ptx, pty = float(pt_n.center[0]), float(pt_n.center[1])
    _via_at(top, pdk, ptx, pty, bottom="met1", top="met2")
    _vbar(top, pdk, ptx, pty, 9.0)
    _hbox(top, pdk, 28.5, 71.0, 9.0)
    _via_at(top, pdk, 71.0, 9.0, bottom="met2", top="met3")
    _hbox(top, pdk, 71.0, 73.4, 9.0, glayer="met3")
    _via_at(top, pdk, 73.4, 9.0, bottom="met2", top="met3")
    _vbar(top, pdk, 73.4, 9.0, vdd_y)

    # --- R2 (series MOS-diode row above the FET block) -----------------
    y_r2 = 44.0
    r_units = []
    with _no_sub_cell_pin_labels():
        for i in range(r2_series):
            ref = top << pmos(pdk, **res_kwargs)
            ref.move((-24.0 + 6.0 * i, y_r2))
            r_units.append(ref)
    for ref in r_units:  # diode strap per unit (G tied to D, west side)
        top << c_route(pdk, ref.ports["gate_W"], ref.ports["drain_W"],
                       cglayer="met2")
    # Series rungs S_i -> D_{i+1}: leave each pad from its EAST side at
    # side-port level (a centred stub would overlap the neighbour pad
    # 0.3 away -- the same invisible short as the mirror sources), jog
    # north clear of the pads, bus over, and drop onto the next drain.
    for a, b in zip(r_units[:-1], r_units[1:]):
        pa = a.ports["source_E"]
        pb = b.ports["drain_E"]
        ax, ay = float(pa.center[0]), float(pa.center[1])
        bx, by = float(pb.center[0]), float(pb.center[1])
        _hbox(top, pdk, ax - 0.3, ax + 1.5, ay - 0.3)
        _vbar(top, pdk, ax + 1.5, ay - 0.3, 48.2)
        _hbox(top, pdk, ax + 1.5, bx - 1.5, 48.2)
        _vbar(top, pdk, bx - 1.5, by + 0.15, 48.2)
        _hbox(top, pdk, bx - 1.5, bx, by + 0.15)
    # R2 nwell ring (own nwell, strapped to VDD twice).
    ring2 = top << tapring(pdk, enclosed_rectangle=(56.0, 12.0),
                           sdlayer="n+s/d", horizontal_glayer="met2",
                           vertical_glayer="met1")
    ring2.move((0.0, y_r2))
    for x in (-20.0, 20.0):
        _vbar(top, pdk, x, vdd_y, y_r2 - 5.5)

    # VE3 (Q3 emitter -> R2 unit0 gate): met2 jog, met3 east+north+west.
    _hbox(top, pdk, x_q3, x_q3 + 2.2, 0.0, glayer="met2")
    _via_at(top, pdk, x_q3 + 2.2, 0.0, bottom="met2", top="met3")
    _hbox(top, pdk, x_q3 + 2.2, 26.0, 0.0, glayer="met3")
    _vbar(top, pdk, 26.0, 0.0, y_r2, glayer="met3")
    _hbox(top, pdk, -23.5, 26.0, y_r2, glayer="met3")
    _via_at(top, pdk, -23.5, y_r2, bottom="met2", top="met3")
    _vbar(top, pdk, -23.5, float(r_units[0].ports["gate_E"].center[1]), y_r2)

    # VREF (M6 drain -> R2 last-unit SOURCE = N side of the chain): met2
    # jog, met3 riser, stub onto the source pad.  NOTE: the stub must
    # land on the source pad (unit8 S = N) and NOT the drain pad 0.3
    # above it (that is INT_8) -- same invisible-short family.
    # The M6D jog must OVERLAP the drain bar (same net) while clearing
    # the source bar below it: y=29.8 overlaps the drain bar (top
    # 29.945) and clears the source bar (top 29.145) by 0.35.
    _hbox(top, pdk, 16.0, 27.0, 29.8)
    _via_at(top, pdk, 27.0, 29.8, bottom="met2", top="met3")
    _vbar(top, pdk, 27.0, 29.8, 45.8, glayer="met3")
    _via_at(top, pdk, 27.0, 45.8, bottom="met2", top="met3")
    _hbox(top, pdk, float(r_units[-1].ports["source_N"].center[0]), 27.0, 45.8)
    vref_pin = (float(r_units[-1].ports["source_N"].center[0]) + 27.0) / 2

    # --- substrate ring, pins, netlist ---------------------------------
    ringB = top << tapring(pdk, enclosed_rectangle=(158.0, 76.0),
                           sdlayer="p+s/d", horizontal_glayer="met2",
                           vertical_glayer="met1")
    ringB.move((4.0, 20.0))

    def _pin(text: str, x: float, y: float, glayer: str) -> None:
        lbl = rectangle(layer=pdk.get_glayer(glayer + "_pin"),
                        size=(0.5, 0.5), centered=True)
        ref = top << lbl
        ref.move((x, y))
        comp = Component()
        top.add(comp.add_label(text, position=(x, y),
                               layer=pdk.get_glayer(glayer + "_label")))

    _pin("VDD", x_m1 - 6.0, vdd_y, "met2")
    _pin("VREF", vref_pin, 45.8, "met2")
    _bport = ringB.ports["W_top_met_N"]
    _pin("B", float(_bport.center[0]), float(_bport.center[1]), "met2")

    flat = top.flatten()
    flat.info["netlist"] = bandgap_netlist(
        pdk, mirror_width, mirror_length, res_width, res_length,
        r1_series, r2_series, q2_units, 2,
    )
    return component_snap_to_grid(rename_ports_by_orientation(flat))
