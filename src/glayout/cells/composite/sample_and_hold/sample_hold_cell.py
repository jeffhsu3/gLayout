import os
from contextlib import contextmanager

from glayout.backend import Component, Port, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.cells.elementary.transmission_gate.transmission_gate import (
    transmission_gate,
)
from glayout.primitives.fet import nmos
from glayout.primitives.mimcap import mimcap_array
from glayout.spice.netlist import Netlist
from glayout.util.label_utils import expose_ports, add_pin_labels
from glayout.util.port_utils import set_port_orientation


_METAL_GLAYERS = ("met1", "met2", "met3", "met4", "met5")


def _stack_glayers(bot_glayer: str, top_glayer: str) -> tuple[str, ...]:
    """The metal glayers a via stack from `bot_glayer` to `top_glayer` occupies."""
    i0 = _METAL_GLAYERS.index(bot_glayer)
    i1 = _METAL_GLAYERS.index(top_glayer)
    return _METAL_GLAYERS[min(i0, i1) : max(i0, i1) + 1]


def _landing_size(
    pdk: MappedPDK, via_comp: Component, glayer: str
) -> tuple[float, float]:
    """(width, height) of a via stack's landing pad on `glayer`, from geometry.

    Deliberately NOT read off the stack's bottom_met_* ports: those coincide with
    the pad edges only while `fullbottom=False`, so port arithmetic would silently
    change meaning if that flag were ever flipped.
    """
    (x0, y0), (x1, y1) = via_comp.extract([pdk.get_glayer(glayer)]).bbox
    return x1 - x0, y1 - y0


@contextmanager
def _no_sub_cell_pin_labels():
    """Suppress sub-cell pin labels while building children.

    transmission_gate and nmos stamp VIN/VOUT/VCC/VSS/VGP/VGN labels on gf180 --
    klayout's deck requires named pins -- gated on GLAYOUT_NO_PIN_LABELS
    (transmission_gate.py:186). Left enabled, those inner labels flatten into this
    cell's GDS and klayout extracts them as extra top-level pins: VGP and VGN land on
    the very nets this cell labels CLK_B and CLK, so LVS reports
    "extra top-level pin(s) in layout: CLK_B|VGP, CLK|VGN".

    Same idiom as low_voltage_cmirror.py:129. Note the sub-cells are @cell-cached by
    argument, so a build earlier in the same process with labels enabled would be
    reused as-is; the DRC/LVS runner builds each cell in a fresh process.
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


def _mimcap_glayers(pdk: MappedPDK) -> tuple[str, str]:
    """(bottom, top) plate glayers of the PDK's MIM stack.

    glayout's mimcap registers only its via array's ports (mimcap.py:323, and the
    array is built with lay_bottom=False), so the bottom plate carries no port to
    read a layer off. Take it from the PDK's own capmet rule -- the same source
    mimcap itself uses -- which keeps this PDK-agnostic without needing a port.
    """
    return (
        pdk.layer_to_glayer(pdk.get_grule("capmet")["capmetbottom"]),
        pdk.layer_to_glayer(pdk.get_grule("capmet")["capmettop"]),
    )


def _bottom_plate_east_port(pdk: MappedPDK, mim_comp, mim_ref, bottom_glayer: str) -> Port:
    """East edge of the mimcap's bottom plate, as a Port, derived from geometry.

    The plate is unported (see _mimcap_glayers). The nested per-via port
    ``row0_col0_array_row0_col0_bottom_met_E`` is NOT a stand-in: it sits at the
    first via of the array, which on a 10x10 sky130 cap is 10.1um inside the plate
    edge -- a route to it would land deep inside the capacitor.

    ComponentReference has no .extract, so measure on the Component and offset by
    the reference. Valid for translated refs, which is all this cell places.
    """
    (cx0, cy0), (cx1, cy1) = mim_comp.bbox
    (px0, py0), (px1, py1) = mim_comp.extract([pdk.get_glayer(bottom_glayer)]).bbox
    off_x = (px1 + px0) / 2 - (cx1 + cx0) / 2
    off_y = (py1 + py0) / 2 - (cy1 + cy0) / 2
    return Port(
        name="bottom_plate_E",
        center=(mim_ref.center[0] + off_x + (px1 - px0) / 2,
                mim_ref.center[1] + off_y),
        width=py1 - py0,
        orientation=0,
        layer=pdk.get_glayer(bottom_glayer),
    )


def _foreign_x_intervals(
    pdk: MappedPDK,
    host,
    glayer: str,
    y_lo: float,
    y_hi: float,
    own_probe: tuple[float, float] | None = None,
) -> list[tuple[float, float]]:
    """x-intervals of metal on `glayer` inside the y-band [y_lo, y_hi] that is NOT
    galvanically joined to the island `port` sits on.

    Geometry-based on purpose. The shapes that crowd a via landing next to a FET
    terminal are fet.py's inter-multiplier c_routes, which carry the neighbouring
    net up to `src_extension` (0.6 um, hardcoded there) past the rail end and
    expose NO port describing that extent -- so a clearance computed from port
    centers under-estimates the real one as soon as multipliers > 1.
    """
    import gdstk

    raw = host.get_polygons(by_spec=pdk.get_glayer(glayer))
    if len(raw) == 0:
        return []
    islands = gdstk.boolean([gdstk.Polygon(p) for p in raw], [], "or")
    # On the port's own layer, its island is the net we are landing on, so exclude it.
    # On the stack's other layers there is no such island yet (the via is what will
    # create it), so every shape counts -- conservative, and never wrong.
    if own_probe is None:
        foreign = list(islands)
    else:
        foreign = [p for p in islands if not gdstk.inside([own_probe], p)[0]]
    if not foreign:
        return []
    bounds = [p.bounding_box() for p in foreign]
    band = gdstk.rectangle(
        (min(b[0][0] for b in bounds) - 1.0, y_lo),
        (max(b[1][0] for b in bounds) + 1.0, y_hi),
    )
    intervals = []
    for piece in gdstk.boolean(foreign, [band], "and"):
        (bx0, _), (bx1, _) = piece.bounding_box()
        intervals.append((bx0, bx1))
    return sorted(intervals)


def _drop_via_beside(
    parent: Component, pdk: MappedPDK, port, top_glayer: str, host
):
    """Drop a via stack on `port`, slid along the port axis until its landing pad
    clears every foreign net in `host`, then stitch it with a rail-width route.

    The landing pad can be far wider than the rail it serves (on sky130 a 0.43 um
    met2 pad on a 0.29 um S/D rail), so it does not always fit inside the rail's
    own y-band. The stitching route inherits the PORT's width, not the pad's, so
    it never widens the rail.

    `host` is the device that owns the port. The search is scoped to it on purpose:
    a landing that only fits outside the device is a sizing problem, and sliding
    across the cell into a neighbouring block is never the right answer.

    Returns the via reference.
    """
    from glayout.primitives.via_gen import via_stack
    from glayout.routing.straight_route import straight_route

    orient = round(port.orientation) % 360
    if orient not in (0, 180):
        raise ValueError(
            f"_drop_via_beside needs an E/W-facing port, got orientation "
            f"{port.orientation} on {port.name}"
        )
    outward = -1 if orient == 180 else 1  # a W-facing port slides its pad west

    bot_glayer = pdk.layer_to_glayer(port.layer)
    sep = pdk.get_grule(bot_glayer)["min_separation"]
    via_comp = via_stack(pdk, bot_glayer, top_glayer, fulltop=True, fullbottom=False)

    # A via stack is not just its landing: it occupies EVERY metal from bot to top,
    # and those footprints differ wildly (sky130: 0.43 / 0.58 / 1.50 / 1.50 um) with a
    # different min_separation each. All of them shift together in x, so solve them
    # simultaneously -- clearing only the landing just moves the violation up a layer.
    inward = 1.0 if orient == 180 else -1.0
    own_probe = (
        port.center[0] + inward * min(0.005, float(port.width) / 4),
        port.center[1],
    )
    constraints = []  # (glayer, footprint_width, min_separation, blocking x-intervals)
    for g in _stack_glayers(bot_glayer, top_glayer):
        w_g, h_g = _landing_size(pdk, via_comp, g)
        if w_g <= 0 or h_g <= 0:
            continue  # stack does not reach this metal
        sep_g = pdk.get_grule(g)["min_separation"]
        constraints.append(
            (
                g,
                w_g,
                sep_g,
                _foreign_x_intervals(
                    pdk,
                    parent,
                    g,
                    port.center[1] - h_g / 2 - sep_g,
                    port.center[1] + h_g / 2 + sep_g,
                    own_probe=own_probe if g == bot_glayer else None,
                ),
            )
        )

    tol = 1e-9  # candidates land exactly on a limit; absorb float round-off

    def fits(cx: float) -> bool:
        for _g, w_g, sep_g, blk in constraints:
            lo, hi = cx - w_g / 2, cx + w_g / 2
            if not all(
                hi - tol <= a - sep_g or lo + tol >= b + sep_g for a, b in blk
            ):
                return False
        return True

    # The displacement is bounded by the owning device. Scanning the whole parent
    # keeps the solver honest about what is really there, but without this bound a
    # crowded landing would happily "solve" itself by sliding into a neighbouring
    # block on the other side of the cell.
    (hx0, _), (hx1, _) = host.bbox
    reach = max(w_g for _g, w_g, _s, _b in constraints) + max(
        s for _g, _w, s, _b in constraints
    )
    # candidate 0 = centred on the port (no displacement); the rest step some layer's
    # footprint just clear of a blocker edge. Nearest feasible candidate wins.
    candidates = [port.center[0]]
    for _g, w_g, sep_g, blk in constraints:
        for a, b in blk:
            candidates.append(
                (a - sep_g - w_g / 2) if outward < 0 else (b + sep_g + w_g / 2)
            )
    candidates = [
        c
        for c in candidates
        if outward * (c - port.center[0]) >= -tol and hx0 - reach <= c <= hx1 + reach
    ]
    feasible = [
        c
        for c in sorted(candidates, key=lambda c: abs(c - port.center[0]))
        if fits(c)
    ]
    if not feasible:
        detail = "; ".join(
            f"{g}: pad {w_g:.3f} um, sep {sep_g}, blocked by "
            f"{[(round(a, 3), round(b, 3)) for a, b in blk]}"
            for g, w_g, sep_g, blk in constraints
        )
        raise NotImplementedError(
            f"no legal via-stack position at {port.name} (y={port.center[1]:.3f}) "
            f"within the owning device [{hx0:.3f}, {hx1:.3f}] -- {detail}. Rebalance "
            f"the multipliers, widen the rails, or raise sd_rmult."
        )
    cx = feasible[0]

    if cx != port.center[0]:
        # grid snapping must not eat the clearance we just solved for, so step
        # outward until the snapped position still fits.
        grid = 2 * float(pdk.grid_size or 0.001)
        for step in range(5):
            snapped = float(pdk.snap_to_2xgrid(cx + outward * step * grid))
            if fits(snapped):
                cx = snapped
                break
        else:
            raise NotImplementedError(
                f"could not grid-snap the via landing at {port.name} without "
                f"re-entering a blocker"
            )

    v_ref = parent << via_comp
    v_ref.movex(cx - v_ref.center[0])
    v_ref.movey(port.center[1] - v_ref.center[1])
    parent << straight_route(
        pdk,
        port,
        v_ref.ports["bottom_met_E" if outward < 0 else "bottom_met_W"],
        glayer1=bot_glayer,
        glayer2=bot_glayer,
    )
    return v_ref


def _netlist_of(component: Component) -> Netlist:
    """The Netlist object a cell stored in ``info``.

    gdsfactory's @cell serializes ``info`` values, so ``info["netlist"]`` on a
    decorated cell (e.g. transmission_gate) comes back as a *str* while an
    undecorated one (mimcap_array) keeps the object. The decorator also stashes
    the original under ``netlist_obj`` plus a ``netlist_data`` dict, so prefer
    those. Same shim as FVF's get_component_netlist (fvf.py:22) -- worth
    centralising once a third caller needs it.
    """
    info = component.info
    if "netlist_obj" in info:
        return info["netlist_obj"]
    if "netlist_data" in info:
        data = info["netlist_data"]
        nl = Netlist(circuit_name=data["circuit_name"], nodes=data["nodes"])
        nl.source_netlist = data["source_netlist"]
        return nl
    return info["netlist"]


def _sample_hold_netlist(
    tg_comp: Component, mim_comp: Component, rst_comp: Component | None = None
) -> Netlist:
    """Composite source netlist for the sample-and-hold cell."""
    nodes = ["VIN", "CLK", "CLK_B", "VSS", "VOUT", "VCC"]
    if rst_comp is not None:
        nodes.extend(["RESET", "VZERO"])

    netlist = Netlist(
        circuit_name="sample_hold_cell",
        nodes=nodes,
    )

    netlist.connect_netlist(
        _netlist_of(tg_comp),
        [
            ("VIN", "VIN"),
            ("VSS", "VSS"),
            ("VOUT", "VOUT"),
            ("VCC", "VCC"),
            ("VGP", "CLK_B"),
            ("VGN", "CLK"),
        ],
    )

    netlist.connect_netlist(
        _netlist_of(mim_comp),
        [
            ("V1", "VOUT"),
            ("V2", "VSS"),
        ],
    )

    if rst_comp is not None:
        netlist.connect_netlist(
            _netlist_of(rst_comp),
            [
                ("D", "VOUT"),
                ("G", "RESET"),
                ("S", "VZERO"),
                ("B", "VSS"),
            ],
        )

    return netlist


@cell
def sample_hold_cell(
    pdk: MappedPDK,
    switch_width: tuple[float, float] = (1.0, 1.0),
    switch_fingers: tuple[int, int] = (2, 2),
    switch_multipliers: tuple[int, int] = (1, 1),
    cap_size: tuple[float, float] = (5.0, 5.0),
    cap_rows: int = 1,
    cap_cols: int = 1,
    with_reset: bool = True,
    reset_fingers: int = 1,
) -> Component:
    """
    A reusable Sample-and-Hold (S&H) storage cell.
    Samples VIN to VOUT on CLK/CLK_B.
    Optionally resets VOUT to VZERO when RESET is high.
    """
    # No explicit name: the @cell wrapper and the DRC/LVS runner both rename the
    # returned component, and a pre-registered "sample_hold_cell" collides with
    # that rename, so gdsfactory writes the top cell as "sample_hold_cell$1" and
    # the gf180 deck then cannot find it ("Cell name ... not found in input
    # layout"). Every other cell here uses a bare Component().
    top_level = Component()

    def place_left_of(left_ref, right_ref, separation: float):
        left_ref.movex(right_ref.xmin - left_ref.xmax - separation)
        left_ref.movey(right_ref.center[1] - left_ref.center[1])

    with _no_sub_cell_pin_labels():
        tg_comp = transmission_gate(
            pdk=pdk,
            width=switch_width,
            fingers=switch_fingers,
            multipliers=switch_multipliers,
            inter_finger_topmet="met1",
            # single-layer guard rings (matches taped-out gf180 practice, e.g. MPW18H1:
            tie_layers=("met1", "met1"),
        )
    tg_ref = top_level << tg_comp

    mim_comp = mimcap_array(
        pdk=pdk,
        rows=cap_rows,
        columns=cap_cols,
        size=cap_size,
    )
    mim_ref = top_level << mim_comp

    # Bbox abutment (the old separation of 0.0) is not a spacing guarantee, and on
    # PDKs that need the full-height bottom-plate exit it is actively wrong: that
    # exit is a via STACK dropped in the gap east of the plate, and the switch's own
    # VOUT stack has to fit east of it. A met2->met5 stack is 1.50 um wide on sky130
    # (intermediate-metal minimum area), so a flush cap left the two stacks
    # overlapping by 0.135 um -- shorting VOUT to VSS through physical met3.
    _mim_bottom_glayer = _mimcap_glayers(pdk)[0]
    placement_sep = pdk.util_max_metal_seperation()
    if pdk.get_grule("capmet").get("requires_full_height_bottom_plate_exit"):
        from glayout.primitives.via_gen import via_stack as _via_stack

        _exit_stack = _via_stack(
            pdk, "met2", _mim_bottom_glayer, fulltop=True, fullbottom=False
        )
        _exit_w = max(
            _landing_size(pdk, _exit_stack, g)[0]
            for g in _stack_glayers("met2", _mim_bottom_glayer)
        )
        # The exit stack needs clearance on BOTH flanks -- from the plate to its west
        # and from the switch's own VOUT stack to its east -- so budget two, not one.
        placement_sep += _exit_w + pdk.util_max_metal_seperation()

    place_left_of(mim_ref, tg_ref, placement_sep)

    rst_comp = None
    rst_ref = None
    if with_reset:
        if reset_fingers > 1:
            # :TODO auto-adjust to τ = Ron_reset × C_hold
            raise NotImplementedError
        with _no_sub_cell_pin_labels():
            rst_comp = nmos(
                pdk,
                width=1.0,
                length=pdk.get_grule("poly")["min_width"],
                fingers=1,
                multipliers=1,
                with_tie=True,
                with_dnwell=False,
                with_substrate_tap=False,
                inter_finger_topmet="met1",
                sd_route_topmet="met2",
                gate_route_topmet="met2",
                tie_layers=("met1", "met1"),  # see TG note above
            )
        rst_ref = top_level << rst_comp
        place_left_of(rst_ref, mim_ref, placement_sep)

    from glayout.primitives.via_gen import via_stack
    from glayout.routing.straight_route import straight_route

    def drop_via(layer1, layer2, port, *, fullbottom: bool = False):
        # Keep the bottom landing at its minimum legal size unless a caller really
        # needs the full stack footprint.  This matters on sky130: glayout met2 is
        # physical M1, and a met2->met5 stack is 1.50 um wide because of an
        # intermediate-metal minimum-area rule, while its legal M1 landing is only
        # 0.43 um wide.  Expanding M1 to the full stack at the TG drain overlaps the
        # VIN connector and shorts VIN to VOUT.
        v = via_stack(
            pdk,
            layer1,
            layer2,
            fulltop=True,
            fullbottom=fullbottom,
        )
        v_ref = top_level << v
        v_ref.movex(port.center[0] - v_ref.center[0])
        v_ref.movey(port.center[1] - v_ref.center[1])
        return v_ref

    # The MIM cap top/bottom plate metals are PDK-dependent (both gf180 and sky130
    # use met5/met4 top/bottom, but other PDKs may differ). Derive them from the
    # cap's actual ports so this cell is PDK-agnostic rather than hardcoding a stack.
    mim_top_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_top_met_E"].layer)
    mim_bottom_layer = _mimcap_glayers(pdk)[0]

    if with_reset:
        rst_drain_port = rst_ref.ports["multiplier_0_drain_E"].copy()
        mim_top_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_top_met_W"].layer)
        mim_layer_top_w = mim_ref.ports["row0_col0_top_met_W"].copy()
        rst_vout_via = _drop_via_beside(
            top_level, pdk, rst_drain_port, mim_top_layer, top_level
        )
        rst_drain_dest = rst_vout_via.ports["top_met_E"].copy()
        rst_drain_dest.center = (mim_layer_top_w.center[0], rst_drain_dest.center[1])
        top_level << straight_route(
            pdk,
            rst_vout_via.ports["top_met_E"],
            rst_drain_dest,
            glayer1=mim_top_layer,
            glayer2=mim_top_layer,
            width=max(
                pdk.get_grule(mim_top_layer)["min_width"],
                rst_vout_via.ports["top_met_E"].width,
            ),
        )

    # NOTE: the bottom-plate exit is built BEFORE the VOUT via on purpose. It is a
    # via STACK sitting in the gap east of the plate, and sky130's wide-metal rule
    # (m3.3ab, 0.4 um because the plate makes its met3 "huge") makes it a hard
    # blocker for the VOUT stack. Placing it first lets _drop_via_beside see it.
    # VSS (V2 = bottom plate): use the rightmost unit in row zero for the shortest
    # multi-column connection.  SKY130 needs its physical-M3 plate to remain
    # rectangular as it exits the MIM: a narrow met4 wire breaks capm.11's
    # surround exemption, while putting the via inside the marker violates
    # capm.8.  Other PDKs can use the smaller direct bottom-metal route.
    rightmost_col = cap_cols - 1
    plate_e = _bottom_plate_east_port(pdk, mim_comp, mim_ref, mim_bottom_layer)
    tie_w = tg_ref.ports["N_tie_N_top_met_W"]
    if pdk.get_grule("capmet").get("requires_full_height_bottom_plate_exit"):
        vss_drop_site = plate_e.copy()
        vss_drop_site.center = (plate_e.center[0], tie_w.center[1])
        vss_plate_via = drop_via("met2", mim_bottom_layer, vss_drop_site)
        vss_plate_via.movex(
            plate_e.center[0] - vss_plate_via.ports["top_met_W"].center[0]
        )
        top_level << straight_route(
            pdk,
            plate_e,
            vss_plate_via.ports["top_met_E"],
            glayer1=mim_bottom_layer,
            glayer2=mim_bottom_layer,
            width=plate_e.width,
        )
        top_level << straight_route(
            pdk,
            tie_w,
            vss_plate_via.ports["bottom_met_E"],
            glayer1="met2",
            glayer2="met2",
        )
    else:
        vss_dst = tie_w.copy()
        vss_dst.center = (plate_e.center[0], tie_w.center[1])
        top_level << straight_route(
            pdk,
            tie_w,
            vss_dst,
            glayer1=mim_bottom_layer,
            glayer2=mim_bottom_layer,
            # straight_route inherits tie_w's width, sized for the met1 tie bar.
            # The route is drawn on the cap's bottom-plate metal, whose minimum
            # width can be larger (ihp130: 0.16 um tie on 0.20 um Metal4).
            width=max(
                float(tie_w.width),
                float(pdk.get_grule(mim_bottom_layer)["min_width"]),
            ),
        )

    # TG VOUT (V1 = top plate). Outside the reset block so VOUT always connects to the
    # hold cap, with or without the reset switch. Single met2->met5 via at the TG drain,
    # then a straight met5 run west onto the top plate at the drain's y -- mirrors the
    # VSS routing but climbs to the TOP plate. The via sits at the drain, EAST of the
    # cap and off its footprint, so the met2->met5 stack never punches through the met4
    # bottom plate (which would short VOUT to VSS).
    #
    # Use the HIGHEST PMOS multiplier index: multiplier_0 is the topmost drain (furthest
    # from the TG center), and the cap plate is centered on the TG, so the highest index
    # (the bottommost drain, nearest center) is the one that lands within the plate
    # y-extent for multipliers > 1. switch_multipliers is (NMOS, PMOS); the drain is PMOS.
    pmos_mult = int(switch_multipliers[1])
    tg_vout_port = tg_ref.ports[f"P_multiplier_{pmos_mult - 1}_drain_W"].copy()

    # The straight met5 run lands on the plate only if the drain's y is within the
    # top-plate y-extent. For tall switches with a short cap (e.g. asymmetric
    # multipliers + a small cap) even the bottommost PMOS drain can sit above the
    # plate, which would silently leave VOUT open. Fail loudly instead.
    # This happens around (4, 4) caps for gf180 
    tp_s = mim_ref.ports["row0_col0_top_met_S"].center[1]
    tp_n = mim_ref.ports["row0_col0_top_met_N"].center[1]
    if not (tp_s <= tg_vout_port.center[1] <= tp_n):
        raise NotImplementedError(
            f"TG VOUT drain (y={tg_vout_port.center[1]:.2f}) falls outside the cap "
            f"top-plate y-extent [{tp_s:.2f}, {tp_n:.2f}] for "
            f"switch_multipliers={switch_multipliers}, cap_size={cap_size}: the "
            f"straight met5 route would leave VOUT open. Increase the cap height "
            f"(cap_size[1]) or rebalance the switch multipliers."
        )

    tg_vout_via = _drop_via_beside(
        top_level, pdk, tg_vout_port, mim_top_layer, top_level
    )
    vout_inset = pdk.get_grule(mim_top_layer)["min_separation"]
    vout_met5_dst = tg_vout_via.ports["top_met_W"].copy()
    vout_met5_dst.center = (
        mim_ref.ports["row0_col0_top_met_E"].center[0] - vout_inset,
        vout_met5_dst.center[1],
    )
    top_level << straight_route(
        pdk,
        tg_vout_via.ports["top_met_W"],
        vout_met5_dst,
        glayer1=mim_top_layer,
        glayer2=mim_top_layer,
    )

    # VOUT_TAP on met3, one layer up from the via's met2 bottom, to keep the routable
    # tap off the TG's congested met2 SD layer. Anchor the port at the via CENTER, which
    # lies on every layer of the stack -- on sky130 the intermediate metals are inset from
    # the met2 edge, so a met3 port at the via's north face would float off the metal.
    vout_tap_port = tg_vout_via.ports["bottom_met_N"].copy()
    vout_tap_port.layer = pdk.get_glayer("met3")
    vout_tap_port.center = tg_vout_via.center

    # CLK (NMOS gate) and CLK_B (PMOS gate) are both gate-E ports at the SAME x (the
    # NMOS sits below the PMOS). Offset CLK_B WEST along its gate strap so the two clock
    # pins occupy separate x-columns, which lets a shared vertical CLK / CLK_B bus run
    # cleanly when these cells are tiled. West keeps the pin on the existing gate metal
    # (the strap already spans west of the port), clear of the source/VIN rail to the east.
    CLK_B_OFFSET = -1.0
    clk_b_src = tg_ref.ports["P_multiplier_0_gate_E"]
    clk_b_port = clk_b_src.copy()
    clk_b_port.center = (clk_b_src.center[0] + CLK_B_OFFSET, clk_b_src.center[1])
    # This is the WEST end of the new route, so make it face west.  Keeping the
    # copied east-facing orientation made expose_ports() inset the CLK_B label
    # outside the metal on SKY130, leaving the PMOS gate disconnected in extraction.
    clk_b_port = set_port_orientation(clk_b_port, 180)
    top_level << straight_route(
        pdk, clk_b_src, clk_b_port, glayer1="met2", glayer2="met2"
    )

    # VSS/VCC come off the guard-ring ties, which are single-layer met1 -- a congested
    # layer (the FETs' own S/D routing is met1), so a downstream consumer landing on the
    # met1 tie collides/shorts with the device routing. The tie y-bands are clear on met2,
    # so via each tie up to met2 and expose the supply pins there, off met1. The via is
    # centred on the tie BAR (via its E-port y), not the north/south edge port, so its
    # landing pad sits on the tie metal.
    def _tie_up(edge_name, xref_name):
        bar = tg_ref.ports[xref_name].copy()
        bar.center = (
            tg_ref.ports[xref_name].center[0],
            tg_ref.ports[edge_name].center[1],
        )
        return drop_via("met1", "met2", bar)

    vss_via = _tie_up("N_tie_S_top_met_E", "N_tie_S_top_met_N")  # NMOS south tie -> VSS
    vcc_via = _tie_up("P_tie_S_top_met_E", "P_tie_S_top_met_S")  # PMOS south tie -> VCC

    # Expose named ports + labels (add_port + inward-offset label) via the shared helper.
    port_labels = [
        # VIN taps the TG's source connector (the met2 trunk between the two FETs) rather
        # than a per-FET source rail -- a cleaner, mid-cell landing for downstream routing.
        ("VIN", tg_ref.ports["source_con_E"]),
        ("CLK", tg_ref.ports["N_multiplier_0_gate_E"]),
        ("CLK_B", clk_b_port),
        ("VCC", vcc_via.ports["top_met_N"]),
        ("VSS", vss_via.ports["top_met_S"]),
        ("VOUT", mim_ref.ports["row0_col0_top_met_W"]),
        ("VOUT_TAP", vout_tap_port),
    ]
    if with_reset:
        port_labels += [
            ("RESET", rst_ref.ports["multiplier_0_gate_W"]),
            ("VZERO", rst_ref.ports["multiplier_0_source_W"]),
        ]
    # VOUT_TAP gets a gdsfactory port but NO text: it is a routing affordance on the
    # VOUT net, not a circuit node, and a second label on that net competes with
    # VOUT's own pin during extraction (magic named the port VOUT_TAP and netgen
    # then reported VOUT as a mismatch).
    labelled = [(n, p) for n, p in port_labels if n != "VOUT_TAP"]
    expose_ports(top_level, pdk, labelled)
    top_level.add_port(name="VOUT_TAP", port=vout_tap_port)
    # expose_ports only drops text on the DRAWING layer, which names a net but does
    # not create a port: magic then extracts ".subckt sample_hold_cell" with an empty
    # port list and netgen fails pin matching even though the netlists match uniquely.
    # add_pin_labels draws the rectangle on <glayer>_pin and the text on
    # <glayer>_label, which is what magic turns into an actual port.
    # VOUT_TAP is deliberately excluded: it is a routing affordance on the VOUT net,
    # not a circuit node, so the source netlist has no counterpart for it. Giving it a
    # pin would put two ports on one net and netgen reports "(no matching pin)". It
    # stays a gdsfactory port for consumers either way.
    # VOUT's PIN goes at the met3 tap, not on the cap top plate. A pin rectangle is
    # metal to the DRC deck, and any sub-3um appendage on a >=3um plate trips the
    # wide-metal self-junction rules (sky130 m4.5ab). The tap is the same net, so
    # magic gets its port without adding a junction to the plate.
    pin_specs = [(n, vout_tap_port if n == "VOUT" else p) for n, p in labelled]
    add_pin_labels(top_level, pdk, pin_specs, flatten=False)

    top_level.info["netlist"] = _sample_hold_netlist(tg_comp, mim_comp, rst_comp)

    return top_level


if __name__ == "__main__":
    from glayout.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    for pdk_name, pdk in [("gf180", gf180_mapped_pdk), ("sky130", sky130_mapped_pdk)]:
        print(f"Generating sample_hold_cell ({pdk_name})...")
        comp = sample_hold_cell(pdk, switch_multipliers=(2, 2), cap_size=(4, 4))
        out = f"sample_hold_cell_{pdk_name}.gds"
        comp.write_gds(out)
        print(f"  wrote {out} ({comp.name})")
