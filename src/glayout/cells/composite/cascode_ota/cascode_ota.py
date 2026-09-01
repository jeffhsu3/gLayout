"""cascode_ota — telescopic-cascode fully-differential OTA core (gf180 3.3V).

The amplifier core of the validated `fd_otac` (analog_ref/fd_ota_casc.spice): an NMOS
input pair on a tail current source, NMOS cascodes, PMOS cascodes, and PMOS loads. Gives
~57 dB open-loop diff gain (A≈675). Bias generator + switched-cap CMFB are added on top of
this core (separate, validated in analog_ref/fd_ota_silicon.spice) — this leaf exposes the
cascode gate rails (VBNC/VBPC), the load-gate control (PCM), the tail bias (VBN) and the
differential I/O so the bias-gen/CMFB wrap around it.

Two branches (Left controls OUTN, Right controls OUTP), each a vertical series stack:
  VSS - tail(VBN) - [NT] - input(INP/INN) - [A/B] - ncasc(VBNC) - [OUTN/OUTP]
      - pcasc(VBPC) - [C/D] - pload(PCM) - VDD
Device sizes mirror the schematic: input/ncasc w8 m4 L0.5; pcasc/pload w16 m2 L0.5;
tail w8 m2 L1 (the tail mirrors the 40uA bias; per-branch 20uA).

STATUS (2026-06-28): placement + signal routing (12 nets via smart_route) DONE and
DRC-CLEAN (magic, gf180mcuD: "No errors found"). The generated netlist topology is an
exact match to fd_otac (pins INP INN OUTP OUTN VBN VBNC VBPC PCM VDD VSS). LVS (gf180:
magic-extract + netgen, verification/run_cascode_ota_lvs.sh) NOT yet clean -- still TODO:
(1) route the supplies/bulk -- NMOS tie rings -> VSS, PMOS tie rings -> VDD, tail source
-> VSS, PMOS load sources -> VDD; (2) route the remaining gate pins INP/INN/VBN and bring
OUTP/OUTN/VBNC/VBPC/PCM out; (3) add pin LABELS so the extracted top has named ports.
Until then the extracted netlist merges the unrouted bulk into one well net (5 nets vs 15).
"""

from glayout.backend import Component, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.primitives.fet import nmos, pmos
from glayout.util.comp_utils import evaluate_bbox
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.routing.smart_route import smart_route
from glayout.spice.netlist import Netlist


def _cascode_ota_netlist(tail, inL, inR, ncL, ncR, pcL, pcR, plL, plR) -> Netlist:
    # pins exposed by the core; nt/a/b/c/d are internal series nodes
    nodes = ["INP", "INN", "OUTP", "OUTN", "VBN", "VBNC", "VBPC", "PCM", "VDD", "VSS"]
    net = Netlist(circuit_name="cascode_ota", nodes=nodes)
    net.connect_netlist(
        tail.info["netlist"], [("D", "NT"), ("G", "VBN"), ("S", "VSS"), ("B", "VSS")]
    )
    net.connect_netlist(
        inL.info["netlist"], [("D", "A"), ("G", "INP"), ("S", "NT"), ("B", "VSS")]
    )
    net.connect_netlist(
        inR.info["netlist"], [("D", "B"), ("G", "INN"), ("S", "NT"), ("B", "VSS")]
    )
    net.connect_netlist(
        ncL.info["netlist"], [("D", "OUTN"), ("G", "VBNC"), ("S", "A"), ("B", "VSS")]
    )
    net.connect_netlist(
        ncR.info["netlist"], [("D", "OUTP"), ("G", "VBNC"), ("S", "B"), ("B", "VSS")]
    )
    net.connect_netlist(
        pcL.info["netlist"], [("D", "OUTN"), ("G", "VBPC"), ("S", "C"), ("B", "VDD")]
    )
    net.connect_netlist(
        pcR.info["netlist"], [("D", "OUTP"), ("G", "VBPC"), ("S", "D"), ("B", "VDD")]
    )
    net.connect_netlist(
        plL.info["netlist"], [("D", "C"), ("G", "PCM"), ("S", "VDD"), ("B", "VDD")]
    )
    net.connect_netlist(
        plR.info["netlist"], [("D", "D"), ("G", "PCM"), ("S", "VDD"), ("B", "VDD")]
    )
    return net


@cell
def cascode_ota(
    pdk: MappedPDK,
    gap: float = 4.0,
) -> Component:
    pdk.activate()
    top = Component()

    # --- devices (sizes mirror fd_otac) ---
    nf = dict(with_dnwell=False, with_substrate_tap=False)
    tail = nmos(pdk, width=8, fingers=2, length=1, **nf)
    inL = nmos(pdk, width=8, fingers=4, length=0.5, **nf)
    inR = nmos(pdk, width=8, fingers=4, length=0.5, **nf)
    ncL = nmos(pdk, width=8, fingers=4, length=0.5, **nf)
    ncR = nmos(pdk, width=8, fingers=4, length=0.5, **nf)
    pcL = pmos(pdk, width=16, fingers=2, length=0.5, with_substrate_tap=False)
    pcR = pmos(pdk, width=16, fingers=2, length=0.5, with_substrate_tap=False)
    plL = pmos(pdk, width=16, fingers=2, length=0.5, with_substrate_tap=False)
    plR = pmos(pdk, width=16, fingers=2, length=0.5, with_substrate_tap=False)

    # --- floorplan: two columns (L at -dx, R at +dx), stacked rows ---
    wn = evaluate_bbox(inL)[0]
    wp = evaluate_bbox(pcL)[0]
    dx = max(wn, wp) / 2 + gap / 2
    hn = evaluate_bbox(inL)[1]
    hp = evaluate_bbox(pcL)[1]
    ht = evaluate_bbox(tail)[1]

    refs = {}
    # row y-centers, bottom -> top
    y_in = 0.0
    y_nc = y_in + hn / 2 + gap + hn / 2
    y_pc = y_nc + hn / 2 + gap + hp / 2
    y_pl = y_pc + hp / 2 + gap + hp / 2
    y_tail = y_in - hn / 2 - gap - ht / 2

    def place(name, comp, x, y):
        r = top << comp
        r.movex(x - r.center[0]).movey(y - r.center[1])
        refs[name] = r
        return r

    place("tail", tail, 0.0, y_tail)
    place("inL", inL, -dx, y_in)
    place("inR", inR, +dx, y_in)
    place("ncL", ncL, -dx, y_nc)
    place("ncR", ncR, +dx, y_nc)
    place("pcL", pcL, -dx, y_pc)
    place("pcR", pcR, +dx, y_pc)
    place("plL", plL, -dx, y_pl)
    place("plR", plR, +dx, y_pl)

    # --- routing ---
    routed, failed = [], []

    def route(net, a_ref, a_port, b_ref, b_port):
        try:
            top << smart_route(
                pdk,
                refs[a_ref].ports[a_port],
                refs[b_ref].ports[b_port],
                refs[a_ref],
                refs[b_ref],
            )
            routed.append(net)
        except Exception as e:  # noqa: BLE001 — collect, report, keep building
            failed.append((net, type(e).__name__))

    # series source/drain stacks (per branch, bottom->top)
    route("NT_L", "tail", "multiplier_0_drain_E", "inL", "multiplier_0_source_E")
    route("NT_R", "tail", "multiplier_0_drain_W", "inR", "multiplier_0_source_W")
    route("A", "inL", "multiplier_0_drain_N", "ncL", "multiplier_0_source_S")
    route("B", "inR", "multiplier_0_drain_N", "ncR", "multiplier_0_source_S")
    route("OUTN", "ncL", "multiplier_0_drain_N", "pcL", "multiplier_0_drain_S")
    route("OUTP", "ncR", "multiplier_0_drain_N", "pcR", "multiplier_0_drain_S")
    route("C", "pcL", "multiplier_0_source_N", "plL", "multiplier_0_drain_S")
    route("D", "pcR", "multiplier_0_source_N", "plR", "multiplier_0_drain_S")
    # shared gate rails (L<->R)
    route("VBNC", "ncL", "multiplier_0_gate_E", "ncR", "multiplier_0_gate_W")
    route("VBPC", "pcL", "multiplier_0_gate_E", "pcR", "multiplier_0_gate_W")
    route("PCM", "plL", "multiplier_0_gate_E", "plR", "multiplier_0_gate_W")
    # supplies: PMOS sources -> VDD rail; tail/nmos bulk ties -> VSS
    route("VDD", "plL", "multiplier_0_source_E", "plR", "multiplier_0_source_W")

    # expose all device ports (prefixed) for routing/debug
    for nm, r in refs.items():
        top.add_ports(r.get_ports_list(), prefix=f"{nm}_")
    top.info["routed"] = routed
    top.info["route_failed"] = failed

    comp = component_snap_to_grid(rename_ports_by_orientation(top))
    comp.info["netlist"] = _cascode_ota_netlist(
        tail, inL, inR, ncL, ncR, pcL, pcR, plL, plR
    )
    comp.info["routed"] = routed
    comp.info["route_failed"] = failed
    return comp


if __name__ == "__main__":
    from glayout.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk

    c = cascode_ota(gf180_mapped_pdk)
    c.write_gds("cascode_ota.gds")
    print("GDS:", c.name, "bbox=", c.bbox)
    print("routed nets:", c.info.get("routed"))
    print("FAILED routes:", c.info.get("route_failed"))
