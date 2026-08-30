from glayout import MappedPDK, sky130,gf180
from glayout.backend import Component, cell, rectangle
from glayout.primitives.fet import nmos, pmos, multiplier
from glayout.util.comp_utils import evaluate_bbox, prec_center, align_comp_to_port, movex, movey
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.routing.straight_route import straight_route
from glayout.routing.c_route import c_route
from glayout.routing.L_route import L_route
from glayout.primitives.guardring import tapring
from glayout.util.port_utils import add_ports_perimeter
from glayout.spice.netlist import Netlist
from glayout.primitives.via_gen import via_stack
try:
    from glayout.verification.evaluator_wrapper import run_evaluation
except ImportError:
    print("Warning: evaluator_wrapper not found. Evaluation will be skipped.")
    run_evaluation = None

def add_tg_labels(tg_in: Component,
                        pdk: MappedPDK
                        ) -> Component:
	
    tg_in.unlock()
    
    # list that will contain all port/comp info
    move_info = list()
    # create labels and append to info list
    # vin
    vinlabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.27,0.27),centered=True).copy()
    vinlabel.add_label(text="VIN",layer=pdk.get_glayer("met2_label"))
    move_info.append((vinlabel,tg_in.ports["N_multiplier_0_source_E"],None))
    
    # vout
    voutlabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.27,0.27),centered=True).copy()
    voutlabel.add_label(text="VOUT",layer=pdk.get_glayer("met2_label"))
    move_info.append((voutlabel,tg_in.ports["P_multiplier_0_drain_W"],None))
    
    # vcc
    vcclabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.5,0.5),centered=True).copy()
    vcclabel.add_label(text="VCC",layer=pdk.get_glayer("met2_label"))
    move_info.append((vcclabel,tg_in.ports["P_tie_S_top_met_S"],None))
    
    # vss
    vsslabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.5,0.5),centered=True).copy()
    vsslabel.add_label(text="VSS",layer=pdk.get_glayer("met2_label"))
    move_info.append((vsslabel,tg_in.ports["N_tie_S_top_met_N"], None))
    
    # VGP
    vgplabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.27,0.27),centered=True).copy()
    vgplabel.add_label(text="VGP",layer=pdk.get_glayer("met2_label"))
    move_info.append((vgplabel,tg_in.ports["P_multiplier_0_gate_E"], None))
    
    # VGN
    vgnlabel = rectangle(layer=pdk.get_glayer("met2_pin"),size=(0.27,0.27),centered=True).copy()
    vgnlabel.add_label(text="VGN",layer=pdk.get_glayer("met2_label"))
    move_info.append((vgnlabel,tg_in.ports["N_multiplier_0_gate_E"], None))

    # move everything to position
    for comp, prt, alignment in move_info:
        alignment = ('c','b') if alignment is None else alignment
        compref = align_comp_to_port(comp, prt, alignment=alignment)
        tg_in.add(compref)
    return tg_in.flatten() 


def get_component_netlist(component):
    """Helper function to get netlist object from component info, compatible with all gdsfactory versions"""
    from glayout.spice.netlist import Netlist
    
    # Try to get stored object first (for older gdsfactory versions)
    if 'netlist_obj' in component.info:
        return component.info['netlist_obj']
    
    # Try to reconstruct from netlist_data (for newer gdsfactory versions)
    if 'netlist_data' in component.info:
        data = component.info['netlist_data']
        netlist = Netlist(
            circuit_name=data['circuit_name'],
            nodes=data['nodes']
        )
        netlist.source_netlist = data['source_netlist']
        return netlist
    
    # Fallback: return the string representation (should not happen in normal operation)
    return component.info.get('netlist', '')

def sky130_add_tg_labels(tg_in: Component) -> Component:
	
    tg_in.unlock()
    
    # define layers`
    met1_pin = (68,16)
    met1_label = (68,5)
    li1_pin = (67,16)
    li1_label = (67,5)
    # list that will contain all port/comp info
    move_info = list()
    # create labels and append to info list
    # vin
    vinlabel = rectangle(layer=met1_pin,size=(0.27,0.27),centered=True).copy()
    vinlabel.add_label(text="VIN",layer=met1_label)
    move_info.append((vinlabel,tg_in.ports["N_multiplier_0_source_E"],None))
    
    # vout
    voutlabel = rectangle(layer=met1_pin,size=(0.27,0.27),centered=True).copy()
    voutlabel.add_label(text="VOUT",layer=met1_label)
    move_info.append((voutlabel,tg_in.ports["P_multiplier_0_drain_W"],None))
    
    # vcc
    vcclabel = rectangle(layer=met1_pin,size=(0.5,0.5),centered=True).copy()
    vcclabel.add_label(text="VCC",layer=met1_label)
    move_info.append((vcclabel,tg_in.ports["P_tie_S_top_met_S"],None))
    
    # vss
    vsslabel = rectangle(layer=met1_pin,size=(0.5,0.5),centered=True).copy()
    vsslabel.add_label(text="VSS",layer=met1_label)
    move_info.append((vsslabel,tg_in.ports["N_tie_S_top_met_N"], None))
    
    # VGP
    vgplabel = rectangle(layer=met1_pin,size=(0.27,0.27),centered=True).copy()
    vgplabel.add_label(text="VGP",layer=met1_label)
    move_info.append((vgplabel,tg_in.ports["P_multiplier_0_gate_E"], None))
    
    # VGN
    vgnlabel = rectangle(layer=met1_pin,size=(0.27,0.27),centered=True).copy()
    vgnlabel.add_label(text="VGN",layer=met1_label)
    move_info.append((vgnlabel,tg_in.ports["N_multiplier_0_gate_E"], None))

    # move everything to position
    for comp, prt, alignment in move_info:
        alignment = ('c','b') if alignment is None else alignment
        compref = align_comp_to_port(comp, prt, alignment=alignment)
        tg_in.add(compref)
    return tg_in.flatten() 


def tg_netlist(nfet: Component, pfet: Component) -> Netlist:

         netlist = Netlist(circuit_name='Transmission_Gate', nodes=['VIN', 'VSS', 'VOUT', 'VCC', 'VGP', 'VGN'])
         # Use helper function to get netlist objects regardless of gdsfactory version.
         # Each fet's dummies physically tie to the fet's own welltie ring (NMOS
         # bulk = VSS, PMOS bulk = VCC), so DUM gets mapped to the same bulk net
         # so the schematic matches the layout extraction.
         nfet_netlist = get_component_netlist(nfet)
         pfet_netlist = get_component_netlist(pfet)
         netlist.connect_netlist(nfet_netlist, [('D', 'VOUT'), ('G', 'VGN'), ('S', 'VIN'), ('B', 'VSS'), ('DUM', 'VSS')])
         netlist.connect_netlist(pfet_netlist, [('D', 'VOUT'), ('G', 'VGP'), ('S', 'VIN'), ('B', 'VCC'), ('DUM', 'VCC')])

         return netlist



@cell
def  transmission_gate(
        pdk: MappedPDK,
        width: tuple[float,float] = (1,1),
        length: tuple[float,float] = (None,None),
        fingers: tuple[int,int] = (1,1),
        multipliers: tuple[int,int] = (1,1),
        substrate_tap: bool = False,
        tie_layers: tuple[str,str] = ("met2","met1"),
        **kwargs
        ) -> Component:
    """
    creates a transmission gate
    tuples are in (NMOS,PMOS) order
    **kwargs are any kwarg that is supported by nmos and pmos
    """
   
    #top level component
    top_level = Component()

    #two fets
    nfet = nmos(pdk, width=width[0], fingers=fingers[0], multipliers=multipliers[0], with_dummy=True, with_dnwell=False,  with_substrate_tap=False, length=length[0], tie_layers=tie_layers, **kwargs)
    pfet = pmos(pdk, width=width[1], fingers=fingers[1], multipliers=multipliers[1], with_dummy=True, with_substrate_tap=False, length=length[1], tie_layers=tie_layers, **kwargs)
    nfet_ref = top_level << nfet
    pfet_ref = top_level << pfet 
    pfet_ref = rename_ports_by_orientation(pfet_ref.mirror_y())

    #Relative move
    pfet_ref.movey(nfet_ref.ymax + evaluate_bbox(pfet_ref)[1]/2 + pdk.util_max_metal_seperation())
    
    #Routing
    # The inter-FET links short the two devices in PARALLEL: source<->source is VIN,
    # drain<->drain is VOUT. Both are required -- the netlist declares both, and a
    # consumer that taps VOUT at the pfet drain would leave the nfet drain floating
    # (half the switch dead) if the drain link were dropped.
    #
    # Which shape those links take depends on the guard rings, hence tie_layers:
    _mn, _mp = multipliers[0] - 1, multipliers[1] - 1
    if tie_layers[0] != "met2":
        # Single-layer rings (e.g. tie_layers=("met1","met1")): met2 crosses the ring
        # N/S segments directly, so the link can stay on met2 with NO vias and no met3
        # trunk. Shape is a same-layer "C": a stub off each port plus a vertical bar
        # OFFSET past the rail ends -- on each flank the source AND drain rails end at
        # the SAME x, so a flush bar would skewer the other net's rail end. (That
        # clearance is the same thing c_route's `extension` buys on the met2-ring path.)
        def _rect(x0, y0, x1, y1):
            _r = top_level << rectangle(
                size=(round(x1 - x0, 3), round(y1 - y0, 3)),
                layer=pdk.get_glayer("met2"),
                centered=True,
            )
            _r.movex((x0 + x1) / 2 - _r.center[0]).movey((y0 + y1) / 2 - _r.center[1])

        def _cbar(p1, p2, east, name):
            sgn = 1 if east else -1
            ext, bw = 0.5, 0.4
            x_edge = (max(sgn * p1.center[0], sgn * p2.center[0]) * sgn)  # outermost rail end
            xb = sorted((x_edge + sgn * ext, x_edge + sgn * (ext + bw)))
            ylo, yhi = 1e9, -1e9
            for p in (p1, p2):
                h = min(float(p.width), 0.7)
                xs = sorted((p.center[0] - sgn * 0.2, x_edge + sgn * (ext + bw)))
                _rect(xs[0], p.center[1] - h / 2, xs[1], p.center[1] + h / 2)
                ylo = min(ylo, p.center[1] - h / 2)
                yhi = max(yhi, p.center[1] + h / 2)
            _rect(xb[0], ylo, xb[1], yhi)
            # Port at the trunk centre: the natural place for a consumer to land this
            # net (VIN=source, VOUT=drain) -- a met2 landing between the two FETs and
            # off the per-FET rails, which are crowded by the multiplier c_routes.
            top_level.add_port(
                name=name,
                center=((xb[0] + xb[1]) / 2, (ylo + yhi) / 2),
                width=bw,
                orientation=0 if east else 180,
                layer=pdk.get_glayer("met2"),
            )

        # trailing "_x" is a placeholder segment: rename_ports_by_orientation replaces
        # the LAST underscore-segment with the orientation letter, giving the public
        # names "source_con_E" and "drain_con_W".
        _cbar(nfet_ref.ports[f"multiplier_{_mn}_source_E"],
              pfet_ref.ports[f"multiplier_{_mp}_source_E"], east=True, name="source_con_x")
        _cbar(nfet_ref.ports[f"multiplier_{_mn}_drain_W"],
              pfet_ref.ports[f"multiplier_{_mp}_drain_W"], east=False, name="drain_con_x")
    else:
        # met2-ring path: met3 trunks hop the rings. With stacked multipliers the
        # per-FET gate trunk crowds the East inter-FET source trunk on met3
        # (sky130 m3.2 / gf180 M3.2a), so push the source c-route further East.
        src_extension = 1.0 if max(multipliers) > 1 else 0.5
        top_level << c_route(pdk, nfet_ref.ports["multiplier_0_source_E"], pfet_ref.ports["multiplier_0_source_E"], extension=src_extension)
        top_level << c_route(pdk, nfet_ref.ports["multiplier_0_drain_W"], pfet_ref.ports["multiplier_0_drain_W"], viaoffset=False)
    
    #Renaming Ports
    top_level.add_ports(nfet_ref.get_ports_list(), prefix="N_")
    top_level.add_ports(pfet_ref.get_ports_list(), prefix="P_")

    #substrate tap
    if substrate_tap:
            substrate_tap_encloses =((evaluate_bbox(top_level)[0]+pdk.util_max_metal_seperation()), (evaluate_bbox(top_level)[1]+pdk.util_max_metal_seperation()))
            guardring_ref = top_level << tapring(
            pdk,
            enclosed_rectangle=substrate_tap_encloses,
            sdlayer="p+s/d",
            horizontal_glayer='met2',
            vertical_glayer='met1',
        )
            guardring_ref.move(nfet_ref.center).movey(evaluate_bbox(pfet_ref)[1]/2 + pdk.util_max_metal_seperation()/2)
            top_level.add_ports(guardring_ref.get_ports_list(),prefix="tap_")
    
    component = component_snap_to_grid(rename_ports_by_orientation(top_level))
    # Store netlist as string to avoid gymnasium info dict type restrictions
    # Compatible with both gdsfactory 7.7.0 and 7.16.0+ strict Pydantic validation
    netlist_obj = tg_netlist(nfet, pfet)
    component.info['netlist'] = netlist_obj.generate_netlist()
    # Store the Netlist object for hierarchical netlist building
    component.info['netlist_obj'] = netlist_obj
    # Store serialized netlist data for reconstruction if needed
    component.info['netlist_data'] = {
        'circuit_name': netlist_obj.circuit_name,
        'nodes': netlist_obj.nodes,
        'source_netlist': netlist_obj.source_netlist
    }

    # gf180 LVS uses klayout's official deck which strictly requires named
    # pin labels on met*_label layers. sky130 magic+netgen tolerates missing
    # labels, so we only stamp them for gf180. Composite cells suppress with
    # GLAYOUT_NO_PIN_LABELS so inner labels don't leak into the parent's GDS.
    import os
    if pdk.name.lower() == "gf180" and not os.environ.get("GLAYOUT_NO_PIN_LABELS"):
        try:
            component = add_tg_labels(component, pdk)
        except KeyError:
            pass

    return component
if __name__ == "__main__":
    # OLD EVAL CODE
    # comp = transmission_gate(sky130)
    # # comp.pprint_ports()
    # comp = add_tg_labels(comp,sky130)
    # comp.name = "TG"
    # comp.show()
    # #print(comp.info['netlist'].generate_netlist())
    # print("...Running DRC...")
    # drc_result = sky130.drc_magic(comp, "TG")
    # ## Klayout DRC
    # #drc_result = gf180.drc(comp)\n
    
    # time.sleep(5)
        
    # print("...Running LVS...")
    # lvs_res=sky130.lvs_netgen(comp, "TG")
    # #print("...Saving GDS...")
    # #comp.write_gds('out_TG.gds')

    # NEW EVAL CODE
    #transmission_gate = transmission_gate(sky130_mapped_pdk)
    transmission_gate = add_tg_labels(transmission_gate(sky130),sky130)
    transmission_gate.show()
    transmission_gate.name = "Transmission_Gate"
    #magic_drc_result = sky130_mapped_pdk.drc_magic(transmission_gate, transmission_gate.name)
    #netgen_lvs_result = sky130_mapped_pdk.lvs_netgen(transmission_gate, transmission_gate.name)
    transmission_gate_gds = transmission_gate.write_gds("transmission_gate.gds")
    res = run_evaluation("transmission_gate.gds", transmission_gate.name, transmission_gate)
