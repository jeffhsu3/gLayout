from glayout.backend import Component, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.cells.composite.opamp.opamp import opamp
from glayout.primitives.mimcap import mimcap_array
from glayout.util.comp_utils import prec_ref_center, evaluate_bbox
from glayout.routing.c_route import c_route
from glayout.routing.straight_route import straight_route
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.spice import Netlist
from glayout.util.snap_to_grid import component_snap_to_grid

def leaky_integrator_netlist(opamp_ref, mimcap_ref) -> Netlist:
    netlist = Netlist(circuit_name="leaky_integrator", nodes=['VDD', 'GND', 'VIN_P', 'VIN_N', 'VOUT', 'IBIAS'])
    
    netlist.connect_netlist(
        opamp_ref.info['netlist'],
        [('vdd', 'VDD'), ('gnd', 'GND'), ('plus', 'VIN_P'), ('minus', 'VIN_N'), ('output', 'VOUT'), ('outputibias', 'IBIAS')]
    )
    netlist.connect_netlist(
        mimcap_ref.info['netlist'],
        [('V1', 'VOUT'), ('V2', 'GND')]
    )
    return netlist

@cell
def leaky_integrator(
    pdk: MappedPDK,
    mim_cap_size: tuple[float, float] = (10.0, 10.0),
    mim_cap_rows: int = 2,
    mim_cap_cols: int = 2,
    rmult: int = 2
) -> Component:
    top_level = Component("leaky_integrator")
    
    # Instantiate OPAMP. The integrator drives its output through the cap, so
    # it needs the buffered output stage: that is also what puts the wrapper's
    # lowercase vdd/gnd/plus/minus/output/outputibias nodes on the netlist that
    # leaky_integrator_netlist connects to (add_output_stage=False would expose
    # the bare two-stage's uppercase ports instead).
    amp = opamp(pdk, add_output_stage=True)
    amp_ref = prec_ref_center(amp)
    top_level.add(amp_ref)
    
    # Instantiate MIMCAP ARRAY
    cap = mimcap_array(pdk, rows=mim_cap_rows, columns=mim_cap_cols, size=mim_cap_size)
    cap_ref = prec_ref_center(cap)
    
    # Place cap array above the opamp
    cap_ref.movey(amp_ref.ymax + evaluate_bbox(cap)[1]/2 + 10)
    top_level.add(cap_ref)
    
    # Route OPAMP output to the top plate of the MIMCAP array
    out_port = amp_ref.ports.get("pin_output_e2")
    if not out_port:
        out_port = amp_ref.ports.get("outputstage_amp_multiplier_0_source_E")

    cap_top_port = cap_ref.ports.get("row0_col0_top_met_S")
    if not cap_top_port:
        cap_top_port = cap_ref.ports.get("row0_col0_top_met_W")
    
    from glayout.routing.smart_route import smart_route
    if out_port and cap_top_port:
        top_level << smart_route(pdk, out_port, cap_top_port)
    
    # Ground the bottom plate of the MIMCAP array.
    gnd_port = amp_ref.ports.get("pin_gnd_E")
    cap_bot_port = cap_ref.ports.get("row0_col0_bottom_met_W")
    
    if gnd_port and cap_bot_port:
        top_level << smart_route(pdk, gnd_port, cap_bot_port)
        
    top_level.add_ports(amp_ref.get_ports_list(), prefix="amp_")
    top_level.add_ports(cap_ref.get_ports_list(), prefix="cap_")
    
    top_level.info['netlist'] = leaky_integrator_netlist(amp_ref, cap_ref)
    
    return component_snap_to_grid(rename_ports_by_orientation(top_level))
