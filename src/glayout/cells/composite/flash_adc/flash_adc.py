from glayout.backend import Component, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.cells.composite.comparator.comparator import comparator
from glayout.primitives.resistor import resistor
from glayout.util.comp_utils import prec_ref_center, evaluate_bbox, prec_array
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.spice import Netlist

def flash_adc_netlist(comp_netlist: Netlist, num_comparators: int) -> Netlist:
    netlist = Netlist(circuit_name="flash_adc", nodes=['VDD', 'GND', 'VIN', 'VREF_TOP', 'VREF_BOT', 'IBIAS1', 'IBIAS2'] + [f'VOUT_{i}' for i in range(num_comparators)])
    
    # Manually defining the resistor string netlist to avoid missing info
    for i in range(num_comparators + 1):
        node_top = 'VREF_TOP' if i == num_comparators else f'VREF_NODE_{i}'
        node_bot = 'VREF_BOT' if i == 0 else f'VREF_NODE_{i-1}'
        netlist.source_netlist += f"\nXR{i} {node_top} {node_bot} RES_MACRO"
        
    for i in range(num_comparators):
        netlist.connect_netlist(
            comp_netlist,
            [('VDD', 'VDD'), ('GND', 'GND'), ('VIN_P', 'VIN'), ('VIN_N', f'VREF_NODE_{i}'), ('VOUT', f'VOUT_{i}'), ('IBIAS1', 'IBIAS1'), ('IBIAS2', 'IBIAS2')]
        )
    return netlist

@cell
def flash_adc(
    pdk: MappedPDK,
    num_comparators: int = 7,
) -> Component:
    top_level = Component("flash_adc")
    
    comp_single = comparator(pdk)
    
    comp_space = 20.0
    comp_array = prec_array(comp_single, rows=num_comparators, columns=1, spacing=[comp_space, comp_space])
    comp_array_ref = prec_ref_center(comp_array)
    top_level.add(comp_array_ref)
    top_level.add_ports(comp_array_ref.get_ports_list(), prefix="comp_array_")
    
    res = resistor(pdk, num_series=num_comparators + 1)
    res_ref = prec_ref_center(res)
    res_ref.movex(comp_array_ref.xmax + evaluate_bbox(res)[0]/2 + 30)
    top_level.add(res_ref)
    
    top_level.add_ports(res_ref.get_ports_list(), prefix="res_")
    
    top_level.info['netlist'] = flash_adc_netlist(comp_single.info['netlist'], num_comparators)
    
    return component_snap_to_grid(rename_ports_by_orientation(top_level))
