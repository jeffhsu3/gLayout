from glayout.backend import Component, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.cells.composite.opamp.opamp import opamp
from glayout.util.comp_utils import prec_ref_center
from glayout.util.port_utils import rename_ports_by_orientation
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.spice import Netlist


def comparator_netlist(opamp_netlist: Netlist) -> Netlist:
    netlist = Netlist(circuit_name="comparator", nodes=['VDD', 'GND', 'VIN_P', 'VIN_N', 'VOUT', 'IBIAS1', 'IBIAS2'])
    netlist.connect_netlist(
        opamp_netlist,
        [('VDD', 'VDD'), ('GND', 'GND'), ('VP', 'VIN_P'), ('VN', 'VIN_N'), ('VOUT', 'VOUT'), ('CS_BIAS', 'IBIAS1'), ('DIFFPAIR_BIAS', 'IBIAS2')]
    )
    return netlist

@cell
def comparator(
    pdk: MappedPDK,
    **kwargs
) -> Component:
    """
    Push-button continuous-time comparator.
    It wraps the two-stage uncompensated opamp for high gain and rail-to-rail comparison.

    Note on the opamp's LVS labels.  This cell renames every opamp pin
    (VP->VIN_P, VN->VIN_N, CS_BIAS->IBIAS1, DIFFPAIR_BIAS->IBIAS2), so
    ``add_opamp_labels`` leaves labels here that name nets the netlist below
    calls something else -- netgen sees ``VP`` as a layout net with no
    schematic counterpart.  They are left on anyway.  Suppressing them
    (``GLAYOUT_NO_PIN_LABELS=1`` around the ``opamp()`` call, which the gate
    in opamp.py honours) was tried and changes no verdict: on the sky130 ZCD
    sizing netgen reports 16/16 devices, 13 layout nets against 11, and a
    failed pin match with the labels either on or off.  The mismatch is in
    the layout, not the labelling.
    """
    top_level = Component("comparator")
    kwargs["add_output_stage"] = False 
    comp = opamp(pdk, **kwargs)
    comp_ref = prec_ref_center(comp)
    top_level.add(comp_ref)
    top_level.add_ports(comp_ref.get_ports_list(), prefix="cmp_")
    top_level.info['netlist'] = comparator_netlist(comp.info['netlist'])
    
    return component_snap_to_grid(rename_ports_by_orientation(top_level))
