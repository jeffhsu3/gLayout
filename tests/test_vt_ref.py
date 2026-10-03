from __future__ import annotations

import importlib
import tempfile
from pathlib import Path

import gdstk
import pytest

from glayout.cells.elementary.vt_ref import vt_ref, vt_ref_netlist
from glayout.pdk.gf180_mapped import gf180_mapped_pdk
from glayout.pdk.sky130_mapped import sky130_mapped_pdk
from glayout.primitives.vpp import (
    SKY130_VPP_0P4_F,
    SKY130_VPP_FINGER_11_MODEL,
    SKY130_VPP_FINGER_41_MODEL,
    sky130_vpp_cap_0p4,
)


def test_vt_ref_canonical_and_compatibility_imports() -> None:
    canonical = importlib.import_module("glayout.cells.elementary.vt_ref.vt_ref")
    compatibility = importlib.import_module("glayout.blocks.elementary.vt_ref.vt_ref")

    assert compatibility.__file__ == canonical.__file__
    assert callable(canonical.vt_ref)
    assert callable(compatibility.vt_ref)


def test_vt_ref_uses_paper_topology_and_models() -> None:
    netlist = vt_ref_netlist(sky130_mapped_pdk).generate_netlist()

    assert ".subckt VT_REF VDD VREF VSS" in netlist
    assert "XM1 VDD VSS VREF VSS sky130_fd_pr__nfet_05v0_nvt l=60 w=3.3" in netlist
    assert "XM2 VREF VREF VSS VSS sky130_fd_pr__nfet_g5v0d10v5 l=60 w=1.5" in netlist
    assert netlist.count(SKY130_VPP_FINGER_41_MODEL) == 4
    assert netlist.count(SKY130_VPP_FINGER_11_MODEL) == 1
    assert f"XCOUT0 VREF VSS VSS {SKY130_VPP_FINGER_41_MODEL}" in netlist


def test_vt_ref_can_omit_output_capacitor() -> None:
    netlist = vt_ref_netlist(sky130_mapped_pdk, with_cout=False).generate_netlist()

    assert "XCOUT" not in netlist
    component = vt_ref(sky130_mapped_pdk, with_cout=False)
    assert "cout_f" not in component.info


def test_sky130_vpp_array_is_nominally_0p4_pf() -> None:
    capacitor = sky130_vpp_cap_0p4(sky130_mapped_pdk)

    assert capacitor.info["unit_count"] == 5
    assert capacitor.info["capacitance_f"] == pytest.approx(SKY130_VPP_0P4_F)
    assert SKY130_VPP_0P4_F == pytest.approx(0.4e-12, rel=0.01)
    assert {"C0_W", "C0_E", "C1_W", "C1_E", "SUB"}.issubset(capacitor.ports)


def test_vt_ref_contains_matched_thick_oxide_markers() -> None:
    component = vt_ref(sky130_mapped_pdk)
    assert {"VDD", "VREF", "VSS"}.issubset(component.ports)

    with tempfile.TemporaryDirectory() as work:
        gds_path = Path(work) / "vt_ref.gds"
        component.write_gds(str(gds_path))
        top = gdstk.read_gds(str(gds_path)).top_level()[0]
        top.flatten()
        layer_specs = {(poly.layer, poly.datatype) for poly in top.polygons}

    assert sky130_mapped_pdk.get_glayer("hvi") in layer_specs
    assert sky130_mapped_pdk.get_glayer("hvntm") in layer_specs
    assert sky130_mapped_pdk.get_glayer("lvtn") in layer_specs
    assert (82, 64) in layer_specs  # SKY130 capacitor.drawing: VPP, not MIM.
    assert (81, 60) not in layer_specs  # LVID would select the wrong native FET.


@pytest.mark.parametrize("generator", [vt_ref, vt_ref_netlist])
def test_vt_ref_rejects_gf180(generator) -> None:
    with pytest.raises(NotImplementedError, match="GF180"):
        generator(gf180_mapped_pdk)


# Need some tests on the warnings for temp issues
# Need some warnings for highly speculuative
# THe parameterizations here are not working that well.   
# I am not sure any of this works well.  

# I think there ae some of the owrst values here even if we can't find them. 
# There are some of the worst values here even if we can't find them
