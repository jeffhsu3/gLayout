"""Brokaw bandgap core (gf180): build + reference-netlist structure.

DRC and LVS signoff run through the cell harnesses
(``tests/drc/run_cell_drc.py`` + ``tests/lvs/run_cell_lvs.py``), which need
the EDA tools; these tests cover everything tool-free: the generator builds
a non-empty layout with a GDS round-trip, and the reference netlist carries
the full Brokaw topology (mirror trio/pair, R1/R2 MOS-diode stacks, Q1 1x /
Q2 8x / Q3 1x plus two edge dummies).
"""
from __future__ import annotations

import importlib
import tempfile
from pathlib import Path

from glayout import gf180
from glayout.cells.composite.bandgap.bandgap import bandgap, bandgap_netlist


def test_bandgap_canonical_and_package_imports() -> None:
    canonical = importlib.import_module("glayout.cells.composite.bandgap.bandgap")
    package = importlib.import_module("glayout.cells.composite.bandgap")
    composite = importlib.import_module("glayout.cells.composite")

    assert package.bandgap is canonical.bandgap
    assert composite.bandgap is canonical.bandgap
    assert composite.bandgap_netlist is canonical.bandgap_netlist
    assert callable(canonical.bandgap)


def test_bandgap_netlist_topology() -> None:
    netlist = bandgap_netlist(gf180).generate_netlist()

    assert ".subckt BANDGAP VDD VREF B" in netlist
    # PMOS mirror trio + NMOS mirror pair (instance lines only)
    assert netlist.count(" BANDGAP_PFET l=") == 3
    assert netlist.count(" BANDGAP_NFET l=") == 2
    # Q1 (1x) + Q2 (8x) + Q3 (1x) + 2 edge dummies, C strapped to E
    assert netlist.count("npn_05p00x05p00") == 12
    assert "X7 VE1 B VE1 B npn_05p00x05p00" in netlist
    # R1 single diode VE2->VR1T, R2 nine-stack VE3->VREF
    assert "X5 VE2 VR1T VDD PMOS_RES" in netlist
    assert "X6 VE3 VREF VDD PMOS_RES_1" in netlist
    assert netlist.count(" DUM PMOS_UNIT_") == 10  # 1 (R1) + 9 (R2)


def test_bandgap_builds_and_writes_gds() -> None:
    gf180.activate()
    component = bandgap(gf180)
    (x0, y0), (x1, y1) = component.bbox
    assert x1 > x0 and y1 > y0
    assert "netlist" in component.info

    with tempfile.TemporaryDirectory() as td:
        gds_path = Path(td) / "bandgap.gds"
        component.write_gds(str(gds_path))
        assert gds_path.stat().st_size > 0
