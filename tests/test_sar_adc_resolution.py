from __future__ import annotations

from collections import Counter

import pytest

from glayout.cells.composite.sar_adc import (
    BIT_ORDER,
    CDAC_GROUPS,
    CDAC_PINS,
    SWITCH_DEVICES,
    SWITCH_MATRIX_PINS,
    bit_order,
    cdac_common_centroid_map,
    cdac_groups,
    cdac_pins,
    clocked_dynamic_preamp,
    preamp_devices,
    sar_cdac_array,
    sar_cdac_switch_matrix,
    strongarm_comparator,
    strongarm_offset_trim,
    switch_matrix_pins,
    trim_devices,
    trim_pins,
)
from glayout.pdk.sky130_mapped import sky130_mapped_pdk


@pytest.mark.parametrize("bits, side", [(8, 16), (10, 32)])
def test_cdac_binary_map_is_complete_and_common_centroid(bits: int, side: int) -> None:
    mapping = cdac_common_centroid_map(bits)
    expected = dict(cdac_groups(bits))

    assert len(mapping) == 1 << bits
    assert Counter(mapping.values()) == expected
    assert all(0 <= row < side and 0 <= col < side for row, col in mapping)

    centre = (side - 1) / 2
    for group, weight in cdac_groups(bits):
        if weight < 2:
            continue
        sites = [site for site, assigned in mapping.items() if assigned == group]
        assert sum(row for row, _ in sites) / weight == pytest.approx(centre)
        assert sum(col for _, col in sites) / weight == pytest.approx(centre)


def test_ten_bit_compatibility_constants_are_unchanged() -> None:
    assert BIT_ORDER == bit_order(10) == tuple(range(9, -1, -1))
    assert CDAC_GROUPS == cdac_groups(10)
    assert CDAC_PINS == cdac_pins(10)
    assert SWITCH_MATRIX_PINS == switch_matrix_pins(10)
    assert len(SWITCH_DEVICES) == 75


def test_eight_bit_public_interfaces() -> None:
    assert bit_order(8) == tuple(range(7, -1, -1))
    assert cdac_pins(8) == (
        "VDAC",
        "BP7",
        "BP6",
        "BP5",
        "BP4",
        "BP3",
        "BP2",
        "BP1",
        "BP0",
        "BPT",
        "AVSS",
    )
    assert len(switch_matrix_pins(8)) == 23


@pytest.mark.parametrize(
    "helper",
    [cdac_groups, cdac_pins, cdac_common_centroid_map, bit_order, switch_matrix_pins],
)
def test_resolution_helpers_reject_unqualified_bits(helper) -> None:
    with pytest.raises(ValueError, match="8, 10"):
        helper(7)


def test_eight_bit_layout_metadata_and_netlists() -> None:
    cdac = sar_cdac_array(sky130_mapped_pdk, bits=8)
    switch_matrix = sar_cdac_switch_matrix(sky130_mapped_pdk, bits=8)

    assert cdac.name == "sar_cdac_array_8bit"
    assert cdac.info["resolution_bits"] == 8
    assert cdac.info["active_units"] == 256
    assert cdac.info["dummy_units"] == 68
    assert cdac.info["matrix"] == (16, 16)
    assert cdac.info["physical_matrix"] == (18, 18)
    assert set(cdac_pins(8)).issubset(cdac.ports)
    cdac_netlist = cdac.info["netlist"].generate_netlist()
    assert ".subckt sar_cdac_array_8bit " in cdac_netlist
    assert cdac_netlist.count("sky130_fd_pr__cap_mim_m3_1") == 256

    assert switch_matrix.name == "sar_cdac_switch_matrix_8bit"
    assert switch_matrix.info["resolution_bits"] == 8
    assert switch_matrix.info["device_count"] == 61
    assert switch_matrix.info["bit_order"] == tuple(range(7, -1, -1))
    assert set(switch_matrix_pins(8)).issubset(switch_matrix.ports)
    switch_netlist = switch_matrix.info["netlist"].generate_netlist()
    assert ".subckt sar_cdac_switch_matrix_8bit " in switch_netlist
    assert switch_netlist.count("sky130_fd_pr__nfet_01v8") == 35
    assert switch_netlist.count("sky130_fd_pr__pfet_01v8") == 26


def test_strongarm_device_scale_preserves_matched_unit_fingers() -> None:
    comparator = strongarm_comparator(sky130_mapped_pdk, device_scale=4)
    netlist = comparator.info["netlist"].generate_netlist()

    assert comparator.info["device_scale"] == 4
    assert comparator.info["unit_fingers"]["M1"] == 32
    assert comparator.info["unit_fingers"]["M2"] == 32
    assert comparator.info["unit_fingers"]["M5"] == 16
    assert "XM1 P VIN1 TAIL 0 sky130_fd_pr__nfet_01v8 L=0.15 W=32" in netlist
    assert "XM2 Q VIN2 TAIL 0 sky130_fd_pr__nfet_01v8 L=0.15 W=32" in netlist


def test_clocked_dynamic_preamp_is_resettable_two_stage_and_noninverting() -> None:
    preamp = clocked_dynamic_preamp(sky130_mapped_pdk)
    netlist = preamp.info["netlist"].generate_netlist()

    assert preamp.info["device_count"] == 18
    assert preamp.info["stage_count"] == 2
    assert preamp.info["clock_gated"] is True
    assert preamp.info["idle_static_path"] is False
    assert preamp.info["clock_low_output_reset"] is True
    assert preamp.info["bounded_positive_feedback"] is True
    assert preamp.info["polarity"] == "OUTP_gt_OUTN_when_VINP_gt_VINN"
    assert preamp.info["row_patterns"] == {
        "s1_load": "zero_first_moment_mirrored",
        "s2_load": "zero_first_moment_mirrored",
        "s1_input": "ABBA",
        "s2_input": "ABBA",
    }
    assert len(preamp_devices()) == 18
    assert set(("VDD", "OUTP", "OUTN", "CLK", "VINP", "VINN", "VSS")) <= set(
        preamp.ports
    )
    assert "XS1_RP MIDP CLK VDD VDD" in netlist
    assert "XS1_INP MIDN VINP TAIL1 VSS" in netlist
    assert "XS2_INP OUTN MIDP TAIL2 VSS" in netlist
    assert "XS2_TAIL TAIL2 CLK VSS VSS" in netlist


def test_strongarm_input_pair_can_be_scaled_independently() -> None:
    comparator = strongarm_comparator(sky130_mapped_pdk, input_pair_scale=4)

    assert comparator.info["device_scale"] == 1
    assert comparator.info["input_pair_scale"] == 4
    assert comparator.info["unit_fingers"]["M1"] == 8
    assert comparator.info["unit_fingers"]["M2"] == 8
    assert comparator.info["finger_width_um"]["M1"] == 4
    assert comparator.info["finger_width_um"]["M2"] == 4
    assert comparator.info["unit_fingers"]["M5"] == 4
    assert comparator.info["unit_fingers"]["S1"] == 1


def test_strongarm_input_pair_area_can_grow_at_constant_w_over_l() -> None:
    comparator = strongarm_comparator(
        sky130_mapped_pdk, input_pair_scale=2, input_pair_length_scale=2
    )
    netlist = comparator.info["netlist"].generate_netlist()

    assert comparator.info["input_pair_area_scale"] == 4
    assert comparator.info["unit_fingers"]["M1"] == 8
    assert comparator.info["finger_width_um"]["M1"] == 2
    assert comparator.info["finger_length_um"]["M1"] == 0.3
    assert "XM1 P VIN1 TAIL 0 sky130_fd_pr__nfet_01v8 L=0.3 W=16" in netlist


def test_strongarm_offset_trim_is_symmetric_binary_weighted_and_clock_gated() -> None:
    trim = strongarm_offset_trim(sky130_mapped_pdk)
    netlist = trim.info["netlist"].generate_netlist()

    assert trim.info["trim_bits_per_side"] == 4
    assert trim.info["signed_code_range"] == [-15, 15]
    assert trim.info["device_count"] == 10
    assert trim.info["idle_static_path"] is False
    assert set(trim_pins()).issubset(trim.ports)
    devices = trim_devices()
    assert [device.width_um for device in devices if device.side == "X"] == [
        0.42,
        0.84,
        1.68,
        3.36,
        8.0,
    ]
    assert "XX0 X TRIM_X0 TRIM_X_TAIL VSS" in netlist
    assert "XCLK_X TRIM_X_TAIL CLK VSS VSS" in netlist
    assert "XY0 Y TRIM_Y0 TRIM_Y_TAIL VSS" in netlist
    assert "XCLK_Y TRIM_Y_TAIL CLK VSS VSS" in netlist


@pytest.mark.parametrize("scale", [0, -1, 1.5])
def test_strongarm_device_scale_rejects_invalid_values(scale) -> None:
    with pytest.raises(ValueError, match="positive integer"):
        strongarm_comparator(sky130_mapped_pdk, device_scale=scale)


@pytest.mark.parametrize("scale", [0, -1, 1.5])
def test_strongarm_input_pair_scale_rejects_invalid_values(scale) -> None:
    with pytest.raises(ValueError, match="positive integer"):
        strongarm_comparator(sky130_mapped_pdk, input_pair_scale=scale)


@pytest.mark.parametrize("scale", [0, -1, 1.5])
def test_strongarm_input_pair_length_scale_rejects_invalid_values(scale) -> None:
    with pytest.raises(ValueError, match="positive integer"):
        strongarm_comparator(sky130_mapped_pdk, input_pair_length_scale=scale)
