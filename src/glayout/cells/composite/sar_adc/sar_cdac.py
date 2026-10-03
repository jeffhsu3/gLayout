"""Full-binary, common-centroid MIM CDAC for the SKY130 SAR ADC.

This generator deliberately does not use :func:`mimcap_array`: that primitive
shorts every plate in the array.  Here every unit top plate belongs to VDAC and
every bottom plate belongs to exactly one binary-weight group.

The default 10-bit 32x32 active matrix contains 1024 identical units.  The
8-bit configuration uses the same construction at 16x16.  A one-cell dummy
perimeter surrounds either matrix.  M1 row collectors and M2 vertical trunks
make the dispersed bottom-plate assignment electrically routable without
consuming the M4/M5 capacitor plates.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
from math import sqrt
from typing import Mapping

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.comp_utils import evaluate_bbox
from glayout.util.snap_to_grid import component_snap_to_grid
from glayout.primitives.mimcap import mimcap
from glayout.primitives.via_gen import via_stack
from glayout.spice import Netlist


DEFAULT_CDAC_BITS = 10
SUPPORTED_CDAC_BITS = (8, 10)


def cdac_groups(bits: int = DEFAULT_CDAC_BITS) -> tuple[tuple[str, int], ...]:
    """Return binary bottom-plate groups plus the termination unit."""

    if bits not in SUPPORTED_CDAC_BITS:
        raise ValueError(f"bits must be one of {SUPPORTED_CDAC_BITS}; got {bits}")
    return tuple((f"BP{bit}", 1 << bit) for bit in range(bits - 1, -1, -1)) + (
        ("BPT", 1),
    )


def cdac_pins(bits: int = DEFAULT_CDAC_BITS) -> tuple[str, ...]:
    """Return the public CDAC pin order for ``bits`` resolution."""

    return ("VDAC",) + tuple(name for name, _ in cdac_groups(bits)) + ("AVSS",)


def _cdac_name(bits: int) -> str:
    return (
        "sar_cdac_array" if bits == DEFAULT_CDAC_BITS else f"sar_cdac_array_{bits}bit"
    )


CDAC_GROUPS: tuple[tuple[str, int], ...] = cdac_groups()
CDAC_PINS: tuple[str, ...] = cdac_pins()


@dataclass(frozen=True)
class MimCapCalibration:
    """Deterministic TT MIM geometry calibration result."""

    size_um: float
    capacitance_ff: float
    target_ff: float
    error_ppm: float


def _reverse_bits(value: int, width: int) -> int:
    result = 0
    for _ in range(width):
        result = (result << 1) | (value & 1)
        value >>= 1
    return result


def _quartet_order(quadrant_side: int) -> list[tuple[int, int]]:
    """Return a deterministic, spatially dispersed order for one quadrant."""

    coordinate_bits = quadrant_side.bit_length() - 1
    coords = [
        (row, col) for row in range(quadrant_side) for col in range(quadrant_side)
    ]
    return sorted(
        coords,
        key=lambda rc: _reverse_bits(
            (rc[0] << coordinate_bits) | rc[1], 2 * coordinate_bits
        ),
    )


def cdac_common_centroid_map(
    bits: int = DEFAULT_CDAC_BITS,
) -> dict[tuple[int, int], str]:
    """Map every active site to one binary CDAC group.

    Groups BP9 through BP2 are built only from four-way mirror quartets, so
    each has the array centre as its exact centroid and equal population in all
    quadrants.  The final central quartet is divided into a diagonal BP1 pair
    and the mutually mirrored BP0/termination pair.
    """

    groups = cdac_groups(bits)
    active_side = 1 << (bits // 2)
    quadrant_side = active_side // 2
    mapping: dict[tuple[int, int], str] = {}
    centre_quadrant_site = (quadrant_side - 1, quadrant_side - 1)
    quartets = [
        site for site in _quartet_order(quadrant_side) if site != centre_quadrant_site
    ]
    cursor = 0

    for name, weight in groups[:-3]:
        quartet_count = weight // 4
        for row, col in quartets[cursor : cursor + quartet_count]:
            for site in (
                (row, col),
                (active_side - 1 - row, col),
                (row, active_side - 1 - col),
                (active_side - 1 - row, active_side - 1 - col),
            ):
                if site in mapping:
                    raise RuntimeError(f"duplicate CDAC site generated: {site}")
                mapping[site] = name
        cursor += quartet_count

    if cursor != len(quartets):
        raise RuntimeError("binary groups did not consume every non-central quartet")

    low = quadrant_side - 1
    high = quadrant_side
    mapping[(low, low)] = "BP1"
    mapping[(high, high)] = "BP1"
    mapping[(low, high)] = "BP0"
    mapping[(high, low)] = "BPT"

    expected = dict(groups)
    actual = Counter(mapping.values())
    if len(mapping) != 1 << bits or actual != expected:
        raise RuntimeError(f"invalid CDAC mapping: {actual}")
    return mapping


def mimcap_capacitance_ff(
    size_um: float,
    *,
    area_ff_per_um2: float = 2.0,
    perimeter_ff_per_um: float = 0.19,
    process_delta_um: float = -0.025,
) -> float:
    """Evaluate the SKY130 TT MIM area-plus-perimeter model for a square."""

    corrected = size_um + process_delta_um
    return area_ff_per_um2 * corrected**2 + 4 * perimeter_ff_per_um * corrected


def mimcap_size_for_target(
    target_ff: float = 56.25,
    *,
    grid_um: float = 0.01,
    area_ff_per_um2: float = 2.0,
    perimeter_ff_per_um: float = 0.19,
    process_delta_um: float = -0.025,
) -> MimCapCalibration:
    """Choose the nearest legal square size for the requested TT capacitance."""

    if target_ff <= 0 or grid_um <= 0:
        raise ValueError("target capacitance and manufacturing grid must be positive")
    # Positive root of A*x^2 + 4*P*x - target = 0, where x includes DW.
    corrected = (
        -4 * perimeter_ff_per_um
        + sqrt((4 * perimeter_ff_per_um) ** 2 + 4 * area_ff_per_um2 * target_ff)
    ) / (2 * area_ff_per_um2)
    ideal = corrected - process_delta_um
    grid_index = round(ideal / grid_um)
    candidates = [max(grid_um, (grid_index + delta) * grid_um) for delta in (-1, 0, 1)]
    size = min(
        candidates,
        key=lambda candidate: abs(
            mimcap_capacitance_ff(
                candidate,
                area_ff_per_um2=area_ff_per_um2,
                perimeter_ff_per_um=perimeter_ff_per_um,
                process_delta_um=process_delta_um,
            )
            - target_ff
        ),
    )
    capacitance = mimcap_capacitance_ff(
        size,
        area_ff_per_um2=area_ff_per_um2,
        perimeter_ff_per_um=perimeter_ff_per_um,
        process_delta_um=process_delta_um,
    )
    return MimCapCalibration(
        size_um=size,
        capacitance_ff=capacitance,
        target_ff=target_ff,
        error_ppm=(capacitance - target_ff) / target_ff * 1e6,
    )


def _canonical_cdac_netlist(
    mapping: Mapping[tuple[int, int], str],
    unit_size_um: float,
    *,
    bits: int = DEFAULT_CDAC_BITS,
    include_layout_dummies: bool = False,
) -> Netlist:
    circuit_name = _cdac_name(bits)
    pins = cdac_pins(bits)
    active_side = 1 << (bits // 2)
    dummy_count = (active_side + 2) ** 2 - active_side**2
    lines = [f".subckt {circuit_name} {' '.join(pins)}"]
    for (row, col), group in sorted(mapping.items()):
        lines.append(
            f"XCU_R{row:02d}_C{col:02d} VDAC {group} "
            "sky130_fd_pr__cap_mim_m3_1 "
            f"w={unit_size_um:.3f} l={unit_size_um:.3f}"
        )
    if include_layout_dummies:
        for index in range(dummy_count):
            lines.append(
                f"XDUMMY_{index:03d} AVSS AVSS sky130_fd_pr__cap_mim_m3_1 "
                f"w={unit_size_um:.3f} l={unit_size_um:.3f}"
            )
    lines.append(f".ends {circuit_name}")
    return Netlist(
        circuit_name=circuit_name,
        nodes=list(pins),
        source_netlist="\n".join(lines),
    )


def _add_rect(
    component: Component,
    layer: tuple[int, int],
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> None:
    if xmax <= xmin or ymax <= ymin:
        raise ValueError("rectangle must have positive area")
    component.add_polygon(
        [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)],
        layer=layer,
    )


def _add_labelled_port(
    component: Component,
    *,
    name: str,
    center: tuple[float, float],
    width: float,
    orientation: int,
    layer: tuple[int, int],
) -> None:
    component.add_port(
        name=name,
        center=center,
        width=width,
        orientation=orientation,
        layer=layer,
    )
    component.add_label(text=name, position=center, layer=layer)


def sar_cdac_array(
    pdk: MappedPDK,
    *,
    bits: int = DEFAULT_CDAC_BITS,
    target_unit_cap_ff: float = 56.25,
    unit_cap_size: tuple[float, float] | None = None,
) -> Component:
    """Generate an 8- or 10-bit full-binary CDAC capacitor matrix.

    The default remains the original 10-bit ``VDAC BP9 ... BP0 BPT AVSS``
    macro.  Passing ``bits=8`` creates a 256-unit 16x16 matrix named
    ``sar_cdac_array_8bit`` with ports ``VDAC BP7 ... BP0 BPT AVSS``.  AVSS
    connects only the physical dummy perimeter; dummies are absent from the
    functional netlist.
    """

    pdk.activate()
    groups = cdac_groups(bits)
    active_side = 1 << (bits // 2)
    physical_side = active_side + 2
    if unit_cap_size is None:
        calibration = mimcap_size_for_target(target_unit_cap_ff)
        unit_cap_size = (calibration.size_um, calibration.size_um)
    elif len(unit_cap_size) != 2 or min(unit_cap_size) <= 0:
        raise ValueError("unit_cap_size must contain two positive dimensions")
    else:
        calibration = MimCapCalibration(
            size_um=unit_cap_size[0],
            capacitance_ff=mimcap_capacitance_ff(unit_cap_size[0]),
            target_ff=target_unit_cap_ff,
            error_ppm=(mimcap_capacitance_ff(unit_cap_size[0]) - target_unit_cap_ff)
            / target_unit_cap_ff
            * 1e6,
        )

    if abs(unit_cap_size[0] - unit_cap_size[1]) > 1e-12:
        raise ValueError("the matched CDAC unit must be square")

    mapping = cdac_common_centroid_map(bits)
    group_index = {name: index for index, (name, _) in enumerate(groups)}
    component = Component(name=_cdac_name(bits))
    unit = mimcap(pdk, unit_cap_size, with_extension=False)
    cap_width, cap_height = evaluate_bbox(unit)
    pitch = max(cap_width, cap_height) + pdk.get_grule("capmet")["min_separation"]
    pitch = pdk.snap_to_2xgrid(pitch)

    # Physical coordinates include one dummy cell on every side.
    active_refs: dict[tuple[int, int], object] = {}
    dummy_refs = []
    for physical_row in range(physical_side):
        for physical_col in range(physical_side):
            ref = component << unit
            ref.move((physical_col * pitch, physical_row * pitch))
            if physical_row in (0, physical_side - 1) or physical_col in (
                0,
                physical_side - 1,
            ):
                dummy_refs.append(ref)
            else:
                active_refs[(physical_row - 1, physical_col - 1)] = ref

    met1 = pdk.get_glayer("met1")
    met2 = pdk.get_glayer("met2")
    met3 = pdk.get_glayer("met3")
    met5 = pdk.get_glayer("met5")
    m1_width = pdk.get_grule("met1")["min_width"]
    m1_sep = pdk.get_grule("met1")["min_separation"]
    m2_width = pdk.get_grule("met2")["min_width"]
    m2_sep = pdk.get_grule("met2")["min_separation"]
    m3_ring_width = max(0.60, pdk.get_grule("met3")["min_width"])
    # M5 shapes wider than 3 um require 0.4 um notch spacing.  A 0.3 um
    # centreline joining 5.14 um dummy plates creates a forbidden narrow neck;
    # 0.8 um removes that notch while staying well inside the electrode.
    # Fully cover the M5 via-array landing shapes in each dummy.  A partial
    # overlap leaves hundreds of narrow notches at the landing boundary.
    m5_width = max(unit_cap_size[0], pdk.get_grule("met5")["min_width"])
    track_pitch = m1_width + m1_sep
    track_offsets = {
        name: (index - (len(groups) - 1) / 2) * track_pitch
        for name, index in group_index.items()
    }

    # A solid M5 common plate joins only the active unit top electrodes.
    active_min = pitch - unit_cap_size[0] / 2
    active_max = active_side * pitch + unit_cap_size[0] / 2
    _add_rect(component, met5, active_min, active_min, active_max, active_max)

    unit_drop = via_stack(pdk, "met1", "met4")
    row_groups: dict[int, set[str]] = defaultdict(set)
    for (row, col), group in mapping.items():
        ref = active_refs[(row, col)]
        drop = component << unit_drop
        drop.move((ref.center[0], ref.center[1] + track_offsets[group]))
        row_groups[row].add(group)

    trunk_pitch = max(
        m2_width + m2_sep, evaluate_bbox(via_stack(pdk, "met1", "met2"))[0] + m2_sep
    )
    first_active_x = pitch
    last_active_x = active_side * pitch
    first_trunk_x = (
        first_active_x - cap_width / 2 - 2.0 - trunk_pitch * (len(groups) - 1)
    )
    trunk_x = {
        name: first_trunk_x + index * trunk_pitch for name, index in group_index.items()
    }
    row_via = via_stack(pdk, "met1", "met2")
    group_track_ys: dict[str, list[float]] = defaultdict(list)

    for row, present_groups in row_groups.items():
        row_y = (row + 1) * pitch
        for group in sorted(present_groups, key=group_index.get):
            y = row_y + track_offsets[group]
            group_track_ys[group].append(y)
            _add_rect(
                component,
                met1,
                trunk_x[group],
                y - m1_width / 2,
                last_active_x,
                y + m1_width / 2,
            )
            transition = component << row_via
            transition.move((trunk_x[group], y))

    trunk_bottom = active_min - 3.0
    for group, _ in groups:
        ys = group_track_ys[group]
        _add_rect(
            component,
            met2,
            trunk_x[group] - m2_width / 2,
            trunk_bottom,
            trunk_x[group] + m2_width / 2,
            max(ys),
        )
        _add_labelled_port(
            component,
            name=group,
            center=(trunk_x[group], trunk_bottom),
            width=m2_width,
            orientation=270,
            layer=met2,
        )

    # Join both plates of every dummy to an AVSS perimeter.  The bottom ring is
    # on M3, not M1: an M1 ring would cross and short all eleven active row
    # collectors on their way to the left-hand M2 trunks.
    dummy_drop = via_stack(pdk, "met3", "met4")
    for ref in dummy_refs:
        drop = component << dummy_drop
        drop.move(ref.center)

    physical_min = -unit_cap_size[0] / 2
    physical_max = (physical_side - 1) * pitch + unit_cap_size[0] / 2
    ring_centres = (0.0, (physical_side - 1) * pitch)
    for layer, width in ((met3, m3_ring_width), (met5, m5_width)):
        for y in ring_centres:
            _add_rect(
                component,
                layer,
                physical_min,
                y - width / 2,
                physical_max,
                y + width / 2,
            )
        for x in ring_centres:
            _add_rect(
                component,
                layer,
                x - width / 2,
                physical_min,
                x + width / 2,
                physical_max,
            )
    # Join the two rings outside the MIM keep-out.  Putting this stack at the
    # corner dummy centre overlaps the capacitor's own M4/M5 access structure;
    # Magic then extracts two AVSS-labelled islands instead of a real short.
    avss_join_x = physical_min - 2.0
    _add_rect(
        component,
        met3,
        avss_join_x - 1.0,
        -m3_ring_width / 2,
        physical_min,
        m3_ring_width / 2,
    )
    _add_rect(
        component,
        met5,
        avss_join_x - 1.0,
        -m5_width / 2,
        physical_min,
        m5_width / 2,
    )
    avss_join = component << via_stack(pdk, "met3", "met5")
    avss_join.move((avss_join_x, 0.0))

    _add_labelled_port(
        component,
        name="VDAC",
        center=(active_max, (active_min + active_max) / 2),
        width=m5_width,
        orientation=0,
        layer=met5,
    )
    _add_labelled_port(
        component,
        name="AVSS",
        center=(physical_max, 0.0),
        width=m5_width,
        orientation=0,
        layer=met5,
    )

    component.info["netlist"] = _canonical_cdac_netlist(
        mapping, unit_cap_size[0], bits=bits
    )
    component.info["lvs_netlist"] = _canonical_cdac_netlist(
        mapping, unit_cap_size[0], bits=bits, include_layout_dummies=True
    )
    component.info["weights"] = dict(groups)
    component.info["resolution_bits"] = bits
    component.info["active_units"] = 1 << bits
    component.info["dummy_units"] = len(dummy_refs)
    component.info["matrix"] = (active_side, active_side)
    component.info["physical_matrix"] = (physical_side, physical_side)
    component.info["unit_cap_size_um"] = unit_cap_size
    component.info["unit_capacitance_ff"] = calibration.capacitance_ff
    component.info["unit_cap_error_ppm"] = calibration.error_ppm
    component.info["mapping"] = {
        f"r{row:02d}c{col:02d}": group for (row, col), group in mapping.items()
    }
    return component_snap_to_grid(component)


if __name__ == "__main__":
    from glayout.pdk.sky130_mapped import sky130_mapped_pdk

    sar_cdac_array(sky130_mapped_pdk).write_gds("sar_cdac_array.gds")
