"""SKY130 vertical-parallel-plate (VPP) finger capacitors.

The geometry below reproduces the characterized SKY130 ``fingercap`` cells
instead of approximating them with the process' dedicated MIM dielectric.  The
default array combines four 2.7 x 41.1 um fingers and one 2.7 x 11.1 um finger,
which the SKY130 model data puts at 396.7 fF nominal -- effectively the 0.4 pF
metal-to-metal output capacitor used by the Seok 2T reference.

The unit-cell geometry and model names follow the Apache-2.0-licensed
``skywater-pdk-libs-sky130_fd_pr`` VPP cells.
"""

from __future__ import annotations

from glayout.backend import Component
from glayout.pdk.mappedpdk import MappedPDK


SKY130_VPP_FINGER_41_MODEL = (
    "sky130_fd_pr__cap_vpp_02p7x41p1_m1m2m3m4_shieldl1_fingercap"
)
SKY130_VPP_FINGER_11_MODEL = (
    "sky130_fd_pr__cap_vpp_02p7x11p1_m1m2m3m4_shieldl1_fingercap"
)

# Capacitances reported by the corresponding SKY130 cell-model tests.
SKY130_VPP_FINGER_41_F = 93.18776e-15
SKY130_VPP_FINGER_11_F = 23.98758e-15
SKY130_VPP_0P4_MODELS = (
    *(SKY130_VPP_FINGER_41_MODEL for _ in range(4)),
    SKY130_VPP_FINGER_11_MODEL,
)
SKY130_VPP_0P4_F = 4 * SKY130_VPP_FINGER_41_F + SKY130_VPP_FINGER_11_F


def _require_sky130(pdk: MappedPDK) -> None:
    pdk_name = pdk.name.lower()
    if pdk_name == "gf180":
        raise NotImplementedError(
            "GF180 has no equivalent model for this characterized SKY130 VPP "
            "finger capacitor."
        )
    if pdk_name != "sky130":
        raise NotImplementedError(
            "The characterized VPP finger capacitor is SKY130-only."
        )


def _rect(
    component: Component,
    bounds: tuple[float, float, float, float],
    layer: tuple[int, int],
) -> None:
    xmin, ymin, xmax, ymax = bounds
    component.add_polygon(
        [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)],
        layer=layer,
    )


def _sky130_vpp_finger_unit(pdk: MappedPDK, height: float) -> Component:
    """Return an exact 2.7-um-wide SKY130 LI-through-M4 VPP finger unit."""
    _require_sky130(pdk)
    if height == 41.1:
        model = SKY130_VPP_FINGER_41_MODEL
        capacitance = SKY130_VPP_FINGER_41_F
        pwell_pin = (1.29, 2.85, 1.395, 3.095)
        c0_pin = (0.705, 1.34, 0.8, 1.44)
        c1_pin = (1.44, 0.095, 1.62, 0.275)
        labels = (("C0", 0.74, 1.4), ("C1", 1.545, 0.185), ("SUB", 1.345, 2.975))
    elif height == 11.1:
        model = SKY130_VPP_FINGER_11_MODEL
        capacitance = SKY130_VPP_FINGER_11_F
        pwell_pin = (1.28, 2.835, 1.385, 3.08)
        c0_pin = (0.695, 1.325, 0.79, 1.425)
        c1_pin = (1.43, 0.08, 1.61, 0.26)
        labels = (("C0", 0.73, 1.385), ("C1", 1.535, 0.17), ("SUB", 1.335, 2.96))
    else:
        raise ValueError(
            "The characterized SKY130 VPP finger heights are 11.1 and 41.1 um."
        )

    unit = Component(model)

    # SKY130-specific markers present in the foundry VPP cells.
    _rect(unit, (0.405, 0.0, 2.295, height), (22, 48))
    _rect(unit, (0.17, 0.17, 2.53, height - 0.17), (82, 64))
    _rect(unit, pwell_pin, (122, 16))

    # Grounded LI shield: tap + LI rails with a 0.4-um-pitch licon ladder.
    for xmin, xmax in ((0.065, 0.235), (2.465, 2.635)):
        _rect(unit, (xmin, 0.0, xmax, height), pdk.layers["tap"])
        _rect(unit, (xmin, 0.0, xmax, height), pdk.layers["li1"])
        contact_count = int(round((height - 0.7) / 0.4))
        for index in range(contact_count):
            ymin = 0.355 + 0.4 * index
            _rect(
                unit,
                (xmin, ymin, xmax, ymin + 0.17),
                pdk.layers["licon1"],
            )

    # Alternating fingers on physical M1 and M2.
    metal1 = [
        (0.3, 0.0, 2.4, 0.33),
        (0.07, height - 0.33, 2.63, height),
        (0.07, 0.63, 0.23, height - 0.33),
        (0.37, 0.33, 0.53, height - 0.63),
        (0.67, 0.63, 0.83, height - 0.33),
        (0.97, 0.33, 1.13, height - 0.63),
        (1.27, 0.63, 1.43, height - 0.33),
        (1.57, 0.33, 1.73, height - 0.63),
        (1.87, 0.63, 2.03, height - 0.33),
        (2.17, 0.33, 2.33, height - 0.63),
        (2.47, 0.63, 2.63, height - 0.33),
    ]
    metal2 = [
        (0.3, 0.0, 2.4, 0.33),
        (0.07, height - 0.33, 0.83, height),
        (1.27, height - 0.33, 2.63, height),
        (0.07, 0.63, 0.23, height - 0.33),
        (0.37, 0.33, 0.53, height - 0.63),
        (0.67, 0.63, 0.83, height - 0.33),
        (0.97, 0.33, 1.13, height),
        (1.27, 0.63, 1.43, height - 0.33),
        (1.57, 0.33, 1.73, height - 0.63),
        (1.87, 0.63, 2.03, height - 0.33),
        (2.17, 0.33, 2.33, height - 0.63),
        (2.47, 0.63, 2.63, height - 0.33),
    ]
    metal34 = [
        (0.3, 0.0, 2.4, 0.33),
        (0.0, height - 0.33, 2.7, height),
        (0.0, 0.63, 0.3, height - 0.33),
        (0.6, 0.33, 0.9, height - 0.63),
        (1.2, 0.63, 1.5, height - 0.33),
        (1.8, 0.33, 2.1, height - 0.63),
        (2.4, 0.63, 2.7, height - 0.33),
    ]
    for bounds in metal1:
        _rect(unit, bounds, pdk.layers["met1"])
    for bounds in metal2:
        _rect(unit, bounds, pdk.layers["met2"])
    for metal in ("met3", "met4"):
        for bounds in metal34:
            _rect(unit, bounds, pdk.layers[metal])

    # Bus vias, copied from the characterized cell.  M1/M2 use five bottom
    # contacts and four top contacts; M3 adds two top contacts.
    for xmin in (0.475, 0.875, 1.275, 1.675, 2.075):
        _rect(unit, (xmin, 0.09, xmin + 0.15, 0.24), pdk.layers["via"])
    for xmin in (0.275, 1.475, 1.875, 2.275):
        _rect(
            unit,
            (xmin, height - 0.24, xmin + 0.15, height - 0.09),
            pdk.layers["via"],
        )
    for via in ("via2", "via3"):
        for xmin in (0.45, 0.85, 1.25, 1.65, 2.05):
            _rect(unit, (xmin, 0.065, xmin + 0.2, 0.265), pdk.layers[via])
    for xmin in (0.25, 1.45, 1.85, 2.25):
        _rect(
            unit,
            (xmin, height - 0.265, xmin + 0.2, height - 0.065),
            pdk.layers["via2"],
        )
    for xmin in (0.25, 0.65, 1.05, 1.45, 1.85, 2.25):
        _rect(
            unit,
            (xmin, height - 0.265, xmin + 0.2, height - 0.065),
            pdk.layers["via3"],
        )

    # Foundry macro pin shapes and labels.  The electrical route ports below
    # are exposed on physical M4, while these shapes preserve Magic extraction.
    _rect(unit, c0_pin, (69, 16))
    _rect(unit, c1_pin, (69, 16))
    for text, x, y in labels:
        unit.add_label(
            text,
            position=(x, y),
            layer=(64, 59) if text == "SUB" else (69, 5),
        )

    for name, x, y, orientation in (
        ("C0_W", 0.0, height - 0.165, 180),
        ("C0_E", 2.7, height - 0.165, 0),
        ("C1_W", 0.3, 0.165, 180),
        ("C1_E", 2.4, 0.165, 0),
    ):
        unit.add_port(
            name=name,
            center=(x, y),
            width=0.33,
            orientation=orientation,
            layer=pdk.layers["met4"],
        )
    unit.add_port(
        name="SUB",
        center=((pwell_pin[0] + pwell_pin[2]) / 2, (pwell_pin[1] + pwell_pin[3]) / 2),
        width=pwell_pin[3] - pwell_pin[1],
        orientation=0,
        layer=(122, 16),
    )
    unit.info["model"] = model
    unit.info["capacitance_f"] = capacitance
    return unit


def sky130_vpp_cap_0p4(pdk: MappedPDK) -> Component:
    """Build the paper's approximately 0.4 pF VPP capacitor for SKY130.

    Four long characterized finger cells plus one short cell give 396.7 fF.
    C0 is the signal plate, C1 the quiet plate, and the LI shields/substrate are
    intended to share the surrounding VSS substrate.
    """
    _require_sky130(pdk)
    cap = Component("SKY130_VPP_0P4")
    pitch = 3.1  # 2.7-um cell plus 0.4-um top-metal spacing.
    heights = (41.1, 41.1, 41.1, 41.1, 11.1)
    units = {
        41.1: _sky130_vpp_finger_unit(pdk, 41.1),
        11.1: _sky130_vpp_finger_unit(pdk, 11.1),
    }
    refs = []
    for index, height in enumerate(heights):
        ref = cap << units[height]
        ref.movex(index * pitch)
        refs.append(ref)

    xmax = (len(heights) - 1) * pitch + 2.7
    met4 = pdk.layers["met4"]
    # Parallel plate buses.  The short cell's C0 finger is extended to the
    # common top bus; all C1 bottom buses already line up.
    _rect(cap, (0.0, 40.77, xmax, 41.1), met4)
    _rect(cap, (0.3, 0.0, xmax - 0.3, 0.33), met4)
    short_x = (len(heights) - 1) * pitch
    _rect(cap, (short_x, 10.77, short_x + 0.3, 40.77), met4)

    cap.add_port("C0_W", center=(0.0, 40.935), width=0.33, orientation=180, layer=met4)
    cap.add_port("C1_W", center=(0.3, 0.165), width=0.33, orientation=180, layer=met4)
    cap.add_port("C0_E", center=(xmax, 40.935), width=0.33, orientation=0, layer=met4)
    cap.add_port(
        "C1_E",
        center=(xmax - 0.3, 0.165),
        width=0.33,
        orientation=0,
        layer=met4,
    )
    cap.add_port("SUB", port=refs[0].ports["SUB"])
    cap.info["capacitance_f"] = SKY130_VPP_0P4_F
    cap.info["models"] = SKY130_VPP_0P4_MODELS
    cap.info["unit_count"] = len(heights)
    return cap


__all__ = [
    "SKY130_VPP_0P4_F",
    "SKY130_VPP_0P4_MODELS",
    "SKY130_VPP_FINGER_11_MODEL",
    "SKY130_VPP_FINGER_41_MODEL",
    "sky130_vpp_cap_0p4",
]
