"""Seok-style two-transistor, ultra-low-power voltage reference.

The topology and default dimensions follow Figure 1 of M. Seok et al.,
"A 0.5 V 3.6 ppm/°C 2.2 pW 2-Transistor Voltage Reference":

* M1 is a 3.3/60 um zero-threshold NFET from VDD to VREF, with its gate at VSS.
* M2 is a 1.5/60 um I/O NFET, diode-connected from VREF to VSS.
* Both devices use the same thick gate oxide.
* A nominal 0.4 pF VPP finger capacitor connects VREF to VSS for PSRR.

SKY130 implements that pairing with ``nfet_05v0_nvt`` and
``nfet_g5v0d10v5``.  The layout therefore carries HVI and HVNTM on both
devices, plus LVTN on M1.  In particular, it deliberately does not draw LVID:
SKY130 uses LVID to select the 3.3 V native device, while the paper-faithful
matched-thick-oxide pair needs the 5 V native device.
"""

from __future__ import annotations

from typing import Optional

from glayout.backend import Component, rectangle
from glayout.pdk.mappedpdk import MappedPDK
from glayout.primitives.fet import nmos
from glayout.primitives.guardring import tapring
from glayout.primitives.vpp import (
    SKY130_VPP_0P4_F,
    SKY130_VPP_0P4_MODELS,
    sky130_vpp_cap_0p4,
)
from glayout.routing.c_route import c_route
from glayout.spice.netlist import Netlist
from glayout.util.label_utils import LabelSpec, add_pin_labels


SKY130_NATIVE_MODEL = "sky130_fd_pr__nfet_05v0_nvt"
SKY130_IO_MODEL = "sky130_fd_pr__nfet_g5v0d10v5"
_MODEL_ALIASES = {
    "nfet_native": SKY130_NATIVE_MODEL,
    "nfet_io": SKY130_IO_MODEL,
}


def _require_supported_pdk(pdk: MappedPDK) -> None:
    """Reject PDKs that cannot express the paper's matched-oxide pair."""
    pdk_name = pdk.name.lower()
    if pdk_name == "gf180":
        raise NotImplementedError(
            "The Seok 2T voltage reference is unavailable for GF180: the "
            "mapped GF180 PDK has no equivalent ZVT/I/O pair with a shared "
            "thick gate oxide."
        )
    if pdk_name != "sky130":
        raise NotImplementedError(
            f"The Seok 2T voltage reference has no device-layer mapping for "
            f"PDK {pdk.name!r}."
        )
    try:
        pdk.has_required_glayers(["hvi", "hvntm", "lvtn"])
    except (KeyError, TypeError, ValueError) as exc:
        raise NotImplementedError(
            "The SKY130 mapping must provide hvi, hvntm, and lvtn to build "
            "the matched-thick-oxide 2T reference."
        ) from exc


def _resolve_model(pdk: MappedPDK, model: str) -> str:
    model = _MODEL_ALIASES.get(model, model)
    return pdk.models.get(model, model)


def _markers_for_model(model: str) -> tuple[str, ...]:
    if model == SKY130_NATIVE_MODEL:
        return ("hvi", "hvntm", "lvtn")
    if model == SKY130_IO_MODEL:
        return ("hvi", "hvntm")
    raise ValueError(
        f"Unsupported 2T-reference device model {model!r}; use "
        f"{SKY130_NATIVE_MODEL!r} for M1 and {SKY130_IO_MODEL!r} for M2."
    )


def _add_device_markers(
    device: Component,
    pdk: MappedPDK,
    model: str,
) -> Component:
    """Enclose one generic NFET with the marker set that selects ``model``."""
    (xmin, ymin), (xmax, ymax) = device.bbox
    center = ((xmin + xmax) / 2, (ymin + ymax) / 2)
    base_size = (xmax - xmin, ymax - ymin)

    for glayer in _markers_for_model(model):
        relation = "poly" if glayer == "lvtn" else "active_diff"
        enclosure = pdk.get_grule(glayer, relation)["min_enclosure"]
        enclosure = float(pdk.snap_to_2xgrid(enclosure))
        marker = rectangle(
            size=(
                float(pdk.snap_to_2xgrid(base_size[0] + 2 * enclosure)),
                float(pdk.snap_to_2xgrid(base_size[1] + 2 * enclosure)),
            ),
            layer=pdk.get_glayer(glayer),
            centered=True,
        )
        marker_ref = device << marker
        marker_ref.move(center)
    return device


def vt_ref_netlist(
    pdk: MappedPDK,
    w1: float = 3.3,
    w2: float = 1.5,
    length: Optional[float] = None,
    model1: str = SKY130_NATIVE_MODEL,
    model2: str = SKY130_IO_MODEL,
    subckt_only: bool = False,
    with_cout: bool = True,
) -> Netlist:
    """Return the paper-faithful 2T reference subcircuit.

    ``subckt_only`` is retained for API compatibility with the OpenFASOC
    generator; a :class:`Netlist` is returned in either mode.
    """
    del subckt_only
    _require_supported_pdk(pdk)
    length = 60 if length is None else length
    m1 = _resolve_model(pdk, model1)
    m2 = _resolve_model(pdk, model2)
    _markers_for_model(m1)
    _markers_for_model(m2)

    cout_instances = ""
    if with_cout:
        cout_instances = "\n" + "\n".join(
            f"XCOUT{index} VREF VSS VSS {model}"
            for index, model in enumerate(SKY130_VPP_0P4_MODELS)
        )

    source = (
        ".subckt {circuit_name} {nodes} "
        + f"l={length} w1={w1} w2={w2} "
        + """
XM1 VDD VSS VREF VSS {m1} l={length} w={w1}
XM2 VREF VREF VSS VSS {m2} l={length} w={w2}"""
        + cout_instances
        + "\n.ends {circuit_name}"
    )
    return Netlist(
        circuit_name="VT_REF",
        nodes=["VDD", "VREF", "VSS"],
        source_netlist=source,
        instance_format=("X{name} {nodes} {circuit_name} l={length} w1={w1} w2={w2}"),
        parameters={
            "m1": m1,
            "m2": m2,
            "length": length,
            "w1": w1,
            "w2": w2,
        },
    )


def vt_ref(
    pdk: MappedPDK,
    w1: float = 3.3,
    w2: float = 1.5,
    length: Optional[float] = None,
    with_dummy: bool = False,
    m1_model: Optional[str] = None,
    m2_model: Optional[str] = None,
    with_cout: bool = True,
    **kwargs,
) -> Component:
    """Build the Seok 2T reference using the SKY130 matched-thick-oxide pair.

    Args:
        pdk: Must be the mapped SKY130 PDK.  GF180 is rejected explicitly.
        w1: Width of the native/ZVT M1, in um.
        w2: Width of the I/O M2, in um.
        length: Common gate length, in um; defaults to the paper's 60 um.
        with_dummy: Add a dummy on both sides of each transistor.
        m1_model: Native-device model or the ``nfet_native`` alias.
        m2_model: I/O-device model or the ``nfet_io`` alias.
        with_cout: Include the paper's nominal 0.4 pF VPP finger capacitor.
        **kwargs: Additional arguments forwarded to :func:`nmos`.
    """
    _require_supported_pdk(pdk)
    length = 60 if length is None else length
    m1_model = _resolve_model(pdk, m1_model or SKY130_NATIVE_MODEL)
    m2_model = _resolve_model(pdk, m2_model or SKY130_IO_MODEL)

    m1 = nmos(
        pdk,
        width=w1,
        length=length,
        with_dummy=with_dummy,
        with_tie=False,
        with_dnwell=False,
        with_substrate_tap=False,
        **kwargs,
    )
    m2 = nmos(
        pdk,
        width=w2,
        length=length,
        with_dummy=with_dummy,
        with_tie=False,
        with_dnwell=False,
        with_substrate_tap=False,
        **kwargs,
    )
    _add_device_markers(m1, pdk, m1_model)
    _add_device_markers(m2, pdk, m2_model)

    top = Component("VT_REF")
    m1_ref = top << m1
    m2_ref = top << m2
    device_spacing = max(float(pdk.get_grule("poly")["min_separation"]), 2.0)
    m2_ref.movex(m1_ref.xmax + device_spacing - m2_ref.xmin)

    # M2 is diode-connected (D=G=VREF).  Its east-side diode route shares the
    # drain landing with the M1-source route, so both shapes are one VREF net.
    top << c_route(
        pdk,
        m2_ref.ports["drain_E"],
        m2_ref.ports["gate_E"],
        extension=1.0,
        cglayer="met3",
        viaoffset=False,
    )
    top << c_route(
        pdk,
        m1_ref.ports["source_E"],
        m2_ref.ports["drain_E"],
        extension=2.0,
        cglayer="met3",
    )

    # M1 gate and M2 source are VSS.  Carry this connection around the west
    # side, opposite the VREF C-routes, to keep the two subthreshold nets apart.
    top << c_route(
        pdk,
        m1_ref.ports["gate_W"],
        m2_ref.ports["source_W"],
        extension=1.0,
        # c_route otherwise inherits the wider gate landing for both arms.
        # Preserve M2's narrower source width so its source-to-drain spacing
        # remains legal at the two ends of the long VSS arm.
        width2=m2_ref.ports["source_W"].width,
        cglayer="met3",
    )

    # A shared p-tap ring ties both bodies to VSS.  Size it after routing so it
    # encloses the complete core and remains clear of HVI/HVNTM/LVTN.
    tap_separation = max(
        float(pdk.util_max_metal_seperation()),
        float(pdk.get_grule("active_diff", "active_tap")["min_separation"]),
        float(pdk.get_grule("hvntm", "active_tap")["min_separation"]),
    )
    tap_separation += float(pdk.get_grule("p+s/d", "active_tap")["min_enclosure"])
    center = ((top.xmin + top.xmax) / 2, (top.ymin + top.ymax) / 2)
    tie_ref = top << tapring(
        pdk,
        enclosed_rectangle=(
            top.xmax - top.xmin + 2 * tap_separation,
            top.ymax - top.ymin + 2 * tap_separation,
        ),
        sdlayer="p+s/d",
        horizontal_glayer="met2",
        vertical_glayer="met1",
    )
    tie_ref.movex(center[0]).movey(center[1])
    top << c_route(
        pdk,
        m1_ref.ports["gate_W"],
        tie_ref.ports["W_top_met_W"],
        extension=0.5,
        cglayer="met3",
    )
    top.add_ports(tie_ref.get_ports_list(), prefix="welltie_")

    # Ensure the p-tap is in the same pwell/substrate region as both devices.
    top.add_padding(
        layers=(pdk.get_glayer("pwell"),),
        default=float(pdk.get_grule("pwell", "active_tap")["min_enclosure"]),
    )

    if with_cout:
        cout_ref = top << sky130_vpp_cap_0p4(pdk)
        cout_spacing = max(float(pdk.util_max_metal_seperation()), 2.0)
        cout_ref.movex(tie_ref.xmax + cout_spacing - cout_ref.xmin)
        cout_ref.movey(center[1] - cout_ref.center[1])

        # Keep the two long cross-cell routes on separate upper metals.  C0 is
        # the VREF plate; C1 and the VPP substrate shields return to VSS.
        top << c_route(
            pdk,
            m1_ref.ports["source_W"],
            cout_ref.ports["C0_W"],
            extension=2.0,
            width2=cout_ref.ports["C0_W"].width,
            # c_route flushes the spine inside both landing edges; allow one
            # 2x-grid quantum so the written GDS retains the 0.30-um M4 rule.
            cwidth=float(
                pdk.snap_to_2xgrid(
                    pdk.get_grule("met5")["min_width"] + 2 * pdk.grid_size
                )
            ),
            cglayer="met5",
        )
        top << c_route(
            pdk,
            m1_ref.ports["gate_W"],
            cout_ref.ports["C1_W"],
            extension=4.0,
            cwidth=float(
                pdk.snap_to_2xgrid(
                    pdk.get_grule("met4")["min_width"] + 2 * pdk.grid_size
                )
            ),
            cglayer="met4",
        )
        top.info["cout_f"] = SKY130_VPP_0P4_F
        top.info["cout_type"] = "sky130_vpp_finger"

    top.add_port(name="VDD", port=m1_ref.ports["drain_W"])
    top.add_port(name="VREF", port=m1_ref.ports["source_W"])
    top.add_port(name="VSS", port=m1_ref.ports["gate_W"])
    add_pin_labels(
        top,
        pdk,
        [
            LabelSpec("VDD", "VDD", size=0.2, alignment=("c", "c")),
            LabelSpec("VREF", "VREF", size=0.2, alignment=("c", "c")),
            LabelSpec("VSS", "VSS", size=0.2, alignment=("c", "c")),
        ],
        flatten=False,
    )

    top.info["netlist"] = vt_ref_netlist(
        pdk,
        w1=w1,
        w2=w2,
        length=length,
        model1=m1_model,
        model2=m2_model,
        subckt_only=True,
        with_cout=with_cout,
    )
    return top


__all__ = [
    "SKY130_IO_MODEL",
    "SKY130_NATIVE_MODEL",
    "vt_ref",
    "vt_ref_netlist",
]
