from glayout.backend import Component, cell
from glayout.pdk.mappedpdk import MappedPDK
from glayout.util.comp_utils import evaluate_bbox
from glayout.primitives.mimcap import mimcap
from glayout.routing.straight_route import straight_route
from glayout.spice.netlist import Netlist

#: The direction mimcap runs its bottom-plate extension in.  It has to be off
#: the routing axis: the top plates are shorted along the row, east to west, and
#: an extension on E or W would put the COMMON contact directly in that path.
#: S also keeps every bottom contact on one side, so COMMON is a single strip.
_BOTTOM_EXTENSION = "S"

#: The bottom plate's contact on the top metal, as mimcap names it.  The plate
#: itself carries no port -- mimcap registers the via array's, not the
#: rectangle's -- and this is the via stack that lifts it to the same glayer as
#: the top plate, on the ``_BOTTOM_EXTENSION`` side.
_BOTTOM_PORT = f"bot_via_{_BOTTOM_EXTENSION}_top_met_"


def _cap_dac_netlist(unit_cap: Component, weights: tuple[int, ...]) -> Netlist:
    nodes = [f"BIT{i}" for i in range(len(weights))] + ["COMMON"]
    netlist = Netlist(circuit_name="cap_dac", nodes=nodes)

    for bit_index, weight in enumerate(weights):
        for _ in range(weight):
            netlist.connect_netlist(
                unit_cap.info["netlist"],
                [
                    ("V1", f"BIT{bit_index}"),
                    ("V2", "COMMON"),
                ],
            )

    return netlist


def _normalize_weights(bits: int, weights: tuple[int, ...] | None) -> tuple[int, ...]:
    if weights is None:
        if bits < 1:
            raise ValueError("bits must be >= 1")
        return tuple(2**bit for bit in range(bits))

    weights = tuple(int(weight) for weight in weights)
    if len(weights) < 1:
        raise ValueError("weights must contain at least one entry")
    if any(weight < 1 for weight in weights):
        raise ValueError("all capacitor DAC weights must be >= 1")
    return weights


@cell
def cap_dac(
    pdk: MappedPDK,
    bits: int = 4,
    weights: tuple[int, ...] | None = None,
    unit_cap_size: tuple[float, float] = (5.0, 5.0),
    route_width: float | None = None,
) -> Component:
    """Binary or custom-weight MIM capacitor DAC.

    The layout is a one-row array of unit MIM caps. All bottom plates are shorted
    to COMMON. Unit top plates are shorted only within each weighted group and
    exposed as BIT0, BIT1, ... . With default weights, BITi has 2**i units.

    Both terminals leave the unit cap on the same glayer -- the top plate
    directly, the bottom plate through the via stack on its ``S`` extension --
    so the two nets are separated by geometry rather than by layer: COMMON runs
    along the row below the plates, at the extension's y, and each BIT runs
    across the plates themselves.

    Ports:
    BIT<i>  -- top plate of weighted capacitor group i
    COMMON  -- shared bottom plate
    """
    weights = _normalize_weights(bits, weights)
    total_units = sum(weights)

    top_level = Component(name=f"cap_dac_{len(weights)}bit")
    unit_cap = mimcap(
        pdk, unit_cap_size, extension_direction=_BOTTOM_EXTENSION
    )
    if f"{_BOTTOM_PORT}E" not in unit_cap.ports:
        raise KeyError(
            f"mimcap exposes no {_BOTTOM_PORT}E port, so the bottom plate "
            "cannot be reached.  Check that it was built with "
            f"with_extension=True and extension_direction={_BOTTOM_EXTENSION!r}.")
    capmet_sep = pdk.get_grule("capmet")["min_separation"]
    x_pitch = evaluate_bbox(unit_cap)[0] + capmet_sep

    def join(left_ref, right_ref, port: str):
        """Short one terminal of two neighbouring units, at its own width.

        Routing at the port's width rather than the layer minimum matters here.
        Both terminals present a face the full height of the metal they sit on
        -- 5 um for the top plate, the extension's 0.4 um for the bottom -- and
        a minimum-width link between two of them leaves a notch on both sides
        of the gap.  Matching the face makes the join a plain rectangle.
        """
        edge1 = left_ref.ports[f"{port}E"]
        edge2 = right_ref.ports[f"{port}W"]
        return straight_route(
            pdk, edge1, edge2,
            width=route_width if route_width is not None else edge1.width,
        )

    cap_refs = []
    for unit_index in range(total_units):
        cap_ref = top_level << unit_cap
        cap_ref.movex(unit_index * x_pitch)
        cap_refs.append(cap_ref)

    # Shared bottom plate: route every neighboring pair into one COMMON net.
    for left_ref, right_ref in zip(cap_refs[:-1], cap_refs[1:]):
        top_level << join(left_ref, right_ref, _BOTTOM_PORT)

    bit_start = 0
    for bit_index, weight in enumerate(weights):
        bit_refs = cap_refs[bit_start : bit_start + weight]

        # Weighted top plate: short only the units belonging to this bit.
        for left_ref, right_ref in zip(bit_refs[:-1], bit_refs[1:]):
            top_level << join(left_ref, right_ref, "top_met_")

        bit_port = bit_refs[0].ports["top_met_N"]
        top_level.add_port(name=f"BIT{bit_index}", port=bit_port)
        top_level.add_label(
            text=f"BIT{bit_index}",
            position=(bit_port.center[0], bit_port.center[1] - 0.1),
            layer=bit_port.layer,
        )
        bit_start += weight

    common_port = cap_refs[0].ports[f"{_BOTTOM_PORT}S"]
    top_level.add_port(name="COMMON", port=common_port)
    top_level.add_label(
        text="COMMON",
        position=(common_port.center[0], common_port.center[1] + 0.1),
        layer=common_port.layer,
    )

    top_level.info["netlist"] = _cap_dac_netlist(unit_cap, weights)
    top_level.info["weights"] = weights
    top_level.info["unit_cap_size"] = unit_cap_size

    return top_level


if __name__ == "__main__":
    from glayout.pdk.gf180_mapped import gf180_mapped_pdk

    print("Generating cap_dac...")
    comp = cap_dac(gf180_mapped_pdk, bits=4)
    comp.write_gds("cap_dac.gds")
    print(f"Generated GDS: {comp.name}")
