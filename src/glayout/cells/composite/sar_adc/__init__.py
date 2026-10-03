"""Schematic-faithful layout generators for the SKY130 SAR ADC analog core."""

from .sar_cdac import (
    CDAC_GROUPS,
    CDAC_PINS,
    DEFAULT_CDAC_BITS,
    SUPPORTED_CDAC_BITS,
    cdac_common_centroid_map,
    cdac_groups,
    cdac_pins,
    mimcap_size_for_target,
    sar_cdac_array,
)
from .sampling_frontend import (
    BOOTSTRAP_DEVICES,
    BOOTSTRAP_PINS,
    SAMPLING_FRONTEND_PINS,
    TOP_RESET_DEVICES,
    TOP_RESET_PINS,
    sar_bootstrapped_switch,
    sar_sampling_frontend,
    sar_top_plate_reset,
)
from .comparator_trim import (
    DEFAULT_TRIM_BITS,
    TRIM_PINS,
    strongarm_offset_trim,
    trim_devices,
    trim_pins,
)
from .dynamic_preamp import (
    DYNAMIC_PREAMP_PINS,
    clocked_dynamic_preamp,
    preamp_devices,
)
from .strongarm import STRONGARM_PINS, strongarm_comparator
from .switch_matrix import (
    BIT_ORDER,
    DEFAULT_SAR_BITS,
    SUPPORTED_SAR_BITS,
    SWITCH_DEVICES,
    SWITCH_MATRIX_PINS,
    bit_order,
    sar_cdac_switch_matrix,
    switch_matrix_pins,
)

__all__ = [
    "BIT_ORDER",
    "CDAC_GROUPS",
    "CDAC_PINS",
    "DEFAULT_CDAC_BITS",
    "DEFAULT_SAR_BITS",
    "BOOTSTRAP_DEVICES",
    "BOOTSTRAP_PINS",
    "DEFAULT_TRIM_BITS",
    "DYNAMIC_PREAMP_PINS",
    "SAMPLING_FRONTEND_PINS",
    "STRONGARM_PINS",
    "TRIM_PINS",
    "SWITCH_DEVICES",
    "SWITCH_MATRIX_PINS",
    "SUPPORTED_CDAC_BITS",
    "SUPPORTED_SAR_BITS",
    "TOP_RESET_DEVICES",
    "TOP_RESET_PINS",
    "bit_order",
    "cdac_common_centroid_map",
    "cdac_groups",
    "cdac_pins",
    "clocked_dynamic_preamp",
    "mimcap_size_for_target",
    "preamp_devices",
    "sar_bootstrapped_switch",
    "sar_cdac_array",
    "sar_cdac_switch_matrix",
    "sar_sampling_frontend",
    "sar_top_plate_reset",
    "strongarm_comparator",
    "strongarm_offset_trim",
    "switch_matrix_pins",
    "trim_devices",
    "trim_pins",
]
