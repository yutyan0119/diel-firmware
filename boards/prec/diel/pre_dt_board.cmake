# The nRF52840 devicetree shares unit addresses between power/clock and acl/flash-controller.
# Suppress that known dtc warning, as ZMK's own nRF52840 boards do.
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
