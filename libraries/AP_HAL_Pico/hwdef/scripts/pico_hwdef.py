"""
hwdef parser for AP_HAL_Pico (RP2350).

PicoHWDef subclasses ChibiOSHWDef so RP2350 boards have a parser that
lives alongside the Pico HAL backend, rather than in AP_HAL_ChibiOS.
Today this is a no-op pass-through — the seam exists so that future
RP2350-specific parser logic can land here as virtual-method overrides
without touching the ChibiOS base parser.
"""

from chibios_hwdef import ChibiOSHWDef


class PicoHWDef(ChibiOSHWDef):
    pass
