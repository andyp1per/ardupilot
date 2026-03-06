# ArduPilot RP2350B Port - Project Plan

## Overview

This project aims to port ArduPilot to the Raspberry Pi RP2350B microcontroller,
creating a new HAL backend (`AP_HAL_Pico`) that uses ChibiOS RT kernel and
RP2350 LLD drivers underneath, but keeps ArduPilot HAL wrapper code separate
from the STM32-focused `AP_HAL_ChibiOS`.

The RP2350B is a dual-core ARM Cortex-M33 running at 150MHz with 520KB SRAM,
48 GPIO pins, and unique PIO (Programmable I/O) capabilities.

## Target Hardware

- **Chip:** RP2350B (QFN-80, 48 GPIO variant)
- **RTOS:** ChibiOS RT with SMP dual-core support
- **Primary target board:** Laurel (Betaflight: HELLBENDER_0001) — RP2350B FC
  with ICM-42688P, DPS310, SD card, 4 motor outputs, PIO UARTs, CRSF RC
- **Development board:** Raspberry Pi Pico 2 (for early bringup)
- **Reference designs:** Holybro Aeolus (HF-A001-RC01), ABSI Flight Controller

## Key Design Decisions

**Separate HAL (`AP_HAL_Pico`) using ChibiOS underneath**, rather than extending
`AP_HAL_ChibiOS` with RP2350 #ifdefs. This gives clean code, rapid development,
and first-class PIO support while still leveraging ChibiOS's production-ready
RP2350 drivers. See [approach-comparison.md](approach-comparison.md) for the
detailed analysis.

**Incremental mergeable PRs**, not one giant branch. Each step (ChibiOS update,
HAL skeleton, each peripheral driver, board support) is a self-contained PR.
After Step 2, peripheral PRs are almost entirely new files — low risk, easy to
review. See [implementation-plan.md](implementation-plan.md).

## Documents

| Document | Description |
|---|---|
| [approach-comparison.md](approach-comparison.md) | Pros/cons of separate HAL vs extending AP_HAL_ChibiOS |
| [rp2350b-chip-analysis.md](rp2350b-chip-analysis.md) | RP2350B capabilities, ChibiOS driver status |
| [architecture.md](architecture.md) | HAL architecture and key design decisions |
| [implementation-plan.md](implementation-plan.md) | Step-by-step plan structured as mergeable PRs |
| [reference-boards.md](reference-boards.md) | Analysis of Holybro and ABSI reference schematics |
| [risks.md](risks.md) | Technical risks and mitigation strategies |
| [developer-concerns.md](developer-concerns.md) | Detailed analysis of specific technical concerns (GCC, DMA, dual-core, performance, features) |

## Quick Reference

### ArduPilot HAL Interfaces Required

Each HAL port must implement these core interfaces (defined in `libraries/AP_HAL/`):

- **UARTDriver** - Serial communication
- **SPIDeviceManager** - SPI bus management
- **I2CDeviceManager** - I2C bus management
- **GPIO** - General purpose I/O
- **RCInput** - RC receiver input
- **RCOutput** - PWM/DShot servo/ESC output
- **AnalogIn** - ADC reading
- **Storage** - Persistent parameter storage
- **Scheduler** - Task scheduling and threading
- **Util** - Utility functions
- **Semaphores** - Thread synchronization
- **Flash** - Flash memory operations
- **OpticalFlow** - (optional) Optical flow sensor
- **DSP** - (optional) Digital signal processing
- **CANIface** - (optional) CAN bus interface

### Existing Ports for Reference

- `libraries/AP_HAL_ChibiOS/` - ChibiOS HAL for STM32 (same RTOS, different MCU)
- `libraries/AP_HAL_ESP32/` - ESP32 port (precedent for separate HAL approach)

### Key Files to Create/Modify

**New:**
- `libraries/AP_HAL_Pico/` - Entire new HAL backend
- `libraries/AP_HAL/board/pico.h` - Board header
- `Tools/ardupilotwaf/pico.py` - Waf build tool

**Modify:**
- `libraries/AP_HAL/AP_HAL_Boards.h` - Add `HAL_BOARD_PICO = 14`
- `Tools/ardupilotwaf/boards.py` - Add `pico` board class
- `Tools/scripts/build_options.py` - Feature flags for Pico

### ChibiOS RP2350 Source

ChibiOS RP2350 support is in [ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn) (trunk). Key paths:
- `os/hal/ports/RP/RP2350/` - Platform HAL (clocks, PLLs, flash)
- `os/hal/ports/RP/LLD/` - Low-level drivers (ADC, SPI, I2C, UART, PWM, etc.)
- `os/common/ports/ARMv8-M-ML/smp/rp2/` - Dual-core SMP port
- `os/hal/boards/RP_PICO2_RP2350/` - Pico 2 board definition
- `demos/RP/RT-RP2350-PICO2/` - Working dual-core demo
