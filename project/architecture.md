# HAL Architecture Design

## Approach: AP_HAL_Pico (ChibiOS-Based, Separate HAL)

This port creates a new HAL backend `AP_HAL_Pico` that uses ChibiOS RT kernel
and the RP2350 LLD drivers underneath, but keeps ArduPilot HAL wrapper code
separate from AP_HAL_ChibiOS.

See [approach-comparison.md](approach-comparison.md) for the detailed pros/cons
analysis of this approach vs extending AP_HAL_ChibiOS.

### Rationale

1. **ChibiOS has production-ready RP2350 support** - complete LLD drivers for
   ADC, DMA, GPIO, I2C, PWM, SPI, UART, USB, WDG, Timer, Flash, plus full
   SMP dual-core support. No need for Pico SDK or FreeRTOS.
2. **PIO is unique to RP2040/RP2350** and has no ChibiOS abstraction. PIO-based
   UART, SBUS, DShot, and CAN need clean integration that doesn't fit the
   STM32 model baked into AP_HAL_ChibiOS.
3. **RCOutput is fundamentally different** - STM32 timer-DMA DShot (2500+ lines)
   shares zero code with RP2350 PWM slices + PIO DShot. Putting both in one
   file with #ifdefs would be unmaintainable.
4. **hwdef simplicity** - RP2350's "any pin, any function" model doesn't need
   the complex STM32 alternate-function tables, DMA topology encoding, or
   timer channel grouping that chibios_hwdef.py encodes.
5. **Rapid development** - can iterate freely without risk of breaking existing
   STM32 boards. No cross-platform testing needed for each change.

### What We Get from ChibiOS

| ChibiOS Component | Used By AP_HAL_Pico | Notes |
|---|---|---|
| RT Kernel | Scheduler, Semaphores | Threads, mutexes, virtual timers, events |
| SMP Port (ARMv8-M-ML) | Dual-core support | Inter-core FIFO, spinlocks, core affinity |
| PAL (GPIO) LLD | GPIO driver | palSetLine, palReadLine, palSetLineCallback |
| SIO (UART) LLD | UARTDriver (HW) | sioStart, async read/write |
| SPI LLD | SPIDevice | spiStart, spiExchange with DMA |
| I2C LLD | I2CDevice | i2cStart, i2cMasterTransmit/Receive |
| ADC LLD | AnalogIn | adcStart, adcConvert with DMA |
| PWM LLD | RCOutput (basic PWM) | pwmStart, pwmEnableChannel |
| USB LLD | UARTDriver (USB CDC) | USB device with SerialUSB driver |
| EFL (Flash) LLD | Storage | Flash erase/program for parameters |
| WDG LLD | Watchdog | wdgStart, wdgReset |
| ST (Timer) LLD | System tick | chSysInit timing source |
| DMA | SPI, ADC, UART | Shared DMA channel allocation |

### What AP_HAL_Pico Adds Beyond ChibiOS

| Component | Implementation | Notes |
|---|---|---|
| PIO runtime | Thin driver using ChibiOS register defs | Program load, SM config, FIFO, DMA, IRQ |
| PIO UART | Custom RP2350 PIO programs | Additional serial ports beyond 2 HW UARTs |
| PIO SBUS/PPM | Custom PIO program | RC receiver input |
| PIO DShot | Custom PIO program | ESC protocol output |
| PIO WS2812 | Custom PIO program | LED strip driver |
| PIO CAN (optional) | Custom PIO program | Or MCP2515 SPI-to-CAN |
| hwdef parser | pico_hwdef.py | RP2350-specific pin mux model |
| Flash storage | XIP-aware wear leveling | QSPI flash parameter storage |

### PIO Support: Pico SDK Integration

ChibiOS has **no PIO driver**, but its `rp2350.h` CMSIS header provides the
complete PIO register definitions (`PIO_TypeDef` struct with `CTRL`, `FSTAT`,
`TXF[]`, `RXF[]`, `INSTR_MEM[32]`, `SM[4].{CLKDIV,EXECCTRL,SHIFTCTRL,PINCTRL}`,
`GPIOBASE`, IRQ registers) and the `PIO0`/`PIO1`/`PIO2` base address macros.

However, the raw registers are not enough. We need:

**1. `pioasm` — PIO assembler (build-time tool)**

`pioasm` compiles `.pio` assembly files into C headers containing instruction
arrays (`uint16_t[]`) and configuration helper functions. It's a standalone
C++ host tool from the Pico SDK (`tools/pioasm/`) that runs on the build
machine, not on the target. It has no runtime dependencies.

Options:
- **Use pioasm from Pico SDK** — simplest, already proven. BSD-3-Clause
  license is compatible. Just needs to be built as a host tool during the
  waf build (similar to how ArduPilot builds other host tools).
- **Pre-assemble** — run pioasm offline, check in the generated `.h` files.
  Simpler build but less maintainable.
- **Hand-encode** — write instruction arrays directly in C (no `.pio` files).
  The instruction encoding is documented and simple (16-bit words), but
  error-prone and hard to read.

Recommended: Use `pioasm` from Pico SDK as a build-time dependency.

**2. PIO runtime API — program loading, SM configuration, FIFO access**

The Pico SDK's `hardware_pio` library (~470 lines of C, ~2000 lines of header)
provides functions like `pio_add_program()`, `pio_sm_init()`,
`pio_sm_set_config()`, `sm_config_set_clkdiv()`, etc. These manipulate the
PIO registers that ChibiOS already defines.

Options:
- **Use Pico SDK `hardware_pio` directly** — include `pio.c` and `pio.h`
  in the build alongside ChibiOS. The runtime code is small (~470 LOC) and
  its register access uses the same memory-mapped addresses as ChibiOS.
  Main dependency: `hardware/claim.h` (resource claiming, ~100 LOC) and
  `hardware/gpio.h` (for `pio_gpio_init()` to set pin function select).
  The GPIO function-select could conflict with ChibiOS PAL — needs a thin
  compatibility shim.
- **Write our own PIO driver** — ~300-400 lines using ChibiOS register defs
  directly. Covers the subset we actually need: program loading, SM config,
  clock divider, pin mapping, FIFO read/write, DMA setup, IRQ enable.
  Avoids any Pico SDK dependency but means more code to write and maintain.
- **Hybrid** — use Pico SDK's `pio_instructions.h` (instruction encoding
  helpers, header-only, no dependencies) plus a custom thin driver for
  SM management using ChibiOS register defs.

Recommended: **Hybrid approach**. Use Pico SDK for:
- `pioasm` (build tool, no runtime impact)
- `pio_instructions.h` (header-only instruction encoding, BSD-3-Clause)
- Pre-assembled PIO program headers (output of pioasm)

Write our own thin PIO runtime (~300 LOC) for:
- Program loading into instruction memory
- State machine configuration (clkdiv, pin mapping, shift, wrap)
- FIFO access and DMA setup (using ChibiOS DMA driver)
- SM enable/disable and IRQ management

This minimizes Pico SDK coupling while leveraging its tools where they
genuinely help. The PIO register layout is the same regardless of whether
ChibiOS or Pico SDK defines the structs.

**3. GPIO function select for PIO pins**

PIO pins need their IO bank function set to PIO0/PIO1/PIO2 (function values
6/7/8 in the IO bank CTRL register). ChibiOS PAL doesn't expose this — it
maps GPIO to SPI/I2C/UART functions but not PIO. We need a thin helper:

```c
// Set GPIO pin to PIO function (6=PIO0, 7=PIO1, 8=PIO2)
static inline void pio_gpio_init(uint pio_index, uint gpio) {
    // IO_BANK0->GPIO[gpio].CTRL = function_select
    volatile uint32_t *ctrl = &IO_BANK0->GPIO[gpio].CTRL;
    *ctrl = (*ctrl & ~0x1f) | (6 + pio_index);
}
```

This is trivial and doesn't need Pico SDK.

## Directory Structure

```
libraries/AP_HAL_Pico/
    AP_HAL_Pico.h                # Main HAL include
    AP_HAL_Pico_Namespace.h      # Namespace declarations
    HAL_Pico_Class.h/.cpp        # HAL class implementation

    # Core drivers (wrapping ChibiOS APIs)
    Scheduler.h/.cpp             # ChibiOS RT thread/timer scheduler
    Semaphores.h/.cpp            # ChibiOS mutex/semaphore wrappers
    UARTDriver.h/.cpp            # HW UART (ChibiOS SIO) + PIO UART + USB
    SPIDevice.h/.cpp             # ChibiOS SPI + DMA
    I2CDevice.h/.cpp             # ChibiOS I2C
    GPIO.h/.cpp                  # ChibiOS PAL
    RCInput.h/.cpp               # PIO-based RC input
    RCOutput.h/.cpp              # ChibiOS PWM + PIO DShot
    AnalogIn.h/.cpp              # ChibiOS ADC
    Storage.h/.cpp               # Flash-based parameter storage
    Util.h/.cpp                  # Utility functions
    Flash.h                      # Flash operations (ChibiOS EFL)
    shared_dma.h/.cpp            # DMA channel arbitration

    # PIO support (RP2350-specific, no ChibiOS equivalent)
    pio/
        pio_manager.h/.cpp       # PIO runtime: program load, SM config, FIFO, DMA
        pio_defs.h               # Register field defines, instruction encoding
                                 # (from Pico SDK pio_instructions.h or hand-written)
        programs/
            uart_tx.pio           # PIO UART transmit (pioasm source)
            uart_rx.pio           # PIO UART receive
            sbus.pio              # PIO SBUS (inverted UART) receive
            dshot.pio             # PIO DShot output
            ws2812.pio            # PIO WS2812 LED driver
        generated/                # pioasm output (checked in or build-generated)
            uart_tx.pio.h         # Pre-assembled instruction arrays
            uart_rx.pio.h
            sbus.pio.h
            dshot.pio.h
            ws2812.pio.h

    # Board configuration
    hwdef/
        scripts/
            pico_hwdef.py        # hwdef.dat processor (generates hwdef.h)
        pico2/
            hwdef.dat            # Raspberry Pi Pico 2 dev board
        <custom-board>/
            hwdef.dat            # Custom flight controller board

    # ChibiOS configuration templates
    cfg/
        mcuconf.h                # MCU peripheral enables, clock config
        halconf.h                # ChibiOS HAL feature enables
        chconf.h                 # ChibiOS RT kernel config

    system.cpp                   # System initialization, fault handlers
```

## Board ID and Registration

### AP_HAL_Boards.h

```cpp
#define HAL_BOARD_PICO     14    // Next available ID after QURT (13)
```

### Board Header (AP_HAL/board/pico.h)

```cpp
#pragma once
#include <hwdef.h>

#define HAL_BOARD_NAME "Pico"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_300    // 520KB SRAM

#define HAL_HAVE_HARDWARE_DOUBLE 0
#define HAL_WITH_EKF_DOUBLE 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_WITH_IO_MCU 0
#define HAL_NUM_CAN_IFACES 0              // Initially; PIO CAN later
#define HAL_WITH_DRONECAN 0               // Initially; enable for AP_Periph
#define HAL_STORAGE_SIZE (16384)           // 16KB in flash

#define O_CLOEXEC 0
#define HAL_OS_POSIX_IO 0
#define HAL_OS_SOCKETS 0
#define HAL_OS_FATFS_IO 0

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 4096
#endif

#ifdef __cplusplus
#include <AP_HAL_Pico/Semaphores.h>
#define HAL_Semaphore Pico::Semaphore
#define HAL_BinarySemaphore Pico::BinarySemaphore
#endif

#define NUM_SERVO_CHANNELS 16
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0
#define HAL_GYROFFT_ENABLED 0

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN
```

## Threading Model

ArduPilot requires multiple concurrent execution contexts. ChibiOS RT provides
native thread support with SMP across both RP2350 cores.

| ArduPilot Thread | ChibiOS Thread | Core | Priority | Stack |
|---|---|---|---|---|
| Main loop (setup/loop) | Main thread | Core 0 | NORMALPRIO | 8KB |
| Timer thread | Timer thread | Core 0 | HIGHPRIO-2 | 4KB |
| UART I/O | UART thread | Core 1 | NORMALPRIO+1 | 4KB |
| I/O thread | IO thread | Core 1 | NORMALPRIO | 4KB |
| Storage | Storage thread | Core 1 | LOWPRIO+1 | 2KB |

**Dual-core strategy:**
- **Core 0:** Main ArduPilot loop, timer callbacks, sensor reads
- **Core 1:** I/O processing (UART, storage writes, USB) via ChibiOS SMP

ChibiOS SMP on RP2350 uses the inter-processor FIFO for cross-core
scheduling. Each core runs its own ChibiOS instance (`ch0`, `ch1`) with
shared ready lists.

The demo in `ChibiOS.svn/demos/RP/RT-RP2350-PICO2/` confirms this works:
core 0 runs the main thread, core 1 runs the shell and timer via
`chSysWaitSystemState()` / `chInstanceObjectInit()`.

## Scheduler Design

The Scheduler class wraps ChibiOS RT thread management:

```
Scheduler::init()
    ├── Create timer thread (chThdCreateStatic, HIGHPRIO-2, core 0)
    ├── Create UART thread (chThdCreateStatic, NORMALPRIO+1, core 1)
    ├── Create I/O thread (chThdCreateStatic, NORMALPRIO, core 1)
    └── Create storage thread (chThdCreateStatic, LOWPRIO+1, core 1)

Scheduler::register_timer_process(proc)
    └── Add to timer callback list (called from timer thread)

Scheduler::delay(ms)
    └── chThdSleepMilliseconds(ms)

Scheduler::delay_microseconds(us)
    └── chThdSleepMicroseconds(us)  [or busy-wait for <100us]

Scheduler::in_main_thread()
    └── chThdGetSelfX() == main_thread_ptr
```

## UART Architecture

With only 2 hardware UARTs, additional serial ports use PIO:

| Serial Port | Assignment | Implementation |
|---|---|---|
| SERIAL0 (console) | USB CDC | ChibiOS USB + SerialUSB driver |
| SERIAL1 (telem1) | HW UART0 | ChibiOS SIO driver (sioStart) |
| SERIAL2 (telem2) | PIO UART | Custom PIO program + DMA |
| SERIAL3 (GPS1) | HW UART1 | ChibiOS SIO driver |
| SERIAL4 (GPS2) | PIO UART | Custom PIO program + DMA |
| SERIAL5 (RC) | PIO SBUS/CRSF | Custom PIO program |

The UARTDriver class abstracts over all three backends:

```cpp
class UARTDriver : public AP_HAL::UARTDriver {
    enum uart_type { HW_SIO, PIO_UART, USB_CDC };
    uart_type _type;
    union {
        SIODriver *sio;         // ChibiOS SIO driver for HW UARTs
        struct {                // PIO UART state
            PIO pio;
            uint sm_tx, sm_rx;
            uint dma_tx, dma_rx;
        } pio_uart;
        SerialUSBDriver *usb;   // ChibiOS USB serial
    } _dev;
};
```

## SPI Architecture

Two hardware SPI buses managed by ChibiOS SPI driver with DMA:

| SPI Bus | Typical Use | ChibiOS Driver |
|---|---|---|
| SPI0 | IMU1, IMU2 (shared bus, separate CS) | SPID0 |
| SPI1 | Barometer, FRAM, or additional IMU | SPID1 |

```cpp
class SPIDevice : public AP_HAL::SPIDevice {
    SPIDriver *_spi_drv;       // ChibiOS SPI driver (SPID0 or SPID1)
    ioline_t _cs_line;         // ChibiOS PAL line for chip select
    SPIConfig _spi_cfg;        // ChibiOS SPI configuration
    mutex_t _bus_mutex;        // Bus arbitration
};
```

ChibiOS SPI LLD on RP2350 supports DMA transfers (SPI_SUPPORTS_CIRCULAR = FALSE,
but standard DMA exchange works). Clock frequency is derived from RP_PERI_CLK.

**DMA note:** ChibiOS provides a complete DMAv1 driver for RP2350 with 16 channels
(`os/hal/ports/RP/LLD/DMAv1/`). DMA is already integrated into the SPI and ADC
LLD drivers - AP_HAL_Pico does not need to manage DMA directly for these
peripherals. The RP2350 DMA architecture differs from STM32 (no streams/channels
hierarchy, simpler request routing), but this is handled inside the ChibiOS LLD.
See [developer-concerns.md](developer-concerns.md) section 5 for details.

## Storage Architecture

Parameter storage uses ChibiOS EFL (Embedded Flash) driver:

```
Flash Layout (external QSPI, accessed via XIP at 0x10000000):
  0x10000000 - 0x103FFFFF : Program code (XIP, up to 4MB)
  0x10400000 - 0x10403FFF : Parameter storage (16KB)
  0x10404000 - 0x10FFFFFF : Available (logging, etc.)
```

The ChibiOS EFL driver for RP2350 handles:
- QMI (Quad-Mode Interface) flash access
- 256-byte page writes, 4K sector erases
- XIP region management (flash reads don't conflict with code execution
  as long as writes target different sectors)

Flash writes require both cores to be coordinated. ChibiOS handles this
via the inter-processor FIFO mechanism.

## Build System Integration

### Waf Integration

Following the ChibiOS pattern (similar to `chibios.py`):

1. **`Tools/ardupilotwaf/boards.py`** - Add `pico` board class
2. **`Tools/ardupilotwaf/pico.py`** - Waf tool for Pico:
   - Runs `pico_hwdef.py` to generate `hwdef.h`
   - Sets up ChibiOS build (RT kernel + RP2350 HAL + LLD drivers)
   - Compiles PIO programs (pioasm assembler)
   - Links ArduPilot + ChibiOS + PIO into final ELF
   - Generates UF2 via picotool
3. **Toolchain** - `arm-none-eabi-gcc` (same as ChibiOS STM32 builds)

### ChibiOS Submodule

AP_HAL_Pico uses the same ChibiOS source that AP_HAL_ChibiOS uses, but
references the RP2350-specific makefiles:

```makefile
# From ChibiOS RP2350 demo, adapted for ArduPilot:
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/RP/RP2350/platform.mk
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk  # or custom board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML/compilers/GCC/mk/port_rp2.mk
```

### Build Flow

```
./waf configure --board pico2
    ├── Process hwdef.dat → hwdef.h, mcuconf.h, halconf.h
    ├── Locate ChibiOS submodule
    ├── Set up arm-none-eabi-gcc toolchain
    └── Configure PIO assembler (pioasm)

./waf AP_Periph  (or copter, plane, rover)
    ├── Compile ChibiOS RT + RP2350 HAL/LLD → libchibios.a
    ├── Assemble PIO programs → pio_programs.o
    ├── Compile AP_HAL_Pico drivers → libhal_pico.a
    ├── Compile ArduPilot libraries + vehicle → libardupilot.a
    ├── Link all → ardupilot.elf
    └── Generate ardupilot.uf2 (via picotool or elf2uf2)
```

### UF2 Output

The RP2350B supports UF2 firmware format for drag-and-drop flashing via
USB bootloader (BOOTSEL mode). The build produces both `.elf` and `.uf2`.

The ChibiOS demo Makefile shows the pattern:
```makefile
PICOTOOL ?= picotool
$(BUILDDIR)/$(PROJECT).uf2: $(BUILDDIR)/$(PROJECT).elf
    $(PICOTOOL) uf2 convert $< $@
```

## hwdef.dat Format

RP2350-specific configuration, parsed by `pico_hwdef.py`. Simpler than
STM32 hwdef because RP2350 has flexible pin muxing (any GPIO can be
routed to any peripheral function via IO bank function select).

```
# Example hwdef.dat for an RP2350B flight controller
MCU RP2350B
OSCILLATOR_HZ 12000000
FLASH_SIZE_KB 4096

# Serial port order
SERIAL_ORDER OTG1 UART0 PIO_UART0 UART1 PIO_UART1

# UART0 - Telem1 (ChibiOS SIO)
GP0  UART0_TX  UART0
GP1  UART0_RX  UART0

# UART1 - GPS1 (ChibiOS SIO)
GP4  UART1_TX  UART1
GP5  UART1_RX  UART1

# PIO UART0 - Telem2
GP8  PIO_UART0_TX  PIO0
GP9  PIO_UART0_RX  PIO0

# PIO UART1 - GPS2
GP10 PIO_UART1_TX  PIO0
GP11 PIO_UART1_RX  PIO0

# SPI0 - IMU bus
GP16 SPI0_MISO  SPI0
GP17 IMU1_CS    OUTPUT
GP18 SPI0_SCK   SPI0
GP19 SPI0_MOSI  SPI0
GP20 IMU2_CS    OUTPUT

# I2C0 - Sensor bus (Baro, Compass)
GP12 I2C0_SDA   I2C0
GP13 I2C0_SCL   I2C0

# PWM outputs
GP2  PWM1  PWM1A
GP3  PWM2  PWM1B
GP6  PWM3  PWM3A
GP7  PWM4  PWM3B

# ADC
GP26 BATT_VOLTAGE_SENS  ADC0
GP27 BATT_CURRENT_SENS  ADC1
GP28 RSSI_ADC_PIN       ADC2

# Misc
GP25 LED_ACTIVITY  OUTPUT

# IMU declarations
SPIDEV imu1  SPI0 DEVID1 IMU1_CS MODE3 2*MHZ 8*MHZ
SPIDEV imu2  SPI0 DEVID2 IMU2_CS MODE3 2*MHZ 8*MHZ
IMU Invensensev3 SPI:imu1 ROTATION_NONE
IMU Invensensev3 SPI:imu2 ROTATION_NONE

# Barometer
BARO ICP201XX I2C:0:0x63

# Compass
COMPASS RM3100 I2C:0:0x20
```

### hwdef Simplifications vs STM32

| Aspect | STM32 hwdef | Pico hwdef |
|---|---|---|
| Pin function | Alternate function number (AF0-AF15) | IO bank function select (simple enum) |
| DMA mapping | Explicit stream/channel per peripheral | Auto-allocated from 16 channels |
| Timer groups | Complex shared-counter groupings | Not needed (PWM slices are independent) |
| Clock tree | Per-family APB/AHB bus assignments | Single PERI_CLK for all peripherals |
| Pin constraints | Each pin has fixed AF options | Any GPIO can be any function |
