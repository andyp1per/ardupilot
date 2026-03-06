# RP2350B Chip Analysis

## RP2350B Key Specifications

| Feature | RP2350B | STM32F405 (MatekF405) | STM32H757 (high-end ref) |
|---|---|---|---|
| **Core** | 2x Cortex-M33 @ 150MHz | 1x Cortex-M4F @ 168MHz | Cortex-M7 @ 480MHz + M4 @ 240MHz |
| **FPU** | Single-precision | Single-precision | Single+Double precision |
| **SRAM** | 520KB (10 banks) | 192KB (128+64) | 1MB |
| **Flash** | External QSPI up to 16MB | 1MB internal | 2MB internal |
| **GPIO** | 48 (B variant) | ~80 (LQFP-64) | 140+ (176-pin) |
| **UART** | 2 hardware (+PIO) | 6 (UART/USART) | 8 (UART/USART) |
| **SPI** | 2 hardware | 3 | 6 |
| **I2C** | 2 hardware (+PIO) | 3 | 4 |
| **ADC** | 8 ext channels (B), 12-bit | 16 channels, 12-bit | 20+ channels, 16-bit |
| **PWM** | 24 channels (12 slices) | 6 typical (timer-limited) | Many timer channels |
| **CAN** | None (+PIO possible) | 2x bxCAN | 2x FDCAN |
| **Ethernet** | None | None | Yes (MII) |
| **USB** | 1.1 host/device | 2.0 OTG FS | 2.0 OTG HS |
| **DMA** | 16 channels | 16 streams (2 controllers) | 32 channels |
| **Special** | 3x PIO (12 state machines) | - | - |
| **Security** | TrustZone, SecureBoot, OTP | - | - |
| **Package** | QFN-80 | LQFP-64 | LQFP-176 |
| **Price** | ~$0.80 | ~$6-10 | ~$15-20 |

The most relevant comparison is the **STM32F405**, which is the same class of
resource-constrained flight controller that the RP2350B will compete with. The
MatekF405 runs ArduCopter, ArduPlane, and Rover with feature limitations
similar to what we expect on RP2350. The H757 is shown for reference as a
high-end dual-core target — not what RP2350 competes with.

## Capability Assessment for ArduPilot

### Sufficient

- **PWM output:** 24 channels is more than enough for typical use (12-16 servos)
- **SPI:** 2 hardware SPI is sufficient for 2-3 IMUs with chip selects
- **I2C:** 2 hardware I2C buses, plus PIO can add more
- **GPIO:** 48 pins on B variant provides adequate pin count
- **USB:** 1.1 device mode sufficient for MAVLink console
- **SRAM:** 520KB is nearly 3x the F405's 192KB — more headroom than expected
- **Flash:** 16MB external QSPI is ample for program storage
- **Clock:** 150MHz Cortex-M33 with DSP extensions provides reasonable compute
- **Dual-core:** Second core can handle sensor sampling or I/O

### Requires PIO Solutions

The RP2350B's PIO (Programmable I/O) state machines are critical for bridging
hardware peripheral gaps. Each PIO block has 4 state machines, and there are
3 blocks (12 total state machines).

| Protocol | Hardware | PIO Required | PIO SMs Needed |
|---|---|---|---|
| Additional UARTs | 2 HW UART | Yes, need 3-5 more | 2 SMs each (TX+RX) |
| CAN bus | None | Yes, or external MCP2515 | 2 SMs per bus |
| SBUS/PPM RC input | None | Yes | 1 SM |
| DShot output | None | Yes | 1-2 SMs |
| WS2812 LEDs | None | Yes | 1 SM |
| SD card (SPI mode) | Via HW SPI | Optional | 0 (use HW SPI) |

**PIO budget estimate (12 available):**
- 3 extra UARTs (GPS, Telem1, Telem2): 6 SMs
- RC input (SBUS/PPM): 1 SM
- DShot output: 1 SM
- WS2812 LEDs: 1 SM
- CAN bus (if not using external controller): 2 SMs
- **Total: 11 SMs** (tight but feasible)

### Limitations

| Limitation | Impact | vs F405 | Mitigation |
|---|---|---|---|
| **No double-precision FPU** | Reduced EKF precision | Same — F405 is also single-precision | Use `HAL_WITH_EKF_DOUBLE 0`, same as F4/ESP32 |
| **150MHz clock** | Lower throughput per core | Comparable — F405 is 168MHz single-core | Dual-core compensates; net compute is higher |
| **8 ADC channels** | Limited analog sensing | F405 has 16; but MatekF405 uses only 3 | Sufficient: battery V, battery I, RSSI |
| **External flash** | Slower code execution | F405 has 1MB internal | XIP cache helps; place hot code in SRAM |
| **No CAN hardware** | CAN requires PIO or external IC | F405 has 2x bxCAN | PIO CAN (proven on RP2040) or MCP2515 via SPI |
| **520KB SRAM** | Tight but workable | F405 has only 192KB — RP2350 has 2.7x more | Actually an advantage over F4 boards |
| **2 HW UARTs only** | Need PIO for additional | F405 has 6 | PIO UARTs add 3-4 more; proven implementations exist |
| **1MB flash** | Less program space | F405 has 1MB — identical | 16MB QSPI available; flash is not the constraint |

## Memory Budget Estimate

### AP_Periph (Initial Target)

| Component | Flash (est.) | RAM (est.) |
|---|---|---|
| Core AP_Periph | ~200KB | ~60KB |
| DroneCAN stack | ~80KB | ~20KB |
| GPS driver | ~40KB | ~10KB |
| Barometer driver | ~20KB | ~5KB |
| Compass driver | ~20KB | ~5KB |
| HAL drivers | ~40KB | ~30KB |
| Pico SDK + FreeRTOS | ~60KB | ~20KB |
| **Total** | **~460KB** | **~150KB** |
| **Available** | **16MB** | **520KB** |

### Minimal Vehicle (ArduPlane or Rover)

| Component | Flash (est.) | RAM (est.) |
|---|---|---|
| Vehicle code | ~600KB | ~100KB |
| Libraries (minimal) | ~400KB | ~150KB |
| EKF3 (single core) | ~100KB | ~80KB |
| HAL + Pico SDK | ~100KB | ~50KB |
| **Total** | **~1.2MB** | **~380KB** |
| **Available** | **16MB** | **520KB** |

For context, the **MatekF405 runs ArduCopter in 192KB SRAM** with aggressive
feature disabling. The RP2350B has 520KB — nearly 3x more — so RAM is less
of a constraint than initially assumed. The binding constraint is more likely
flash execution speed (XIP vs internal) and single-precision FPU performance,
both of which the F405 also suffers from.

## ChibiOS RP2350 Support

ChibiOS trunk now includes production-ready RP2350 support. This is the
foundation for AP_HAL_Pico (see [architecture.md](architecture.md)).

### ChibiOS LLD Driver Status (All COMPLETE)

| Driver | ChibiOS LLD | DMA | Notes |
|---|---|---|---|
| ADC | ADCv1 | YES | Single + round-robin, temp sensor, FIFO |
| DMA | DMAv1 | N/A | 16 channels with allocation/interrupt management |
| GPIO | GPIOv1 (PAL) | NO | 48 GPIO pins, event callbacks, multicore-aware |
| I2C | I2Cv1 | NO | Master TX/RX with timeout, 10-bit addressing |
| PWM | PWMv1 | NO | 12 slices, 2 channels each, period change support |
| SPI | SPIv1 | YES | DMA transfers, polled fallback |
| UART | UARTv1 (SIO) | NO | Modern SIO interface, full event support |
| USB | USBv1 | NO | Device mode, 15 endpoints, VBUS detection |
| Watchdog | WDGv1 | NO | Persistent scratch registers |
| Timer | TIMERv1 | NO | SysTick + TIMER0, 4 alarms, multicore-aware |
| Flash | EFL (RP2350) | NO | QMI QSPI, 256B pages, 4K sectors, XIP support |

### ChibiOS SMP Dual-Core

- Full SMP port for ARMv8-M-ML (Cortex-M33)
- Spinlock #31 for inter-core synchronization
- Inter-processor FIFO for scheduling messages
- Core-local memory sections (SCRATCH_X at 0x20080000, SCRATCH_Y at 0x20081000)
- Proven in demo: `demos/RP/RT-RP2350-PICO2/` in [ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn)

### What ChibiOS Does NOT Provide

- **PIO driver** - No ChibiOS abstraction for RP2350's Programmable I/O
- **CAN driver** - RP2350 has no hardware CAN; PIO CAN or external MCP2515 needed
- **SD/MMC** - No SDMMC peripheral; must use SPI mode via SPIv1

## PIO Deep Dive

PIO state machines are the key differentiator that makes the RP2350B viable
despite limited hardware peripherals. Each state machine:

- Runs independently at up to system clock (150MHz)
- Has its own instruction memory (32 instructions per PIO block, shared)
- Can bit-bang protocols with cycle-accurate timing
- Has TX/RX FIFOs (4-8 words deep) for DMA-driven data transfer
- Can generate interrupts

### Proven PIO Implementations (from RP2040 ecosystem)

- **UART TX/RX** - Part of official Pico SDK examples
- **SPI master/slave** - Official examples
- **I2C** - Official examples
- **CAN 2.0B** - Community implementations (e.g., Kevin O'Connor's canbus PIO)
- **DShot** - Community implementations
- **WS2812** - Official examples
- **SBUS** - Inverted UART at 100kbaud, trivial with PIO
- **PPM** - Pulse timing, simple PIO program
