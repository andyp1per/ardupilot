# RP2350B Chip Analysis

## RP2350B Key Specifications

| Feature | RP2350B | STM32H753 (Aeolus) | ESP32 |
|---|---|---|---|
| **Core** | 2x Cortex-M33 @ 150MHz | 1x Cortex-M7 @ 480MHz | 2x Xtensa LX6 @ 240MHz |
| **FPU** | Single-precision | Single+Double precision | Single-precision |
| **SRAM** | 520KB (10 banks) | 1MB | 520KB + 4MB PSRAM |
| **Flash** | External QSPI up to 16MB | 2MB internal | External up to 16MB |
| **GPIO** | 48 (B variant) | 140+ (176-pin) | 34 |
| **UART** | 2 hardware | 8 (UART/USART) | 3 |
| **SPI** | 2 hardware | 6 | 4 |
| **I2C** | 2 hardware | 4 | 2 |
| **ADC** | 8 ext channels (B), 12-bit | 20+ channels, 16-bit | 18 channels, 12-bit |
| **PWM** | 24 channels (12 slices) | Many timer channels | 16 channels |
| **CAN** | None | 2x FDCAN | None |
| **Ethernet** | None | Yes (MII) | None |
| **USB** | 1.1 host/device | 2.0 OTG HS | None (via UART) |
| **DMA** | 16 channels | 32 channels | Yes |
| **Special** | 3x PIO (12 state machines) | - | - |
| **Security** | TrustZone, SecureBoot, OTP | - | Flash encryption |
| **Package** | QFN-80 | LQFP-176 | QFN-48 |

## Capability Assessment for ArduPilot

### Sufficient

- **PWM output:** 24 channels is more than enough for typical use (12-16 servos)
- **SPI:** 2 hardware SPI is sufficient for 2-3 IMUs with chip selects
- **I2C:** 2 hardware I2C buses, plus PIO can add more
- **GPIO:** 48 pins on B variant provides adequate pin count
- **USB:** 1.1 device mode sufficient for MAVLink console
- **SRAM:** 520KB is tight but workable for a minimal build (comparable to STM32F4)
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

| Limitation | Impact | Mitigation |
|---|---|---|
| **No double-precision FPU** | Reduced EKF precision | Use `HAL_WITH_EKF_DOUBLE 0`, same as ESP32 |
| **150MHz clock** | Lower throughput than H7 | Dual-core, optimize hot paths, reduce loop rate |
| **4 ADC channels** | Limited analog sensing | Prioritize: battery V, battery I, RSSI, spare |
| **External flash** | Slower code execution | XIP cache helps; place hot code in SRAM |
| **No CAN hardware** | CAN requires PIO or external IC | PIO CAN (proven on RP2040) or MCP2515 via SPI |
| **No Ethernet** | No Ethernet MAVLink | Not required for most use cases |
| **520KB SRAM** | Must disable many features | Minimal build like ESP32; AP_Periph fits easily |
| **2 HW UARTs only** | Need PIO for additional | Well-proven PIO UART implementations exist |

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

RAM is the binding constraint. A minimal vehicle build should fit within 520KB
with careful feature selection, but margins are tight. Copter with its more
demanding attitude control loop may require further optimization.

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
