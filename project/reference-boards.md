# Reference Board Analysis

## MatekF405 (STM32F405, Primary Comparison)

**MCU:** STM32F405RG (LQFP-64, Cortex-M4F @ 168MHz, 1MB flash, 192KB SRAM)
**Status:** Shipping, widely used, runs ArduCopter/Plane/Rover

The MatekF405 is the most relevant comparison for the RP2350B port — it's a
resource-constrained F4 board that runs full ArduPilot vehicles. The RP2350B
has similar clock speed (150 vs 168MHz), similar FPU (single-precision), but
significantly more RAM (520KB vs 192KB) and dual-core.

### MatekF405 Key Specs

| Resource | MatekF405 | RP2350B (Laurel) | Comparison |
|---|---|---|---|
| Clock | 168MHz Cortex-M4F | 150MHz Cortex-M33 x2 | Similar per-core; RP2350 has 2 cores |
| SRAM | 192KB | 520KB | RP2350 has 2.7x more |
| Flash | 1MB internal | No internal; 16MB external QSPI (XIP) | F405 advantage: internal flash execution |
| SPI buses | 3 (IMU, OSD, SD) | 2 (IMU, SD/OSD) | Similar |
| I2C buses | 1 | 2 | RP2350 has more |
| UARTs | 5 + USB | 2 HW + PIO UARTs + USB | Similar total with PIO |
| PWM outputs | 6 | 4 (PWM) + DShot via PIO | Similar |
| ADC | 3 used (V, I, RSSI) | 3 used (V, I, RSSI) | Identical |
| CAN | 2x bxCAN (unused on MatekF405) | None (PIO or MCP2515) | F405 advantage |
| OSD | MAX7456 on SPI2 | MAX7456 on SPI1 | Identical |
| SD card | SPI mode on SPI3 | SPI mode on SPI1 | Identical |
| Storage | 15KB flash | 16KB flash | Similar |
| IMU | MPU6000 on SPI1 | ICM-42688P on SPI0 | Both single SPI IMU |
| Baro | BMP280 on I2C (optional) | DPS310 on I2C | Both I2C baro |
| Price | ~$25-35 (board) | ~$0.80 (MCU) | RP2350 MCU far cheaper |

---

## Laurel / HELLBENDER_0001 (Primary Target)

**Source:** Betaflight `src/config/configs/HELLBENDER_0001/config.h`
**MCU:** RP2350B (QFN-80, 48 GPIO)
**Manufacturer ID:** RASP
**Status:** Flying in Betaflight (DShot, CRSF, ICM-42688P, DPS310)

This is the primary target board for the ArduPilot RP2350 port. Betaflight
already has a working configuration, giving us a verified pin mapping.

### Sensors and Peripherals

| Component | Type | Interface | GPIO Pins |
|---|---|---|---|
| ICM-42688P | 6-axis IMU | SPI0 | SCK=2, MOSI=3, MISO=4, CS=1, INT=22 |
| DPS310 | Barometer | I2C0 | SDA=44, SCL=45 |
| W25Q128FV | 16MB Flash | QSPI | CS=0 (QMI handles QSPI pins) |
| SD Card | Storage | SPI1 | SCK=26, MOSI=27, MISO=24, CS=25 |
| MAX7456 | OSD (optional) | SPI1 | CS=17 (mutually exclusive with SD) |

### Serial Ports

| Port | Type | GPIO | Purpose |
|---|---|---|---|
| UART0 | Hardware SIO | TX=12, RX=13 | VTX / MSP Displayport |
| UART1 | Hardware SIO | TX=8, RX=9 | GPS |
| PIO UART0 | PIO | TX=20, RX=21 | CRSF RC input |
| PIO UART1 | PIO | TX=34, RX=35 | Aux serial |
| UART5 RX | ? | RX=36 | SBUS RX (alternate RC) |
| UART6 RX | ? | RX=37 | ESC sensor telemetry |

### Motor / PWM Outputs

| Output | GPIO | Notes |
|---|---|---|
| Motor 1 | GP28 | DShot via PIO0 in Betaflight |
| Motor 2 | GP29 | DShot via PIO0 |
| Motor 3 | GP30 | DShot via PIO0 |
| Motor 4 | GP31 | DShot via PIO0 |

### Other Peripherals

| Function | GPIO | Notes |
|---|---|---|
| LED0 | GP6 | Status LED |
| LED1 | GP7 | Status LED |
| LED Strip | GP38 | WS2812 via PIO2 in Betaflight |
| Beeper | GP5 | PWM at 1971Hz |
| VBAT ADC | GP40 | Battery voltage |
| Current ADC | GP41 | Battery current |
| RSSI ADC | GP42 | RSSI |
| 5V BEC Enable | GP14 | Power control output |
| 9V BEC Enable | GP15 | Power control output |

### PIO Allocation (from Betaflight)

| PIO Block | Use | State Machines |
|---|---|---|
| PIO0 | DShot (4 motors) | 4 SMs |
| PIO1 | PIO UARTs (2 ports) | 4 SMs (2 TX + 2 RX) |
| PIO2 | LED Strip (WS2812) | 1 SM |

### Key Differences from ArduPilot's Approach

Betaflight uses **Pico SDK + bare metal** (no RTOS). ArduPilot will use
**ChibiOS RT** underneath AP_HAL_Pico. This means:
- Different startup code (ChibiOS `halInit()`/`chSysInit()` vs Pico SDK `runtime_init`)
- Different threading model (ChibiOS threads vs Betaflight's cooperative scheduler)
- Different DMA management (ChibiOS DMAv1 vs Pico SDK `hardware_dma`)
- Same PIO programs can be reused (PIO is hardware, programs are ISA-level)
  though licensing requires clean-room reimplementation

---

## Mapping to RP2350B Design

### What Can Be Replicated

| Feature | RP2350B Approach | Notes |
|---|---|---|
| Triple IMU (SPI) | 2 HW SPI + shared bus | CS-per-device, 2-3 IMUs feasible |
| Dual barometer (I2C) | 2 HW I2C | One baro per bus |
| Dual GPS (UART) | 1 HW UART + 1 PIO UART | Sufficient |
| Telemetry (UART) | 1 HW UART or PIO UART | With flow control via GPIO |
| PWM output | HW PWM (up to 24 ch) | Native, excellent support |
| Battery monitoring | 4 ADC channels | Matches available channels |
| USB console | TinyUSB CDC | Native USB 1.1 |
| Status LEDs | GPIO | Simple |
| SD card | SPI mode via HW SPI | Not SDMMC, but functional |
| FRAM | SPI via HW SPI | Same approach |

### What Cannot Be Directly Replicated (vs MatekF405)

| Feature | RP2350B Limitation | vs MatekF405 (F405) | Alternative |
|---|---|---|---|
| CAN bus | No CAN hardware | F405 has 2x bxCAN (unused on MatekF405) | PIO CAN or MCP2515 via SPI |
| 6 HW UARTs | Only 2 HW UART | F405 has 6; MatekF405 uses 5 | PIO UARTs (3-4 additional) |
| Internal flash | No internal flash — all code via QSPI XIP | F405 has 1MB internal flash | ArduPilot H750 XIP model (`EXT_FLASH_SIZE_MB`); 16KB XIP cache + SRAM hot paths |

### RP2350B Advantages over MatekF405

| Feature | RP2350B | MatekF405 | Benefit |
|---|---|---|---|
| SRAM | 520KB | 192KB | 2.7x more — room for more features |
| Cores | 2 (SMP) | 1 | I/O offloading, higher effective throughput |
| PIO | 12 state machines | None | Flexible protocol support without CPU |
| Flash storage | 16MB QSPI | 1MB internal | Ample program + log space |
| I2C buses | 2 (+PIO) | 1 used | More sensor buses |
| Price (MCU) | ~$0.80 | ~$6-10 | 8-12x cheaper |

### Recommended RP2350B Flight Controller Minimum Viable Peripherals

Based on the MatekF405 and Laurel board designs, a minimal RP2350B flight
controller should have:

**SHALL (Required):**
- 1x ICM-45686 IMU on SPI
- 1x BMP390 or ICP-20100 barometer on I2C
- 1x GPS UART
- 1x Telemetry UART
- 8x PWM outputs
- Battery voltage + current sense (2 ADC)
- USB-C console
- Status LED
- Parameter storage (FRAM or flash)

**SHOULD (Highly Recommended):**
- 2nd IMU on SPI (shared bus, separate CS)
- 2nd barometer on I2C
- RC input (SBUS via PIO)
- 1x CAN bus (PIO or MCP2515)
- SD card for logging (SPI mode)

**MAY (Optional):**
- 3rd IMU
- 2nd GPS
- 2nd CAN bus
- Compass (if external GPS module doesn't include one)
- Airspeed sensor (I2C)
