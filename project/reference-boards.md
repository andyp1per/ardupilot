# Reference Board Analysis

## Holybro Aeolus (HF-A001-RC01)

**Schematic:** `HF-A001-RC01.pdf` (15 sheets)
**MCU:** STM32H753IIK (176-pin LQFP, Cortex-M7 @ 480MHz, 2MB flash, 1MB RAM)

### Sensor Suite

| Sensor | Type | Interface | Quantity | ArduPilot Driver |
|---|---|---|---|---|
| ICM-45686 | 6-axis IMU (accel+gyro) | SPI1, SPI2, SPI3 | 3 | AP_InertialSensor_Invensensev3 |
| ICP-20100 | Barometer | I2C2, I2C4 | 2 | AP_Baro_ICP201XX |
| MS4525 | Airspeed (differential) | I2C1 | 1 | AP_Airspeed_MS4525 |
| RM3100 | Magnetometer | I2C3 | 1 | AP_Compass_RM3100 |
| IST8310 | Magnetometer (not installed) | I2C3 | 0 | AP_Compass_IST8310 |

### Communication Interfaces

| Interface | STM32 Peripheral | External | Purpose |
|---|---|---|---|
| UART7 | USART7 | BST0014MN52G | Telem1 (with flow control) |
| UART5 | UART5 | BST0014MN52G | Telem2 (with flow control) |
| UART8 | UART8 | NEO-F9P | GPS2 |
| USART1 | USART1 | NEO-F9P | GPS1 |
| USART6 TX | USART6 | - | ELRS output |
| USART6 RX | USART6 | - | RC_IN (SBUS/DSM/PPM) |
| USART3 | USART3 | - | FMU Debug |
| CAN1 | FDCAN1 | TJA1051 | DroneCAN bus 1 |
| CAN2 | FDCAN2 | TJA1051 | DroneCAN bus 2 |
| ETH | ETH MII | PHY chip | Ethernet |
| USB | USB OTG | Type-C | Console/MAVLink |

### PWM Outputs

- 12 PWM channels via timer outputs
- Level shifting: VER0 = no shift (3.3V), VER1 = with shift (configurable voltage)
- Two groups of 8 driven by SN74LVC8T245BNR level shifters
- Additional 2 PWM through BSS214NW6 FETs

### Power Architecture

- Input: 10-52V via XT60-M connector
- Power MUX: LTC4417 with triple input (Brick1 > Brick2 > VBUS priority)
- Over-voltage: 5.6V, Under-voltage: 3.6V, Hysteresis: 30mV
- Voltage sensing: Two scales (18x and 35x dividers)
- Current sensing: 132A max (BATT_AMP_PERVLT = 40)
- Regulated outputs: 5V servo, 3.3V sensors, 5V peripherals

### Storage

- FRAM on SPI5 (fast parameter storage)
- SD NAND on SDMMC2 (logging)
- 24LC64 EEPROM on I2C (board ID / calibration)
- SE050 secure element on I2C (cryptographic operations)

### Key Design Patterns for RP2350B Port

1. **Triple-redundant IMUs** on separate SPI buses with individual chip selects
2. **Dual barometers** on separate I2C buses for redundancy
3. **Hardware flow control** on telemetry UARTs (CTS/RTS)
4. **Level-shifted PWM** to support 5V servo rail
5. **Dual CAN** with TJA1051 transceivers (high-side/low-side)
6. **Dual GPS** with separate UART connections + shared PPS/Event
7. **Battery monitoring** with separate voltage and current sense per battery

---

## ABSI Flight Controller (SCH-ABSI-Flight_Control)

**Schematic:** `SCH-ABSI-Flight_Control.PDF` (7 sheets)
**MCU:** STM32H743VIT6 (100-pin LQFP, Cortex-M7 @ 480MHz)
**Customer:** ABSI Aerospace & Defense
**Designer:** Pelican Engineering

### Sensor Suite

| Sensor | Type | Interface | Priority | Notes |
|---|---|---|---|---|
| ICM-45686 (U2) | 6-axis IMU | SPI (AP_SDO, AP_SDI, AP_CS) | SHALL | Primary IMU |
| ICM-45686 (U3) | 6-axis IMU | SPI (IMU2_MISO/MOSI/CS) | SHOULD | Redundant IMU |
| ICM-45686 (U7) | 6-axis IMU | SPI (IMU3_MISO/MOSI/CS) | MAY | Third IMU |
| BMP390 (U4) | Barometer | I2C1 (SENSOR_SCL/SDA) | SHALL | I2C addr 0x76 |
| RM3100 | Magnetometer | I2C | SHOULD | Draft removed from project |

### Communication Interfaces

| Interface | Purpose | Notes |
|---|---|---|
| UART7 + UART8 | GPS + Compass | TX, RX, I2C SDA/SCL |
| UART6 | Telemetry | TX, RX, CTS, RTS (flow control) |
| UART1, UART3, UART4, UART5 | Additional UARTs | General purpose |
| USART2 TX | Misc | Single direction |
| FDCAN1 | CAN bus 1 | TJA1462A with standby |
| FDCAN2 | CAN bus 2 | TJA1462A |
| USB OTG | Console | Via USB-C breakout board |

### PWM Outputs

- **16 PWM outputs** organized in groups:
  - TIM1: CH1-CH4 (4 channels)
  - TIM8: CH1-CH4 (4 channels)
  - TIM3: CH2 + TIM4: CH1-CH2 (3 channels)
  - TIM2: CH1 + TIM4: CH1-CH2 (3 channels)
  - TIM5: CH1-CH4 (last 2 for PWM13-16)

### Power Architecture

- PDB (Power Distribution Board) interface via FTS/FLE connectors
- Battery monitoring: 4x ADC inputs (BATTERY1_V, BATTERY1_I, BATTERY2_V, BATTERY2_I)
- 4-in-1 ESC interface: ESC_BATTERY, ESC_CURRENT, ESC_TELEMETRY
- Power rails: Servo (5V/6V, 15A), Telemetry (5V, 6A), VTX (9V/12V, 6A), MCU/Sensor (3.3V, 3A)

### Key Design Patterns

1. **SHALL/SHOULD/MAY priority system** for component population variants
2. **USB-C breakout PCB** as a separate sub-board (with DFU boot button)
3. **CAN standby mode** via TJA1462A (supports bus wake-up)
4. **Board-to-board mounting** with FTS/FLE connector system (5.85mm stack height)
5. **Extensive decoupling** on ADC inputs (100nF per channel)
6. **External oscillators** for both LSE (32.768kHz) and HSE (8MHz)

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

### What Cannot Be Directly Replicated

| Feature | Limitation | Alternative |
|---|---|---|
| Dual FDCAN | No CAN hardware | PIO CAN or MCP2515 via SPI |
| Ethernet | No Ethernet | Not available; use WiFi add-on |
| 4+ I2C buses | Only 2 HW I2C | PIO I2C for additional buses |
| 8 UARTs | Only 2 HW UART | PIO UARTs (3-4 additional) |
| DMA sophistication | 12 ch vs 32 ch | Adequate for this scale |
| Double-precision FPU | Single only | Software double; EKF impact |
| 480MHz clock | 150MHz | Dual-core compensates partially |
| 1MB+ SRAM | 520KB | Minimal feature set |

### Recommended RP2350B Flight Controller Minimum Viable Peripherals

Based on both reference designs, a minimal RP2350B flight controller should have:

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
