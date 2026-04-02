# ABSI Aerospace & Defense Flight Controller

The ABSI FC is a professional-grade flight controller designed by ABSI Aerospace & Defense, featuring an STM32H743 processor, dual ICM-45686 IMUs for redundancy, and comprehensive interfaces including dual CAN buses. The system is designed for integration with the ABSI Power Distribution Board via board-to-board connectors.

## Features

 - MCU: STM32H743VIT6 32-bit processor running at 480 MHz (100-pin LQFP)
 - 2MB Flash, 1MB RAM
 - Two ICM-45686 6-axis IMUs (SPI1 and SPI4)
 - BMP390 barometer (I2C, address 0x76)
 - microSD card slot for logging (SDMMC2, 4-bit mode) with card detect
 - 8x UARTs (including USB)
 - 2x CAN ports with TJA1462 transceivers
 - 3x I2C ports with 4.7k pull-ups
 - 16x PWM outputs (8 motor + 8 aux)
 - Dual battery monitoring inputs
 - 5V rail voltage monitoring
 - GPIO-controlled power outputs (VTX, 5V-2A rail, SPST relay)
 - External 8MHz and 32.768kHz crystals
 - USB-C interface via external breakout board with DFU boot button
 - Designed for integration with ABSI Power Distribution Board

## Connectors

All external connectors use JST-GH (1.25mm pitch) or JST-SH (1.0mm pitch) series:

| Connector | Type | Function |
|-----------|------|----------|
| J4 | SM04B-GHS-TB (4-pin JST-GH) | GPS + Compass |
| J5 | SM04B-GHS-TB (4-pin JST-GH) | I2C |
| J6 | SM06B-GHS-TB (6-pin JST-GH) | UART (USART1) |
| J7 | SM06B-GHS-TB (6-pin JST-GH) | Telemetry (with CTS/RTS) |
| J8 | SM04B-GHS-TB (4-pin JST-GH) | CAN |
| J9 | SM08B-SRSS-TB (8-pin JST-SH) | 4-in-1 ESC |
| J3 | SM06B-SRSS-TB (6-pin JST-SH) | USB-C Breakout Interface |

## UART Mapping

The UARTs are marked Rn and Tn in the pinout. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (RC Input, on J6)
 - SERIAL2 -> USART2 (ESC Telemetry, on J9)
 - SERIAL3 -> USART3 (Spare, solder pads)
 - SERIAL4 -> UART4 (Spare, solder pads)
 - SERIAL5 -> UART5 (Spare, solder pads)
 - SERIAL7 -> UART7 (GPS, DMA-enabled)
 - SERIAL8 -> UART8 (Telemetry with flow control, on J7)

### Telemetry Connector (J7) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | 5V | 5V Telemetry Power |
| 2 | TX | UART8_TX (SERIAL8) |
| 3 | RX | UART8_RX (SERIAL8) |
| 4 | CTS | UART8_CTS (flow control) |
| 5 | RTS | UART8_RTS (flow control) |
| 6 | GND | Ground |

### GPS Connector (J4) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | 5V | 5V Telemetry Power |
| 2 | TX | UART7_TX (GPS_TX) |
| 3 | RX | UART7_RX (GPS_RX) |
| 4 | SCL | I2C1_SCL (for compass) |
| 5 | SDA | I2C1_SDA (for compass) |
| 6 | GND | Ground |

### 4-in-1 ESC Connector (J9) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | ESC_BATTERY | Battery voltage sense from ESC |
| 2 | GND | Ground |
| 3 | ESC_CURRENT | Current sense (PC5, ADC1_INP8) |
| 4 | ESC_TELEMETRY | USART2_RX (ESC telemetry, SERIAL2) |
| 5 | M1 | Motor 1 signal |
| 6 | M2 | Motor 2 signal |
| 7 | M3 | Motor 3 signal |
| 8 | M4 | Motor 4 signal |

### CAN Connector (J8) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | 5V | 5V Telemetry Power |
| 2 | CAN_H | CAN High |
| 3 | CAN_L | CAN Low |
| 4 | GND | Ground |

## RC Input

RC input is configured on USART1 (SERIAL1) by default. The board supports all ArduPilot compatible RC protocols including:

 - SBUS (directly connected, no inverter needed for most receivers)
 - DSM/DSM2/DSMX
 - CRSF/ELRS (bidirectional, connect to TX and RX pins)
 - FPort (bidirectional)
 - SRXL2

For bidirectional protocols like CRSF or FPort, connect to the SERIAL1 TX/RX pins:

 - :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` should be set to "23"
 - CRSF/ELRS: :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` = 0
 - FPort: :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` = 15
 - SRXL2: :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` = 4 (TX pin only)

## PWM Output

The ABSI FC supports up to 16 PWM or DShot outputs. PWM outputs are active low and accent to V_Servo (5V or 6V from PDB). The PWM outputs are organized in groups by timer:

 - PWM 1-4 in group1 (TIM1) - Motor outputs, bi-directional DShot capable
 - PWM 5-8 in group2 (TIM8) - Motor outputs, bi-directional DShot capable
 - PWM 9-11 in group3 (TIM3) - Aux outputs, DShot capable
 - PWM 12-13 in group4 (TIM4) - Aux outputs, DShot capable
 - PWM 14-16 in group5 (TIM2) - Aux outputs, PWM only

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot. Channels 1-13 support DShot, channels 1-8 support bi-directional DShot for RPM telemetry.

### PWM Connector Pinout (3-pin headers)

Each PWM output has a 3-pin header with:
 - Pin 1: Signal
 - Pin 2: V_Servo (5V/6V)
 - Pin 3: GND

## Battery Monitoring

The board has built-in voltage and current monitoring. Battery sensing is provided through the Power Distribution Board interface.

### Battery 1 (Primary)

Voltage divider: 105k/10k (11.5:1 ratio)
Current sensor: INA186A2 with 500uOhm shunt resistor

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.5
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40.0

### Battery 2 (Secondary, via PDB)

The ICD allocates PC2/PC3 for Battery 2 voltage and current sensing. However, these pins are ADC3-only inputs which are not available on the 100-pin LQFP package (STM32H743VIT6). For second battery monitoring, use a DroneCAN battery monitor on CAN2 instead:

 - :ref:`BATT2_MONITOR<BATT2_MONITOR>` = 8 (DroneCAN)

### Current Sensing Specifications

 - Shunt resistor: 500uOhm, 10W, 1%
 - Maximum continuous current: 90A
 - Maximum burst current: 120A
 - Current sense amplifier: INA186A2 (gain = 50V/V)

## CAN

The ABSI FC has two CAN ports using TJA1462ATK transceivers:

 - **CAN1 (FDCAN1)**: External DroneCAN port for GPS, airspeed sensors, compass, and other peripherals. Active by default.
 - **CAN2 (FDCAN2)**: Internal interface to the ABSI Power Distribution Board via board-to-board connector.

Both CAN ports include 60.4 Ohm termination resistors and 47nF common-mode filter capacitors. The flight controller is not expected to be a central node; termination can be removed by the user if necessary.

CAN1 includes a standby control pin (active high) for power management.

## I2C

Three I2C buses are available:

 - **I2C4** (bus 0): Internal sensors (barometer BMP390 at 0x76)
 - **I2C1** (bus 1): External I2C on GPS and standalone I2C connectors (for compass)
 - **I2C2** (bus 2): PDB interface for expansion

All I2C buses have 4.7k Ohm pull-up resistors installed.

## Compass

The ABSI FC does not have a builtin compass, but you can attach an external compass using:

 - I2C on the GPS connector (I2C1, bus 1)
 - CAN-connected compass via DroneCAN

## Barometer

The board includes a BMP390 barometer connected via I2C4 (bus 0) at address 0x76.

## IMU

Two ICM-45686 6-axis IMUs provide redundant inertial measurement:

 - **IMU1**: Connected to SPI1
 - **IMU2**: Connected to SPI4

Both IMUs have independent interrupt lines for high-rate sampling.

## GPIO and Relay Control

The board provides GPIO-controlled power outputs accessible via the RELAYn parameters:

| GPIO | Function | Default State | Control |
|------|----------|---------------|---------|
| GPIO 81 | VTX power enable (EN_VTX) | LOW (off) | RELAY1 |
| GPIO 82 | 5V-2A rail enable (EN_5V-2A) | LOW (off) | RELAY2 |
| GPIO 83 | SPST relay enable (EN_SPST) | LOW (off) | RELAY3 |

These can be controlled via:
 - MAVLink relay commands
 - Lua scripts
 - RC channel passthrough

Example Lua script to enable VTX power:
```lua
gpio:write(81, 1)  -- Enable VTX power
```

## Power Distribution Board Interface

The ABSI FC is designed to integrate with the ABSI Power Distribution Board via board-to-board connectors (Samtec FTS/FLE series):

### Power Connector (J1 - FTS-125-01-L-DV, 50-pin)

| Rail | Voltage | Current | Description |
|------|---------|---------|-------------|
| V_Servo | 5V or 6V | 8A | Servo/motor power, always on |
| 5V_Telemetry | 5V | 3A | Telemetry and peripherals, always on |
| V_VTX | 9V or 12V | 3A | VTX power, relay controlled |
| 3V3_MCU | 3.3V | 1A | MCU and sensors, always on |

### Signal Connector (J2)

| Signal | MCU Pin | Description |
|--------|---------|-------------|
| EN_VTX | PD10 | VTX rail enable (GPIO 81) |
| EN_V-ADD | PD11 | 5V-2A rail enable (GPIO 82) |
| EN_SPST | PA8 | SPST relay enable (GPIO 83) |
| BATTERY1_V | PC0 | Battery 1 voltage sense |
| BATTERY1_I | PC1 | Battery 1 current sense |
| BATTERY2_V | PC2 | Battery 2 voltage sense (ADC3, see note) |
| BATTERY2_I | PC3 | Battery 2 current sense (ADC3, see note) |
| I2C_SDA | PB11 | I2C2 data (PDB expansion) |
| I2C_SCL | PB10 | I2C2 clock (PDB expansion) |
| FDCAN2_TX | PB13 | CAN2 transmit |
| FDCAN2_RX | PB12 | CAN2 receive |
| ESC_BATTERY | N/A | 4-in-1 ESC battery sense |

### PDB Power Specifications

The Power Distribution Board accepts 6V-36V input (2S-8S LiPo) with the following protections:

 - TVS diode: SMBJ36CA-13-F (36V standoff, 58.1V clamping @ 10.4A)
 - Reverse polarity protection available via optional ideal diode circuit

### PDB Output Relay

The PDB includes a solid-state relay (TLP241A) controllable via EN_SPST:
 - Maximum voltage: 30V
 - Maximum current: 2A
 - Can switch V_Servo, 5V_Telemetry, or V_VTX via jumper selection

## USB-C Breakout Board

The USB-C interface is provided via an external breakout board connected through a 6" cable (JST-SH 6-pin).

### USB-C Breakout Features

 - USB-C connector (UJ40-C-H-G-L1-SMT-TR)
 - 3.3V LDO regulator (TLV75733PDBVR) for MCU power when USB-powered
 - DFU boot button (TL3365AF180QG tactile switch)
 - Green power LED
 - ESD and reverse current protection

### USB-C Breakout Connector (J2) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | 3V3 | 3.3V output from LDO |
| 2 | DFU_BOOT | Boot button signal (active high) |
| 3 | V_BUS | USB 5V (protected) |
| 4 | USB_N | USB D- |
| 5 | USB_P | USB D+ |
| 6 | GND | Ground |

When powered via USB-C only:
 - 3V3_MCU rail: 1A maximum
 - 5V_Telemetry rail: 1A maximum (via V_BUS)

## LED Indicators

The board has three status LEDs on dedicated GPIO pins:

| LED | Color | GPIO | Function |
|-----|-------|------|----------|
| D1 | Blue | PE15 | Activity/Status |
| D2 | Green | PE12 | Armed status |
| D3 | Amber | PD3 | Auxiliary indicator |

## Debug Interface

SWD debug pads are provided for firmware development:

 - SWCLK (PA14)
 - SWDIO (PA13)
 - 3V3
 - GND

## Loading Firmware

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`__ in sub-folders labeled "ABSIFC".

### Initial Firmware Load (DFU Mode)

1. Connect the USB-C breakout board to your computer
2. Press and hold the DFU boot button on the USB-C breakout board
3. While holding the button, connect USB power (or press reset if already powered)
4. Release the button after 1 second
5. The board should enumerate as a DFU device
6. Load the "with_bl.hex" firmware using your favorite DFU tool:
   - dfu-util (Linux/macOS): `dfu-util -a 0 -D ABSIFC_with_bl.hex`
   - STM32CubeProgrammer (Windows/Linux/macOS)
   - Zadig may be needed on Windows to install WinUSB driver

### Firmware Updates

Once the initial firmware is loaded, you can update the firmware using any ArduPilot ground station software:

 - Mission Planner
 - QGroundControl
 - MAVProxy

Updates should be done with the *.apj firmware files.

## Mechanical

### Board-to-Board Stack Height

The FTS/FLE connector pair provides a 6mm mated stack height between the flight controller and power distribution board.

### Mounting Hardware

 - 4x M2x6mm standoffs
 - 8x M2x4mm screws

## Schematic Reference

The hardware was designed by Pelican Engineering (Lafayette, LA) for ABSI Aerospace & Defense. Schematic documents:

 - SCH-ABSI-Flight_Control-System.SchDoc
 - SCH-ABSI-Power_Distribution_2ESC-System.SchDoc
 - SCH-ABSI-USBC_Breakout-System.SchDoc
