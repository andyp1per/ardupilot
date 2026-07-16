# FLOWTECH H7 Wing Flight Controller

The FLOWTECH H7 Wing is a flight controller produced by [FLOWTECH](URL_HERE), featuring an STM32H743 processor, dual ICM-45686 IMUs for redundancy, and a DroneCAN port. It is designed to stack with the FLOWTECH Power Distribution Board, which supplies all of its power rails and carries its battery monitoring.

## Features

 - MCU - STM32H743VIT6 32-bit processor running at 480 MHz (100-pin LQFP)
 - 2MB Flash, 1MB RAM
 - Two ICM-45686 IMUs (SPI1 and SPI4), each on a dedicated 3.3V regulator, both running fast and high-resolution sampling
 - BMP390 barometer
 - microSD card slot for logging (SDMMC2, 4-bit) with card detect
 - 9x UARTs (including two USB endpoints)
 - CAN support (FDCAN1) on two mirrored connectors, plus a second CAN to the Power Distribution Board
 - 3x I2C buses with 4.7k pull-ups
 - 16x PWM outputs (8 motor, 8 aux)
 - Battery voltage and current monitoring via the Power Distribution Board
 - 5V rail monitoring and a 4-in-1 ESC current input
 - GPIO-controlled power outputs (VTX rail, additional 5V rail, SPST relay)
 - External 8MHz and 32.768kHz crystals
 - USB-C via an external breakout board with DFU boot button
 - 6mm mated stack height to the Power Distribution Board

*Pinout images not yet available.*

## Connectors

| Connector | Type | Function |
|-----------|------|----------|
| J1 | FTS-125-01-L-DV (50-pin board-to-board) | Power Distribution Board power and signal |
| J2 | SM06B-SRSS-TB (6-pin JST-SH) | USB-C breakout board |
| J3 | SM04B-GHS-TB (4-pin JST-GH) | CAN |
| J4 | SM04B-GHS-TB (4-pin JST-GH) | CAN (mirrored with J3) |
| J5 | SM06B-GHS-TB (6-pin JST-GH) | GPS + Compass |
| J6 | SM06B-GHS-TB (6-pin JST-GH) | Telemetry (with CTS/RTS) |
| J7 | SM04B-GHS-TB (4-pin JST-GH) | I2C |
| J8 | SM08B-SRSS-TB (8-pin JST-SH) | 4-in-1 ESC |
| SKT1 | microSD socket | Logging |

The PWM outputs and the spare UARTs are brought out to 2.54mm through-holes and
solder pads rather than connectors.

## UART Mapping

 - SERIAL0 -> USB (MAVLink2)
 - SERIAL1 -> USART1 (RC Input, DMA-enabled, on the Tx1/Rx1 pads)
 - SERIAL2 -> USART2 (ESC Telemetry, RX on J8, TX on the Tx2 pad)
 - SERIAL3 -> USART3 (MAVLink2, on the Tx3/Rx3 pads)
 - SERIAL4 -> UART4 (Spare, on the Tx4/Rx4 pads)
 - SERIAL5 -> UART5 (Spare, on the Tx5/Rx5 pads, no DMA)
 - SERIAL7 -> UART7 (GPS, DMA-enabled, on J5)
 - SERIAL8 -> UART8 (MAVLink2 with CTS/RTS flow control, on J6, no DMA)
 - SERIAL9 -> USB (MAVLink2, second USB endpoint on the same USB-C connector)

Each pair of UART pads has an adjacent V-Telem and GND pad so a breakout can be
fitted locally.

### GPS + Compass Connector (J5)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | V-SEL1 | Selectable power, V-MCU (5V) by default |
| 2 | TX | UART7_TX |
| 3 | RX | UART7_RX |
| 4 | SCL | I2C1_SCL |
| 5 | SDA | I2C1_SDA |
| 6 | GND | Ground |

### Telemetry Connector (J6)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | V-SEL2 | Selectable power, V-MCU (5V) by default |
| 2 | TX | UART8_TX |
| 3 | RX | UART8_RX |
| 4 | CTS | UART8_CTS |
| 5 | RTS | UART8_RTS |
| 6 | GND | Ground |

### I2C Connector (J7)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | V-MCU | 5V |
| 2 | SCL | I2C1_SCL |
| 3 | SDA | I2C1_SDA |
| 4 | GND | Ground |

J7 shares I2C1 with the GPS connector.

### CAN Connectors (J3, J4)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | V-SEL3 | Selectable power, V-MCU (5V) by default |
| 2 | CAN_H | CAN High |
| 3 | CAN_L | CAN Low |
| 4 | GND | Ground |

### 4-in-1 ESC Connector (J8)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | ESC_BATTERY | Battery voltage from the ESC |
| 2 | GND | Ground |
| 3 | ESC_CURRENT | Current sense (PC5) |
| 4 | ESC_TELEMETRY | USART2_RX (SERIAL2) |
| 5 | M1 | Motor 1 signal |
| 6 | M2 | Motor 2 signal |
| 7 | M3 | Motor 3 signal |
| 8 | M4 | Motor 4 signal |

### Connector Power Selection

The power pin of the GPS (J5), Telemetry (J6) and CAN (J3/J4) connectors is fed
from a selection pad rather than a fixed rail. All three default to V-MCU, the
5V 3A rail from the Power Distribution Board. To move one to the V-Telem rail
instead, remove its V-MCU jumper resistor and bridge the V-Telem pad. Do not
bridge both pads of a selection rail at once.

## RC Input

The default RC input is configured on USART1 (SERIAL1) on the Tx1/Rx1 pads. RC could
be applied instead to a different UART port such as SERIAL3 or SERIAL4, setting the
protocol to receive RC data :ref:`SERIALn_PROTOCOL<SERIALn_PROTOCOL>` = 23 and changing
:ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` to something other than '23'. For RC protocols
other than unidirectional, the USART1_TX pin will need to be used:

 - :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` should be set to "23".
 - FPort would require :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` be set to "15".
 - CRSF would require :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` be set to "0".
 - SRXL2 would require :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` be set to "4" and connects only the TX pin.

## PWM Output

The FLOWTECH H7 Wing supports up to 16 PWM or DShot outputs on 3-pin 2.54mm
through-holes (P1 to P16). M1 to M4 are additionally present on the 4-in-1 ESC
connector (J8). Each 3-pin header is signal on pin 1, V-Servo on pin 2 and GND
on pin 3, with V-Servo supplied at 5V or 6V by the Power Distribution Board.

The PWM is in 5 groups:

 - PWM 1-4   in group1 (TIM1)
 - PWM 5-8   in group2 (TIM8)
 - PWM 9-11  in group3 (TIM3)
 - PWM 12-13 in group4 (TIM4)
 - PWM 14-16 in group5 (TIM2)

Channels within the same group need to use the same output rate. If any channel
in a group uses DShot then all channels in the group need to use DShot. Channels
1-13 support DShot, channels 1-8 support bi-directional DShot. Channels 14-16 are
PWM only.

## Battery Monitoring

Battery voltage and current sensing are provided by the Power Distribution Board
and reach the flight controller over the J1 board-to-board connector. The voltage
sensor covers the 6V-36V input range of the PDB (2S-8S).

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.5
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40.0

Voltage is divided 105k/10k on the PDB and buffered, giving the 11.5 multiplier.
Current is sensed with a 500uOhm 10W shunt and an INA186A2 at a gain of 50, which
gives 25mV/A and the 40.0 A/V scale. The shunt is rated for 90A continuous and
120A burst.

### Second Battery

PC2 and PC3 carry Battery 2 voltage and current over J1, but the Power Distribution
Board ships without the parts a second analog battery needs, so the inputs are left
disabled in the hwdef. Use a DroneCAN battery monitor instead:

 - :ref:`BATT2_MONITOR<BATT2_MONITOR>` = 8 (DroneCAN)

On a board that does populate them, uncomment the two ADC lines in the hwdef and set
:ref:`BATT2_MONITOR<BATT2_MONITOR>` = 4 with `BATT2_VOLT_PIN` 12 and `BATT2_CURR_PIN` 13.

## Analog inputs

Analog 5V rail monitoring uses pin 4, scaled 2:1 on the board.
The 4-in-1 ESC current input on J8 uses pin 8, and can be used as a second
analog current monitor with :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN__AP_BattMonitor_Analog>` = 8.

## CAN

FDCAN1 is the external DroneCAN port. It is brought out on two mirrored 4-pin
connectors (J3 and J4) so peripherals can be daisy-chained, and uses a TCAN3414
transceiver. GPIO 70 drives the transceiver standby pin and is held low (active)
by default.

The bus is terminated on the board with a 60.4 Ohm split termination and a 560pF
split capacitor, which suits the flight controller sitting at the end of the CAN
bus. If it needs to sit in the middle of a bus instead, remove the R17 jumper
resistor to lift the termination.

FDCAN2 is routed to the Power Distribution Board over J1. It has no transceiver on
the flight controller; the transceiver for that link lives on the PDB.

## I2C

Three I2C buses are available:

 - **I2C4** (bus 0): internal, barometer only
 - **I2C1** (bus 1): external, on the GPS (J5) and I2C (J7) connectors
 - **I2C2** (bus 2): Power Distribution Board interface over J1

All three have 4.7k pull-ups.

## Compass

The FLOWTECH H7 Wing does not have a builtin compass, but you can attach an
external compass using I2C on the GPS (J5) or I2C (J7) connectors, or a
DroneCAN compass on J3/J4.

## Barometer

The board has a BMP390 barometer on I2C4 at address 0x76.

## GPIO and Relay Control

The board provides three GPIO-controlled outputs on the Power Distribution Board:

| GPIO | Function | Default State | Control |
|------|----------|---------------|---------|
| GPIO 81 | VTX rail enable (EN_VTX) | LOW (off) | RELAY1 |
| GPIO 82 | Additional 5V rail enable (EN_V-ADD) | LOW (off) | RELAY2 |
| GPIO 83 | SPST relay enable (EN_SPST) | LOW (off) | RELAY3 |

These can be controlled via MAVLink relay commands, Lua scripts, or RC channel
passthrough.

## Power Distribution Board Interface

The flight controller stacks onto the Power Distribution Board through J1, a
50-pin FTS-125-01-L-DV carrying both power and signal. The FTS/FLE connector pair
gives a 6mm mated stack height.

### Rails supplied over J1

| Rail | Voltage | Current | Description |
|------|---------|---------|-------------|
| V-Servo | 5V or 6V | 8A | Servo and motor power, always on |
| V-Telem | 5V | 3A | Telemetry and peripherals, always on |
| V-VTX | 9V or 12V | 3A | VTX power, relay controlled |
| V-MCU | 5V | 3A | Flight controller and sensors, always on |

The flight controller regulates V-MCU down to 3.3V locally with three AP7361C-33
regulators, giving IMU1, IMU2 and the remaining sensors and MCU their own supplies.

### Signals carried over J1

| Signal | MCU Pin | Description |
|--------|---------|-------------|
| EN_VTX | PD10 | VTX rail enable (GPIO 81) |
| EN_V-ADD | PD11 | Additional 5V rail enable (GPIO 82) |
| EN_SPST | PA8 | SPST relay enable (GPIO 83) |
| BATTERY1_V | PC0 | Battery 1 voltage sense |
| BATTERY1_I | PC1 | Battery 1 current sense |
| BATTERY2_V | PC2 | Battery 2 voltage sense (see above) |
| BATTERY2_I | PC3 | Battery 2 current sense (see above) |
| I2C_SCL | PB10 | I2C2 clock |
| I2C_SDA | PB11 | I2C2 data |
| FDCAN2_TX | PB13 | CAN2 transmit |
| FDCAN2_RX | PB12 | CAN2 receive |
| ESC_BATTERY | N/A | 4-in-1 ESC battery sense |

### PDB Power Input

The Power Distribution Board accepts 6V-36V (2S-8S) and is protected by an
SMBJ36CA-13-F TVS diode (36V standoff, 58.1V clamping at 10.4A).

### PDB Output Relay

EN_SPST drives a DMN2056U-7 MOSFET switch on the PDB, rated 20V and 4A continuous.
Its input is selected with a 0 Ohm jumper from V-Servo, V-Telem or V-VTX through a
B320A-13-F Schottky diode, which limits the output to 3A.

## USB-C Breakout Board

The USB-C interface is provided by an external breakout board on a 6-pin JST-SH
cable. V-USB from the breakout feeds the V-MCU rail through a B320A-13-F Schottky
diode, so the board can be powered from USB alone. Each connector pin is rated 1A.

### USB-C Breakout Connector (J2) Pinout

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | V-USB | USB 5V |
| 2 | V-USB | USB 5V |
| 3 | USB_N | USB D- |
| 4 | USB_P | USB D+ |
| 5 | DFU_BOOT | Boot button signal, drives BOOT0 high for DFU |
| 6 | GND | Ground |

## LED Indicators

The board has three status LEDs on dedicated GPIO pins:

| LED | Color | Pin |
|-----|-------|-----|
| D3 | Blue | PE15 |
| D4 | Green | PE12 |
| D5 | Amber | PD3 |

## Debug Interface

SWD debug pads are provided:

 - SWCLK (PA14)
 - SWDIO (PA13)
 - 3V3
 - GND

## Loading Firmware

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`__ in sub-folders labeled "FLOWTECH_H7_WING".

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
