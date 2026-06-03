# SP Racing H7 NF AIO Flight Controller

The SPRacingH7NFAIO is an all-in-one flight controller produced by
[Seriously Pro Racing](http://www.seriouslypro.com), featuring an STM32H743
processor, dual IMUs, and a 4Gbit SPI NAND flash used for onboard logging via
the LittleFS filesystem.

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - Two IMUs (ICM42605 / ICM42688P)
 - BMP388 or DPS310 barometer
 - Optional onboard compass (auto-probed)
 - MAX7456 OSD
 - 512MByte (4Gbit) MT29F SPI NAND flash for LittleFS logging
 - 6 UARTs
 - 10 PWM / DShot outputs plus a serial LED output
 - Analog battery voltage, current and RSSI inputs
 - Switchable VTX power output

## Pinout

*Pinout images not yet available.*

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB (MAVLink2)
 - SERIAL1 -> USART1 (RC input, DMA-enabled)
 - SERIAL2 -> USART2 (MAVLink2)
 - SERIAL3 -> USART3 (GPS)
 - SERIAL4 -> UART4 (GPS)
 - SERIAL5 -> UART5 (ESC telemetry, no DMA)
 - SERIAL8 -> UART8 (no DMA)

## RC Input

RC input is configured by default on USART1. All unidirectional RC protocols are
supported on the RX1 pin. For protocols requiring a TX line (CRSF/ELRS telemetry,
SRXL2) the TX1 pin is also used.

 - For CRSF/ELRS connect to RX1 and TX1.
 - PPM is not supported.

## OSD Support

The SPRacingH7NFAIO supports OSD using OSD_TYPE 1 (MAX7456 driver). DisplayPort
HD OSD is supported on any spare UART (UART4 is the HD VTX connector).

## PWM Output

The board supports up to 10 PWM / DShot outputs plus a serial LED output. The
outputs are in 4 groups:

 - PWM 1-4   in group1 (TIM5)
 - PWM 5-8   in group2 (TIM4)
 - PWM 9-10  in group3 (TIM3)
 - PWM 11    in group4 (TIM15), default serial LED

Channels within the same group must use the same output rate / protocol. If any
channel in a group uses DShot then all channels in that group must use DShot.
Channels 1-10 support bi-directional DShot.

## Battery Monitoring

The board has a built-in voltage and current sensor.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 12.5

The voltage and current multipliers above are starting values and should be
calibrated against an external meter.

## RSSI Input

Analog RSSI uses :ref:`RSSI_PIN<RSSI_PIN>` 12.

## Compass

The SPRacingH7NFAIO may have an onboard compass on I2C1 (auto-probed). You can
also attach an external compass using I2C on the SDA and SCL pads.

## Logging

Logging is stored on the onboard 4Gbit MT29F SPI NAND flash using the LittleFS
filesystem; no microSD card is required.

## VTX Power Control

GPIO 81 controls the VTX power output. By default RELAY2 is configured to drive
this pin and sets it high (VTX powered) at boot.

## Loading Firmware

Firmware for this board can be found at https://firmware.ardupilot.org in
sub-folders labeled "SPRacingH7NFAIO".

Initial firmware load can be done with DFU by plugging in USB with the bootloader
button pressed. Then you should load the "with_bl.hex" firmware, using your
favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any
ArduPilot ground station software. Updates should be done with the *.apj firmware
files.
