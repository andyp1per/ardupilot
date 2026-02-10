# wgdrones-dr001 Flight Controller

The wgdrones-dr001 is a flight controller produced by [WG Drones](URL_HERE), featuring an STM32H743 processor, dual IMUs for redundancy, and 6 UARTs with dual CAN bus support.

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - ICM42688 and BMI088 dual IMUs
 - DPS310 barometer
 - microSD card slot
 - 6x UARTs (2 with flow control)
 - 2x CAN bus
 - 6x PWM outputs (4 support bi-directional DShot)
 - Safety switch input
 - Dedicated Spektrum RC input
 - Analog RSSI input
 - Battery voltage and current monitoring

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> USART2 (MAVLink2, with flow control)
 - SERIAL2 -> USART3 (MAVLink2, with flow control)
 - SERIAL3 -> UART4 (GPS)
 - SERIAL4 -> UART8
 - SERIAL5 -> UART7
 - SERIAL6 -> USART6

## RC Input

RC input is configured via a dedicated Spektrum connector (SERIAL_RC binding
support). RC could also be applied to any other available UART by setting its
protocol to receive RC data, for example :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` = 23.

For RC protocols other than unidirectional, both TX and RX pins of the UART will
need to be connected:

 - :ref:`SERIALx_PROTOCOL<SERIALx_PROTOCOL>` should be set to "23".
 - FPort would require :ref:`SERIALx_OPTIONS<SERIALx_OPTIONS>` be set to "15".
 - CRSF would require :ref:`SERIALx_OPTIONS<SERIALx_OPTIONS>` be set to "0".
 - SRXL2 would require :ref:`SERIALx_OPTIONS<SERIALx_OPTIONS>` be set to "4" and connects only the TX pin.

## PWM Output

The wgdrones-dr001 supports up to 6 PWM or DShot outputs.

The PWM is in 2 groups:

 - PWM 1-4 in group1 (TIM1)
 - PWM 5-6 in group2 (TIM4)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-4 support bi-directional DShot.

## Battery Monitoring

The board has a built-in voltage and current sensor input.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 14
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 15
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 10.1
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 17.0

## Analog RSSI input

Analog RSSI uses :ref:`RSSI_ANA_PIN<RSSI_ANA_PIN>` 11

## CAN

The wgdrones-dr001 has two CAN ports for DroneCAN peripherals such as GPS, compass, airspeed, and rangefinder.

## Compass

The wgdrones-dr001 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Safety Switch

The board has a dedicated safety switch input.

## Loading Firmware

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`__ in sub-folders labeled "wgdrones-dr001".

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
\*.apj firmware files.
