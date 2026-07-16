# FLOWTECH H7 Wing Hardware Review

This document contains recommendations for hardware improvements based on analysis of the schematics and pin assignments. The review focuses on the STM32H743VIT6 (100-pin LQFP package) pin constraints.

Reviewed against schematic revision A.1 (SCH-FLOWTECH-Flight_Control, SCH-FLOWTECH-Power_Distribution_2ESC) and ABSI-Flight_Controller-ICD-V3R7. Revision A.1 resolved several findings raised against the earlier ABSI-branded schematics; those are recorded below rather than deleted so the history is visible.

---

## Reference Documents

The following reference documents were used for this review:

### STMicroelectronics
- [STM32H743 Datasheet (DS12110)](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) - Electrical specifications, pin definitions, package information
- [STM32H743 Reference Manual (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) - Peripheral details, register descriptions
- [AN5096: STM32H7 Getting Started](https://www.st.com/resource/en/application_note/an5096-getting-started-with-stm32h7-series-hardware-development-stmicroelectronics.pdf) - Hardware design guidelines including power supply
- [AN2867: Oscillator Design Guide](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf) - Crystal selection and load capacitor calculations

### TDK InvenSense
- [ICM-45686 Datasheet](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf) - IMU specifications, power requirements, decoupling recommendations
- [IMU PCB Design and MEMS Assembly Guidelines (AN-000393)](https://invensense.tdk.com/wp-content/uploads/2024/03/AN-000393_TDK-InvenSense-IMU-PCB-Design-and-MEMS-Assembly-Guidelines.pdf) - Comprehensive PCB layout recommendations for IMUs

### Bosch Sensortec
- [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) - Barometer specifications

### Texas Instruments
- [TCAN3413/TCAN3414 Datasheet (SLLSFS8A)](https://www.ti.com/lit/ds/symlink/tcan3414.pdf) - CAN FD transceiver specifications, termination recommendations

### ECS International
- [ECS-80-8-33B2Q-CVY-TR3 Datasheet](https://ecsxtal.com/store/pdf/csm-7x.pdf) - 8MHz crystal specifications
- [ECS-.327-6-34R-TR Datasheet](https://ecsxtal.com/store/pdf/ECS_Catalog.pdf) - 32.768kHz crystal specifications

### Vishay / Diodes Inc
- [SIC472ED Datasheet](https://www.vishay.com/docs/75786/sic47x.pdf) - 8A switching regulator
- [AP64350/AP64200/AP64100 Datasheet](https://www.diodes.com/datasheet/download/AP64350.pdf) - Buck converters

---

## Open Issues

### 1. IMU Power Supply Decoupling - IMPROVEMENT RECOMMENDED

**Location:** SCH-FLOWTECH-Flight_Control-Sensors.SchDoc, IMU [6-AXIS] section

**Current Design (A.1):** Each ICM-45686 has 100nF + 22nF on VDD and 100nF + 22nF on VDDIO, and is fed by its own AP7361C-33 regulator (U1 for IMU1, U2 for IMU2) with 4.7uF on the regulator output.

Per [ICM-45686 Datasheet](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf) Section 10 "Application Information":
> "Place 0.1uF and 10uF capacitors as close as possible to the VDD and VDDIO pins"

The dedicated per-IMU regulators are a substantial improvement over the shared rail in the ABSI revision, and the 4.7uF regulator output capacitor provides local bulk. However, the 22nF parts added in A.1 are high-frequency capacitors, not bulk: there is still no capacitor in the 1uF-10uF range at the IMU pins themselves.

**Recommendation:** Add a 10uF (or at minimum 1uF) capacitor at each IMU's VDD pin, in addition to the existing 100nF. Keep it within 2mm of the pin.

**Rationale:** IMUs are sensitive to power supply noise. Insufficient bulk decoupling causes increased noise floor, vibration sensitivity, and offset drift.

---

### 2. Second Battery Monitoring - PARTS NOT POPULATED

**Location:** SCH-FLOWTECH-Flight_Control-MCU.SchDoc, ADC section

**Correction:** an earlier revision of this review claimed PC2/PC3 could not be read on the 100-pin package, and cited datasheet "Note 7". That was wrong on both counts, and the hwdef comment that repeated it has been fixed. Note 7 concerns VREF+ on the TFBGA100 package and is unrelated. The pins work.

Per [STM32H743 Datasheet](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) Table 10 "Pin and ball definitions":
- On LQFP100, PC2_C is pin 17 and PC3_C is pin 18. The plain PC2/PC3 pads are not bonded on any LQFP package - only on the largest BGA - so this is not a 100-pin limitation.
- PC2_C/PC3_C carry **Note 6**: "There is a direct path between Pxy_C and Pxy pins/balls, through an analog switch. Pxy alternate functions are available on Pxy_C when the analog switch is closed."
- PC2's function list is `ADC123_INP12` and PC3's is `ADC12_INP13`. With the switch closed - its reset state, and ArduPilot never writes `SYSCFG_PMCR` - `ADC1_INP12`/`ADC1_INP13` reach the pads.
- ArduPilot's own pin database agrees: `STM32H743xx.py` maps PC2 to ADC1 channel 12 and PC3 to channel 13, and does not list either under ADC3. In-tree precedent: CUAV-7-Nano reads `PC2 BATT_VOLTAGE_SENS ADC1` on a shipping H743 in exactly this configuration.

**Actual issue:** the Power Distribution Board lists the second battery parts (TVS, voltage and current op-amps, shunt, ideal-diode controllers) under "Components required for dual battery support" and does not populate them. The signals reach the MCU and could be read; there is simply nothing driving them.

**Recommendation:** no silicon change needed. Either populate the PDB parts on variants that want a second analog battery - the hwdef has the two ADC lines ready to uncomment, giving `BATT2_VOLT_PIN` 12 and `BATT2_CURR_PIN` 13 - or drop the feature from the ICD and use a DroneCAN battery monitor, which is what the board documents today.

---

### 3. Servo Rail Voltage Not Monitored - RECOMMENDED FOR A WING

**Location:** SCH-FLOWTECH-Flight_Control-MCU.SchDoc, ADC section; PDB V_Servo rail

**Issue:** V-Servo (5V/6V at 8A) feeds the control surfaces, but no sense line for it reaches the MCU. Only the battery, the 5V V-MCU rail and the 4-in-1 ESC current are monitored.

**Impact:** ArduPilot has first-class support for this and the board cannot use any of it. Without a pin labelled `FMU_SERVORAIL_VCC_SENS`, `chibios_hwdef.py` does not emit `HAL_HAVE_SERVO_VOLTAGE`, so:
- :ref:`BRD_VSERVO_MIN<BRD_VSERVO_MIN>` is inert - the servo rail prearm check in `AP_Arming::board_voltage_checks()` compiles out.
- MAVLink `POWER_STATUS.Vservo` reports zero.

This matters more on a fixed-wing than on most airframes: the servo rail is the one that sags when several surfaces load up simultaneously, and a BEC drooping under transient load is a classic cause of in-flight resets that `BRD_VSERVO_MIN` exists to catch on the ground. 32 in-tree boards implement it.

**Recommendation:** bring V-Servo to a spare ADC1 pin through a divider on a future revision, and label it `FMU_SERVORAIL_VCC_SENS` in the hwdef. Sizing must cope with the 6V setting: a 3:1 divider gives 2.0V at 6V, comfortably inside VREF+. Note the two candidate pins, PC2 and PC3, are the ones the ICD assigns to the unpopulated second battery, so if dual analog batteries are not going to be built, one of them is free for this.

---

### 4. CAN Termination - JUMPER SELECTABLE (IMPROVED IN A.1)

**Location:** SCH-FLOWTECH-Flight_Control-CAN_XCVR.SchDoc

**Current Design (A.1):** FDCAN1 uses a single TCAN3414DRBR (U8) with a 60.4 Ohm split termination (R13/R14) and a 560pF split capacitor (C39). R17, a 0 Ohm jumper, sits in series with the termination.

Per [TCAN3414 Datasheet](https://www.ti.com/lit/ds/symlink/tcan3414.pdf) Section 8.2.1.1 "CAN Termination":
> "Termination may be a single 120-Ohm resistor at each end of the bus... If filtering and stabilization of the common-mode voltage of the bus is desired then split termination may be used"

**Status:** A.1 added the R17 jumper, so termination can now be lifted for a mid-bus node without removing the termination resistors themselves. This addresses the substance of the original finding.

**Remaining consideration:** Termination is still a solder-time choice rather than a runtime one. The best solution is a GPIO-controlled MOSFET switch on the termination, allowing users to enable/disable it via ArduPilot parameters without hardware modification. This is standard practice on professional flight controllers and is worth considering for a future revision.

---

## Resolved in Revision A.1

### 5. MCU VCAP Capacitors - RESOLVED

~~**Issue:** The schematic showed 100nF capacitors on VCAP pins (pins 48 and 73), where [DS12110](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) Table 124 "VCAP operating conditions" requires CDEXT = 2.2uF +/-20% with ESR < 100 mOhm.~~

**Status:** Resolved in A.1. C7 and C8 are now 2.2uF, and the schematic note reads "VCAP: Each pin requires 1 x 2.2uF decoupling cap, as the internal voltage regulator will be used."

---

### 6. Crystal Load Capacitor Values - RESOLVED

~~**Issue:** Both crystals had load capacitors roughly 2pF low.~~

**Status:** Resolved in A.1. Per [AN2867](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf) Section 5.2, `CL = (CL1 x CL2) / (CL1 + CL2) + Cstray`, and the schematic now assumes Cstray = 3pF:

- **32.768kHz (Y1 - ECS-.327-6-34R-TR, CL = 6pF):** C18/C19 raised from 2pF to 6pF. CL = 3pF + 3pF = 6pF, matching the crystal.
- **8MHz (Y2 - ECS-80-8-33B2Q-CVY-TR3, CL = 8pF):** C20/C21 raised from 6pF to 10pF. CL = 5pF + 3pF = 8pF, matching the crystal.

---

### 7. Third IMU - RESOLVED

~~**Observation:** The schematic showed a third ICM-45686 footprint marked "MAY" (optional population) and annotated "no longer used".~~

**Status:** Resolved in A.1. IMU3 and its chip select, interrupts and shared SPI4 nets have been removed from the design. Two IMUs (U5, U6) remain, which is what the hwdef defines.

---

### 8. SD Card Detect - RESOLVED (V3R3)

~~**Issue:** No MCU pin was assigned for card detection.~~

**Status:** Resolved in V3R3 and carried into A.1. PD4 is SD_DETECT and is configured in the hwdef. Per the schematic: "When an SD Card is inserted, CD_B will short CD_A to GND and set SD_DETECT LOW", so the pin is active low as the hwdef assumes.

---

### 9. USART2 - RESOLVED (V3R7)

~~**Observation:** USART2 was not exposed.~~

**Status:** Resolved in V3R7. USART2_RX (PA3) carries ESC telemetry from the 4-in-1 ESC connector (J8 pin 4) and USART2_TX (PD5) is brought out on the Tx2 pad.

---

### 10. 5V Rail Monitoring - RESOLVED (V3R7)

~~**Issue:** The board lacked rail voltage monitoring.~~

**Status:** Resolved in V3R7. PC4 (ADC1_INP4) monitors V-MCU through a 10k/10k 0.1% divider buffered by a TLV333IDCKR, giving 2.5V at a 5V rail. The hwdef declares this as `VDD_5V_SENS ... SCALE(2)`, which the generator turns into `ANALOG_VCC_5V_PIN 4` and `HAL_HAVE_BOARD_VOLTAGE 1`. ArduPilot reports it as `POWER_STATUS.Vcc` and prearm-checks it against :ref:`BRD_VBUS_MIN<BRD_VBUS_MIN>`.

The 3.3V rails are not monitored. They are generated locally by three AP7361C-33 regulators, so a divider on one would need an ADC1 pin. The only unassigned ones are PC2/PC3, which are better spent on the servo rail (finding 3) if the second battery is not being populated.

---

## Observations - No Action Needed

### 11. MCU Internal Monitoring - ALREADY WORKING

**Correction:** an earlier revision of this review claimed MCU monitoring was unavailable because "ADC3 is not available on the 100-pin LQFP package", and recommended moving to a 144-pin or 176-pin part to gain it. That was wrong, and the recommendation would have bought nothing.

`HAL_WITH_MCU_MONITORING` uses ADC3's **internal** channels - VSENSE (ch18), VREFINT (ch19) and VBAT (ch17). Internal channels are not bonded to pins, so no package can affect them. The define is set in the MCU definition itself (`STM32H743xx.py`, `DEFINES` block), not per board, and it is present in this board's generated `hwdef.h`. MCU temperature and VREF monitoring are already live and logged.

**What is true about the package:** ADC3's *external* inputs are unavailable, because ArduPilot maps them exclusively to port F and port H pins (PF3-PF10, PH2-PH5), and neither port is bonded on LQFP100. That constrains nothing on this design, which uses ADC1 throughout.

**Recommendation:** none. No action, and no reason to change package.

---

### 12. IMU Data Ready Interrupts - NOT USED

The IMU interrupt signals are routed to the MCU but commented out in the hwdef:
- PB2: IMU1_INT1
- PE10: IMU1_INT2
- PE4: IMU2_INT1
- PC13: IMU2_INT2

ArduPilot uses FIFO-based IMU access rather than interrupt-driven sampling. DRDY interrupts are not required and their absence does not impact flight performance. No action needed.

---

### 13. Barometer Interrupt - NOT CONNECTED

The BMP390 has a DRDY/INT pin shown as BAR_INT. Per the ICD it was removed in V3R3, and in A.1 the net is not routed to any MCU pin (PD4, which previously carried it, is now SD_DETECT).

**Impact:** None. The barometer is polled, which is standard for ArduPilot and adequate at barometer data rates.

---

### 14. PWM Group Allocation

**Current Timer Allocation:**
| Timer | Channels | PWM Outputs | Notes |
|-------|----------|-------------|-------|
| TIM1 | CH1-CH4 | PWM 1-4 | Bi-directional DShot |
| TIM8 | CH1-CH4 | PWM 5-8 | Bi-directional DShot |
| TIM3 | CH2-CH4 | PWM 9-11 | DShot capable |
| TIM4 | CH1-CH2 | PWM 12-13 | DShot capable |
| TIM2 | CH1-CH3 | PWM 14-16 | PWM only |

Per [RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) Section 29.3, TIM1 and TIM8 are advanced-control timers suited to motor control and DShot.

**Note:** PWM 14-16 are PWM-only because TIM2 is marked `NODMA` in the hwdef, not because of a timer conflict. The ChibiOS system timer is TIM5 (`STM32_ST_USE_TIMER 5`, which is also the H7 default), so TIM2 is free. The DMA streams are deliberately given to TIM1, TIM8, SPI1, SPI4 and SDMMC via `DMA_PRIORITY`, and the aux channels give up DShot to pay for it. This is a reasonable trade for outputs intended for servos.

---

## Power Architecture Summary

### PDB Power Rails (from SCH-FLOWTECH-Power_Distribution_2ESC, rev A.1)

| Rail | Regulator | Voltage | Current | Control |
|------|-----------|---------|---------|---------|
| V-Servo | [SIC472ED-T1-GE3](https://www.vishay.com/docs/75786/sic47x.pdf) | 5V/6V | 8A | Always on |
| V-Telem | [AP64350SP-13](https://www.diodes.com/datasheet/download/AP64350.pdf) | 5V | 3A | Always on |
| V-VTX | [AP64350SP-13](https://www.diodes.com/datasheet/download/AP64350.pdf) | 9V/12V | 3A | EN_VTX (GPIO 81) |
| V-MCU | [AP64350SP-13](https://www.diodes.com/datasheet/download/AP64350.pdf) | 5V | 3A | Always on |
| V-Additional | [AP64200SP](https://www.diodes.com/assets/Datasheets/AP64200.pdf) | 5V | 2A | EN_V-ADD (GPIO 82) |

A.1 changed the MCU/sensor supply from a 3.3V 1A rail (AP64100SP) to a 5V 3A rail, with 3.3V now generated on the flight controller itself:

| Local Regulator | Input | Output | Supplies |
|-----------------|-------|--------|----------|
| U1 AP7361C-33 | V-MCU | V-IMU1 | IMU1 |
| U2 AP7361C-33 | V-MCU | V-IMU2 | IMU2 |
| U3 AP7361C-33 | V-MCU | V-Sensors | MCU, barometer, CAN transceiver |

Splitting the IMUs onto their own regulators is the most significant change in A.1 for flight performance, and is a good design decision.

### Power Sequencing Consideration

Per [AN5096 Section 3.5.1](https://www.st.com/resource/en/application_note/an5096-getting-started-with-stm32h7-series-hardware-development-stmicroelectronics.pdf):
> "VDD and VDDA must be powered simultaneously... VDDA must be equal to or higher than VDD"

VDD and VDDA are both fed from V-Sensors, so they rise together and this is satisfied. VBAT is tied to V-Sensors, which is correct where there is no backup battery.

**Recommendation:** Consider routing a power-good signal to a GPIO. The SIC472ED has a PGOOD output that is currently unused.

---

## Revision History (Firmware-Relevant Changes)

| Revision | Changes affecting hwdef |
|----------|------------------------|
| V3R3 | SD card detect moved to PD4; barometer interrupt removed |
| V3R4 | Barometer moved from I2C1 to I2C4; GPS/compass moved from I2C2 to I2C1 |
| V3R5 | Blue LED moved from PE10 to PE15; Green LED moved from PE15 to PE12; Amber LED added on PD3 |
| V3R6 | PDB I2C moved to PB10/PB11 (I2C2) |
| V3R7 | IMU1_CS moved from PE3 to PA4; IMU2_CS moved from PE4 to PE3; CAN1_STB moved from PD3 to PA15; SPST relay moved from PD4 to PA8; USART2 added (PD5/PA3); UART4 added (PC10/PC11); PC4 reassigned to VDD_5V_SENS; PC5 reassigned to ESC current |
| A.1 | Rebranded ABSI to FLOWTECH; IMU3 removed; CAN transceiver changed from TJA1462 to TCAN3414 and CAN2's transceiver moved to the PDB; second CAN connector added; MCU/sensor rail changed from 3.3V to 5V with three local AP7361C-33 regulators; USB-C breakout interface changed to V-USB with no 3.3V supply; PDB output relay changed from TLP241A to a DMN2056U-7 MOSFET; connector designators renumbered; VCAP and crystal load capacitors corrected |

No A.1 change altered an MCU pin assignment, so the hwdef pin map is unchanged from V3R7.

---

## Known Documentation Errata (A.1)

These are errors in the vendor documentation rather than the hardware, noted so they are not "fixed" into the hwdef by mistake:

1. **SCH-FLOWTECH-Flight_Control-PDB_Interface.SchDoc, NOTES block** states "2x I2C (I2C4 -> PB8/9)" for the PDB I2C. This is stale text carried over from the ABSI revision. The netlist and ICD V3R7 both route the PDB I2C to I2C2 on PB10/PB11 (changed in V3R6), and PB8/PB9 (I2C4) go to the onboard barometer. The netlist is correct; the note is not.

2. **ABSI-Flight_Controller-ICD-V3R7.xlsx** labels connector UART and flow-control pins from the peripheral's point of view (connector "TX" maps to the FC's UART RX). The schematic labels the same pins from the flight controller's point of view. Both describe identical wiring; the README uses the flight controller's convention.

---

*Review basis: SCH-FLOWTECH-Flight_Control-A.1, SCH-FLOWTECH-Power_Distribution_2ESC-A.1, ABSI-Flight_Controller-ICD-V3R7.xlsx*
*Reviewer: ArduPilot hwdef analysis*
