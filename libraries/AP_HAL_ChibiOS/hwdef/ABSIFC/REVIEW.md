# ABSI FC Hardware Review

This document contains recommendations for hardware improvements based on analysis of the schematics and pin assignments. The review focuses on the STM32H743VIT6 (100-pin LQFP package) pin constraints.

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

### NXP Semiconductors
- [TJA1462 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1462.pdf) - CAN transceiver specifications, termination recommendations

### ECS International
- [ECS-80-8-33B2Q-CVY-TR3 Datasheet](https://ecsxtal.com/store/pdf/csm-7x.pdf) - 8MHz crystal specifications
- [ECS-.327-6-34R-TR Datasheet](https://ecsxtal.com/store/pdf/ECS_Catalog.pdf) - 32.768kHz crystal specifications

### Vishay / Diodes Inc
- [SIC472ED Datasheet](https://www.vishay.com/docs/75786/sic47x.pdf) - 8A switching regulator
- [AP64350/AP64200/AP64100 Datasheet](https://www.diodes.com/datasheet/download/AP64350.pdf) - Buck converters

---

## Critical Issues

### 1. MCU VCAP Capacitors - INCORRECT VALUE

**Location:** SCH-ABSI-Flight_Control-MCU.SchDoc, MCU POWER section

**Issue:** The schematic shows 100nF capacitors on VCAP pins (pins 48 and 73).

**Problem:** Per [STM32H743 Datasheet](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) Table 124 "VCAP operating conditions" (Section 6.3.18, page 153), the VCAP pins require:
- CDEXT = 2.2 uF +/-20%
- ESR < 100 mOhm

See also [AN5096 Section 3.5 "Power supply"](https://www.st.com/resource/en/application_note/an5096-getting-started-with-stm32h7-series-hardware-development-stmicroelectronics.pdf) which states:
> "The two VCAP pins must be connected to 2.2 uF ceramic capacitors (X5R or X7R, 4V minimum rating)"

**Current Design:** 2x 100nF (C7, C8)

**Required:** 2x 2.2uF ceramic capacitors (X5R or X7R, 6.3V minimum)

**Impact:** The internal voltage regulator may not be stable. This can cause:
- Random resets
- Erratic behavior under load
- Failed operation at temperature extremes
- Potential permanent damage to the MCU

**Recommendation:** Replace C7 and C8 with 2.2uF X5R or X7R capacitors. Place as close to VCAP pins as possible with short, wide traces to ground.

---

## Important Issues

### 2. Crystal Load Capacitor Values - VERIFY

**Location:** SCH-ABSI-Flight_Control-MCU.SchDoc, EXTERNAL OSCILLATORS section

Per [AN2867: Oscillator Design Guide](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf) Section 5.2, the load capacitor calculation is:

```
CL = (CL1 x CL2) / (CL1 + CL2) + Cstray
```

Where Cstray is typically 2-5pF for PCB traces.

**8MHz Crystal (Y2 - ECS-80-8-33B2Q-CVY-TR3):**
- Per [ECS datasheet](https://ecsxtal.com/store/pdf/csm-7x.pdf): CL = 8pF
- Load capacitors: C13, C14 = 6pF each
- With 6pF caps and ~3pF stray: CL = 3pF + 3pF = 6pF
- **This appears to be 2pF low.** Recommend 8-10pF capacitors.

**32.768kHz Crystal (Y1 - ECS-.327-6-34R-TR):**
- Per [ECS datasheet](https://ecsxtal.com/store/pdf/ECS_Catalog.pdf): CL = 6pF
- Load capacitors: C11, C12 = 2pF each
- Calculated CL = 1pF + 3pF stray = 4pF
- **This is 2pF low.** May cause frequency drift or startup issues.
- Recommend 4-6pF capacitors for better frequency accuracy.

**Impact:** Incorrect load capacitors cause frequency deviation which affects:
- RTC timekeeping accuracy
- USB timing (may cause enumeration issues per [AN2867 Section 6.1](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf))
- UART baud rate accuracy

---

### 3. IMU Power Supply Decoupling - IMPROVEMENT RECOMMENDED

**Location:** SCH-ABSI-Flight_Control-Sensors.SchDoc, IMU [6-AXIS] section

**Current Design:** Each ICM-45686 has:
- VDD: 1x 100nF
- VDDIO: 1x 100nF

Per [ICM-45686 Datasheet](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf) Section 10 "Application Information":
> "Place 0.1uF and 10uF capacitors as close as possible to the VDD and VDDIO pins"

The datasheet Figure 10-1 "Application Circuit" shows:
- VDD: 100nF + 10uF
- VDDIO: 100nF (minimum), 100nF + 10uF (recommended)

**Recommendation:** Add bulk capacitors per datasheet recommendations:
- VDD: 100nF + 10uF (or at minimum 1uF)
- VDDIO: 100nF + 1uF (or 10uF for best performance)

**Rationale:** IMUs are extremely sensitive to power supply noise. Insufficient decoupling causes:
- Increased noise floor
- Vibration sensitivity
- Offset drift

---

### 4. Second Battery Monitoring - ADC3 LIMITATION

**Location:** SCH-ABSI-Flight_Control-MCU.SchDoc, ADC section

**Issue:** The ICD allocates PC2 (ADC3_INP0) and PC3 (ADC3_INP1) for BATTERY2_V and BATTERY2_I.

**Problem:** Per [STM32H743 Datasheet](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) Table 10 "STM32H743xI pin and ball definitions" (page 90), on the STM32H743VIT6 (100-pin LQFP package):
- PC2 and PC3 are mapped to ADC3 channels only
- ADC3 inputs on these pins are not bonded out in the 100-pin package

From the datasheet pin definitions:
- PC0 = ADC1_INP10 / ADC2_INP10 / ADC3_INP10 (available)
- PC1 = ADC1_INP11 / ADC2_INP11 / ADC3_INP11 (available)
- PC2 = ADC3_INP0 only (NOT available on 100-pin, see Note 7)
- PC3 = ADC3_INP1 only (NOT available on 100-pin, see Note 7)

**Impact:** Second battery monitoring cannot be implemented via analog ADC with current pin assignments on the 100-pin package.

**Recommendation:** Use DroneCAN battery monitor on CAN2 (PDB interface) for second battery monitoring. This is the cleanest architecture since the PDB already has CAN2 connectivity.

---

## Moderate Issues

### 5. SD Card Detect Pin - RESOLVED (V3R3)

~~**Issue:** No MCU pin was assigned for card detection.~~

**Status:** Resolved in V3R3. PD4 is now assigned as SDCARD_DETECT and is configured in the hwdef.

---

### 6. IMU Data Ready Interrupts - NOT USED (Low Priority)

**Location:** SCH-ABSI-Flight_Control-Sensors.SchDoc

**Observation:** The IMU interrupt signals are routed to the MCU but commented out in the hwdef:
- PB2: IMU1_INT1
- PE10: IMU1_INT2
- PE4: IMU2_INT1
- PC13: IMU2_INT2

**Note:** ArduPilot typically uses FIFO-based IMU access rather than interrupt-driven sampling. The FIFO approach is more than adequate for most applications and is the standard method used across ArduPilot-supported boards. DRDY interrupts are not required and their absence does not impact flight performance.

**Recommendation:** No action needed. The current FIFO-based approach is appropriate.

---

### 7. CAN Termination - ALWAYS PRESENT

**Location:** SCH-ABSI-Flight_Control-CAN_XCVR.SchDoc

**Issue:** Both CAN buses have fixed 60.4 Ohm termination resistors (R12/R14 for CAN1, R13/R15 for CAN2).

Per [TJA1462 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1462.pdf) Section 8.1 "Bus termination":
> "For proper CAN bus operation, the bus must be terminated at both ends with 120 Ohm resistors."

The 60.4 Ohm split termination with 47nF capacitor provides 120 Ohm DC termination with common-mode filtering (per Section 8.2 "Split termination").

**Note from schematic:** "Flight controller not expected to be central node. Termination components will remain and can be removed by user if necessary."

**Consideration:** Fixed termination is acceptable if:
- FC is always at end of CAN bus
- PDB (connected via CAN2) has no termination

If FC is in the middle of a CAN bus, the extra termination will cause signal integrity issues.

**Recommendation:** The best solution is to make termination software-configurable via a GPIO pin controlling a MOSFET switch on the termination resistor. This allows users to enable/disable termination via ArduPilot parameters without hardware modification. Alternatively, consider adding solder jumpers to allow termination to be disabled manually. Both approaches are standard practice on professional flight controllers.

---

### 8. Voltage Monitoring - PARTIALLY RESOLVED

**Issue:** The board previously lacked comprehensive voltage monitoring capabilities.

**Status (V3R7):** PC4 is now configured as VDD_5V_SENS with a 2:1 voltage divider, providing 5V rail monitoring. This is a significant improvement.

**Remaining limitations:**
1. **Internal MCU monitoring not available:** `HAL_WITH_MCU_MONITORING` requires ADC3 internal channels for MCU temperature, VREF, and VBAT monitoring. However, ADC3 is NOT available on the 100-pin LQFP package (STM32H743VIT6). This is a package limitation, not a design error.

2. **No 3.3V rail monitoring:** Consider adding a voltage divider from the 3.3V rail to a spare ADC pin if one becomes available.

**Recommendation for future revision:**
1. Consider using the 144-pin or 176-pin STM32H743 package to gain access to ADC3 for internal monitoring

---

## Minor Issues / Suggestions

### 9. USART2 - RESOLVED (V3R7)

~~**Observation:** USART2 was not exposed.~~

**Status:** Resolved in V3R7. USART2 is now exposed on PD5 (TX) and PA3 (RX) for ESC telemetry on the 4-in-1 ESC connector.

---

### 10. Third IMU Footprint

**Observation:** The schematic shows a third ICM-45686 footprint with "MAY" designation (optional population).

**Status:** IMU3_CS (PE?) is defined but IMU3 is marked as "no longer used" in the schematic notes.

**Recommendation:** If triple-redundant IMUs are desired for safety-critical applications, complete the IMU3 circuit. Otherwise, remove the footprint to reduce BOM cost and assembly complexity.

---

### 11. PWM Group Allocation

**Current Timer Allocation:**
| Timer | Channels | PWM Outputs | Notes |
|-------|----------|-------------|-------|
| TIM1 | CH1-CH4 | PWM 1-4 | BIDIR DShot |
| TIM8 | CH1-CH4 | PWM 5-8 | BIDIR DShot |
| TIM3 | CH2-CH4 | PWM 9-11 | DShot capable |
| TIM4 | CH1-CH2 | PWM 12-13 | DShot capable |
| TIM2 | CH1-CH3 | PWM 14-16 | PWM only |

Per [STM32H743 Reference Manual (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) Section 29.3, TIM1 and TIM8 are advanced-control timers with complementary outputs ideal for motor control and DShot.

**Note:** TIM2 outputs (PWM 14-16) are PWM-only since TIM2 is used as the system timer which precludes DShot support on those channels.

---

### 12. Barometer Interrupt - NOT USED

**Location:** The BMP390 has a DRDY/INT pin (pin 7) shown as BAR_INT in the schematic.

Per [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) Section 3.6 "Interrupt":
> "The interrupt can be configured to signal data ready, FIFO watermark, or FIFO full conditions."

**Status:** Not connected to MCU in current implementation.

**Impact:** Barometer is polled rather than interrupt-driven. This is acceptable - barometer data rates are low (~50-100Hz typical).

---

## Power Architecture Summary

### PDB Power Rails (from Power Distribution schematic)

| Rail | Regulator | Voltage | Current | Control |
|------|-----------|---------|---------|---------|
| V_Servo | [SIC472ED](https://www.vishay.com/docs/75786/sic47x.pdf) | 5V/6V | 8A | Always on |
| V_VTX | [AP64350SP](https://www.diodes.com/datasheet/download/AP64350.pdf) | 9V/12V | 3A | EN_VTX (GPIO 81) |
| 5V_Telemetry | [AP64350SP](https://www.diodes.com/datasheet/download/AP64350.pdf) | 5V | 3A | Always on |
| 3V3_MCU | [AP64100SP](https://www.diodes.com/assets/Datasheets/AP64100.pdf) | 3.3V | 1A | Always on |
| 5V_Additional | [AP64200SP](https://www.diodes.com/assets/Datasheets/AP64200.pdf) | 5V | 2A | EN_5V-2A (GPIO 82) |

### Power Sequencing Consideration

The schematics note "Consider power rail sequencing" on multiple regulators.

Per [AN5096 Section 3.5.1](https://www.st.com/resource/en/application_note/an5096-getting-started-with-stm32h7-series-hardware-development-stmicroelectronics.pdf):
> "VDD and VDDA must be powered simultaneously... VDDA must be equal to or higher than VDD"

For H7 MCUs:
1. 3V3_MCU should be stable before releasing reset
2. VDDA should rise with or after VDD
3. Current design ties VBAT to VDD (correct for no backup battery)

**Recommendation:** Add power-good monitoring on critical rails if not already present. The SIC472ED has PGOOD output - consider routing to MCU GPIO.

---

## ICD Revision History (Firmware-Relevant Changes)

| Revision | Changes affecting hwdef |
|----------|------------------------|
| V3R3 | SD card detect moved to PD4 |
| V3R4 | Barometer moved from I2C1 to I2C4; GPS/compass moved from I2C2 to I2C1; PDB I2C moved to I2C2 |
| V3R5 | Blue LED moved from PE10 to PE15; Green LED moved from PE15 to PE12; Amber LED added on PD3 |
| V3R6 | PDB I2C pins confirmed on PB10/PB11 (I2C2) |
| V3R7 | IMU1_CS moved from PE3 to PA4; IMU2_CS moved from PE4 to PE3; CAN1_STB moved from PD3 to PA15; SPST relay moved from PD4 to PA8; USART2 added (PD5/PA3); UART4 added (PC10/PC11); PC4 reassigned to VDD_5V_SENS; PC5 reassigned to ESC current |

---

## Summary of Recommendations by Priority

### Must Fix (Before Production)
1. **VCAP capacitors:** Change from 100nF to 2.2uF - [DS12110 Table 124](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf)

### Should Fix (High Impact)
2. **Crystal load capacitors:** Verify and adjust values - [AN2867](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf)
3. **IMU bulk decoupling:** Add 10uF capacitors - [ICM-45686 DS Section 10](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf)
4. **Battery 2 ADC:** Use DroneCAN battery monitor on CAN2 - [DS12110 Table 10](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf)

### Consider (Improvements)
5. **CAN termination:** Add GPIO-controlled or solder jumpers - [TJA1462 DS Section 8.1](https://www.nxp.com/docs/en/data-sheet/TJA1462.pdf)
6. **3.3V rail monitoring:** Add voltage divider if spare ADC pin available
7. **Power-good monitoring:** Route to GPIO - [SIC472ED DS](https://www.vishay.com/docs/75786/sic47x.pdf)

### Resolved in V3R3-V3R7
- SD card detect (V3R3): PD4 now connected
- USART2 exposure (V3R7): PA3/PD5 now available for ESC telemetry
- 5V rail monitoring (V3R7): PC4 configured as VDD_5V_SENS

---

*Review Date: Based on schematics SCH-ABSI-Flight_Control-System.SchDoc and SCH-ABSI-Power_Distribution_2ESC-System.SchDoc*
*ICD Reference: ABSI-Flight_Controller-ICD-V3R7.xlsx*
*Reviewer: ArduPilot hwdef analysis*
