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
- CDEXT = 2.2 µF ±20%
- ESR < 100 mΩ

See also [AN5096 Section 3.5 "Power supply"](https://www.st.com/resource/en/application_note/an5096-getting-started-with-stm32h7-series-hardware-development-stmicroelectronics.pdf) which states:
> "The two VCAP pins must be connected to 2.2 µF ceramic capacitors (X5R or X7R, 4V minimum rating)"

**Current Design:** 2x 100nF (C7, C8)

**Required:** 2x 2.2µF ceramic capacitors (X5R or X7R, 6.3V minimum)

**Impact:** The internal voltage regulator may not be stable. This can cause:
- Random resets
- Erratic behavior under load
- Failed operation at temperature extremes
- Potential permanent damage to the MCU

**Recommendation:** Replace C7 and C8 with 2.2µF X5R or X7R capacitors. Place as close to VCAP pins as possible with short, wide traces to ground.

---

## Important Issues

### 2. Crystal Load Capacitor Values - VERIFY

**Location:** SCH-ABSI-Flight_Control-MCU.SchDoc, EXTERNAL OSCILLATORS section

Per [AN2867: Oscillator Design Guide](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf) Section 5.2, the load capacitor calculation is:

```
CL = (CL1 × CL2) / (CL1 + CL2) + Cstray
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
> "Place 0.1µF and 10µF capacitors as close as possible to the VDD and VDDIO pins"

The datasheet Figure 10-1 "Application Circuit" shows:
- VDD: 100nF + 10µF
- VDDIO: 100nF (minimum), 100nF + 10µF (recommended)

**Recommendation:** Add bulk capacitors per datasheet recommendations:
- VDD: 100nF + 10µF (or at minimum 1µF)
- VDDIO: 100nF + 1µF (or 10µF for best performance)

**Rationale:** IMUs are extremely sensitive to power supply noise. Insufficient decoupling causes:
- Increased noise floor
- Vibration sensitivity
- Offset drift

---

### 4. Second Battery Monitoring - NOT AVAILABLE

**Location:** SCH-ABSI-Flight_Control-MCU.SchDoc, ADC section

**Issue:** The schematic shows BATTERY2_V on PC2 (ADC3_INP0) and BATTERY2_I on PC3 (ADC3_INP1).

**Problem:** Per [STM32H743 Datasheet](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) Table 10 "STM32H743xI pin and ball definitions" (page 90), on the STM32H743VIT6 (100-pin LQFP package):
- PC2 and PC3 are mapped to ADC3 channels only
- ADC3 inputs on these pins are not bonded out in the 100-pin package

From the datasheet pin definitions:
- PC0 = ADC1_INP10 / ADC2_INP10 / ADC3_INP10 ✓
- PC1 = ADC1_INP11 / ADC2_INP11 / ADC3_INP11 ✓
- PC2 = ADC3_INP0 only (NOT available on 100-pin, see Note 7)
- PC3 = ADC3_INP1 only (NOT available on 100-pin, see Note 7)

**Impact:** Second battery monitoring cannot be implemented with current pin assignments.

**Recommendations:**

*Option A (Different ADC pins):*
Use pins that have ADC1/ADC2 alternate functions (per Table 10):
- PA0 = ADC1_INP16 (currently UART4_TX)
- PA1 = ADC1_INP17 (currently UART4_RX)
- PA2 = ADC1_INP14 (currently PWM13)
- PA3 = ADC1_INP15 (currently PWM14)

*Option B (Reduce UARTs):*
- Remove UART4 (PA0/PA1) and use those pins for BATT2_V/BATT2_I
- This maintains all PWM outputs

*Option C (Use PDB CAN for battery monitoring):*
- Implement battery monitoring on the PDB via CAN (FDCAN2)
- PDB sends battery data to FC over DroneCAN
- This is actually a cleaner architecture

---

## Moderate Issues

### 5. SD Card Detect Pin - NOT CONFIGURED

**Location:** SCH-ABSI-Flight_Control-User_IO.SchDoc, SD CARD section

**Issue:** The SD card socket (SKT1 - Wurth 5033981892) has CD_A (pin 9) and CD_B (pin 10) card detect pins, but no MCU pin is assigned for card detection.

**Impact:**
- ArduPilot cannot detect SD card insertion/removal
- Must assume card is always present
- Hot-swap of SD card may cause filesystem corruption

**Recommendation:** Connect CD_A or CD_B to an available GPIO (active low typically). Suggested pins:
- PC13 (available, near SD signals)
- Any unused PE pin

---

---

### 6. IMU Data Ready Interrupts - NOT USED (Low Priority)

**Location:** SCH-ABSI-Flight_Control-Sensors.SchDoc

**Observation:** The IMU interrupt signals (IMU1_INT1, IMU1_INT2, IMU2_INT1, IMU2_INT2) are routed to the MCU but commented out in the hwdef.

**Current hwdef:**
```
# Optional data ready interrupts - not required for operation
# PE12 IMU1_DRDY INPUT
# PC13 IMU2_DRDY INPUT
```

**Note:** ArduPilot typically uses FIFO-based IMU access rather than interrupt-driven sampling. The FIFO approach is more than adequate for most applications and is the standard method used across ArduPilot-supported boards. DRDY interrupts are not required and their absence does not impact flight performance.

**Recommendation:** No action needed. The current FIFO-based approach is appropriate.

---

### 7. CAN Termination - ALWAYS PRESENT

**Location:** SCH-ABSI-Flight_Control-CAN_XCVR.SchDoc

**Issue:** Both CAN buses have fixed 60.4Ω termination resistors (R12/R14 for CAN1, R13/R15 for CAN2).

Per [TJA1462 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1462.pdf) Section 8.1 "Bus termination":
> "For proper CAN bus operation, the bus must be terminated at both ends with 120Ω resistors."

The 60.4Ω split termination with 47nF capacitor provides 120Ω DC termination with common-mode filtering (per Section 8.2 "Split termination").

**Note from schematic:** "Flight controller not expected to be central node. Termination components will remain and can be removed by user if necessary."

**Consideration:** Fixed termination is acceptable if:
- FC is always at end of CAN bus
- PDB (connected via CAN2) has no termination

If FC is in the middle of a CAN bus, the extra termination will cause signal integrity issues.

**Recommendation:** The best solution is to make termination software-configurable via a GPIO pin controlling a MOSFET switch on the termination resistor. This allows users to enable/disable termination via ArduPilot parameters without hardware modification. Alternatively, consider adding solder jumpers to allow termination to be disabled manually. Both approaches are standard practice on professional flight controllers.

---

### 8. Voltage Monitoring - LIMITED

**Issue:** The board lacks comprehensive voltage monitoring capabilities:

1. **Internal MCU monitoring not available:** `HAL_WITH_MCU_MONITORING` requires ADC3 internal channels for MCU temperature, VREF, and VBAT monitoring. However, ADC3 is NOT available on the 100-pin LQFP package (STM32H743VIT6). This is a package limitation, not a design error.

2. **No 5V rail monitoring:** There is no `VDD_5V_SENS` pin to monitor the 5V supply rail voltage. Many flight controllers include this to detect brownouts or power issues.

**Impact:** 
- Cannot monitor MCU die temperature for thermal throttling warnings
- Cannot detect 5V rail voltage drops that may indicate power system issues
- Cannot use internal VREF for ADC calibration

**Recommendation for future revision:**
1. Consider using the 144-pin or 176-pin STM32H743 package to gain access to ADC3 for internal monitoring
2. Add a voltage divider from the 5V rail to an ADC pin for `VDD_5V_SENS`:
   - If RSSI input is not needed, repurpose PC5 for VDD_5V_SENS
   - Alternatively, add a dedicated voltage divider (e.g., 10k/10k for 2:1 ratio) to a spare ADC-capable pin
3. For 3.3V monitoring, a simple voltage divider to any ADC pin would suffice

**Example hwdef for 5V monitoring:**
```
PC5 VDD_5V_SENS ADC1 SCALE(2)  # 10k/10k divider = 2:1 ratio
```

---

## Minor Issues / Suggestions

### 9. USART2 - NOT EXPOSED

**Observation:** Per [STM32H743 Datasheet Table 9](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf), USART2 is available on:
- PD5 = USART2_TX (AF7)
- PD6 = USART2_RX (AF7) - but used for SDMMC2_CK

PD5 appears unused in the current design.

**Opportunity:** Could expose USART2_TX on PD5 as a half-duplex debug/auxiliary UART if needed in future revisions.

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
| TIM1 | CH1-CH4 | PWM 1-4 | BIDIR DShot ✓ |
| TIM8 | CH1-CH4 | PWM 5-8 | BIDIR DShot ✓ |
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
| 3V3_MCU | [AP64100SP](https://www.diodes.com/datasheet/download/AP64100.pdf) | 3.3V | 1A | Always on |
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

## Firmware Considerations

Based on the hardware analysis, the following firmware configurations are recommended:

### hwdef.dat Updates Needed

1. **Add SD card detect** (if GPIO is connected):
```
# Example if using PC13:
PC13 SDCARD_DETECT INPUT
define HAL_SDCARD_DETECT_PIN 13
```

2. **Battery 2 monitoring** (if using CAN-based monitoring):
```
define HAL_BATTMON_SMBUS_ENABLE 0
# Configure DroneCAN battery monitor instead
```

---

## Summary of Recommendations by Priority

### Must Fix (Before Production)
1. ❌ **VCAP capacitors:** Change from 100nF to 2.2µF - [DS12110 Table 124](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf)

### Should Fix (High Impact)
2. ⚠️ **Crystal load capacitors:** Verify and adjust values - [AN2867](https://www.st.com/resource/en/application_note/an2867-guidelines-for-oscillator-design-on-stm8afals-and-stm32-mcusmpus-stmicroelectronics.pdf)
3. ⚠️ **IMU bulk decoupling:** Add 10µF capacitors - [ICM-45686 DS Section 10](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf)
4. ⚠️ **Battery 2 ADC:** Reassign pins or use CAN monitoring - [DS12110 Table 10](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf)

### Consider (Improvements)
5. 💡 **SD card detect:** Add GPIO connection
6. 💡 **CAN termination:** Add GPIO-controlled or solder jumpers - [TJA1462 DS Section 8.1](https://www.nxp.com/docs/en/data-sheet/TJA1462.pdf)
7. 💡 **Voltage monitoring:** Add VDD_5V_SENS or use larger MCU package for HAL_WITH_MCU_MONITORING
8. 💡 **Power-good monitoring:** Route to GPIO - [SIC472ED DS](https://www.vishay.com/docs/75786/sic47x.pdf)

---

*Review Date: Based on schematics SCH-ABSI-Flight_Control-System.SchDoc and SCH-ABSI-Power_Distribution_2ESC-System.SchDoc*
*Reviewer: ArduPilot hwdef analysis*
