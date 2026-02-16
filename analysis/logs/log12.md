# log12 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log12.bin
- **Type**: **GROUND SESSION — no flight**

## Summary

This log is a ground-only session. The vehicle was never armed and motors never spun.
Duration: ~206 seconds. No arming was attempted (no prearm failure messages in log).

This may have been a sensor check, compass calibration, or configuration session.

## Parameters

Same as [log11](log11.md) — no parameter changes.

Key values:
| Parameter | Value |
|-----------|-------|
| TKOFF_GNDEFF_TMO | 3.0 |
| ARMING_CHECK | 1 |
| MOT_THST_HOVER | 0.1338 |
| INS_ACC_VRFB_Z | -0.3948 |
| INS_ACC2_VRFB_Z | -0.2221 |
| ACC_ZBIAS_LEARN | 3 |
| EK3_MAG_CAL | 7 |
| COMPASS_MOTCT | 2 |

## EKF Status

- Both cores initialized and aligned
- SH (height variance) stayed < 0.02 throughout — excellent on ground
- No EKF failsafe events
- GPS: 3D fix, up to 16 sats, best HDop 0.73

## Temperature Warmup

| Sensor | Start | End | Rise |
|--------|-------|-----|------|
| IMU0 | 32.7 C | 50.6 C | **+17.9 C** |
| IMU1 | 34.2 C | 51.1 C | **+16.9 C** |
| BARO | 41.8 C | 61.8 C | **+20.0 C** |

Significant thermal soak from cold start.

## Baro Thermal Drift

- Alt drift: 0.000m → **-1.170m** over 206s
- Temp change: 41.8°C → 61.8°C (+20.0°C)
- Alt vs Temp correlation: **-0.978** (very strong)
- Drift rate: ~0.058 m/°C

With motors off, BARO1_THST_SCALE has no effect. The -1.17m drift is purely thermal.
This quantifies the baro thermal drift on this vehicle: **~6cm per degree C**.

## Compass — Physical Rotation Detected

At T+200s, the vehicle was physically rotated (likely by hand):
- MagX swept 55→400→22 (range 378 mGauss)
- MagY swept 199→-326 (range 525 mGauss)
- MagZ swept 365→242→403 (range 161 mGauss)

This rotation generated magnetometer innovation spikes up to 275 mGauss (26 spikes >50 mG).

**Ground baseline compass field:** |Field|=479 mGauss, std=6.3 mGauss (motors off).
This is the "clean" compass reading without motor interference — compare to 631 mGauss
mean during hover in log11 with 400-650 mGauss motor compensation applied.

A single earlier spike at T+188s (MagX jumped 244→333→243, 89 mGauss) suggests a brief
physical bump or nearby ferrous object.

## Rangefinder

- 4,118 readings, 4,111 good (99.8%)
- Reading ~0.01m on ground (expected)

## Velocity Innovation Pattern

Both cores show IVN/IVE at zero until GPS provides updates:
- Core 0: First non-zero IVN at T+39s, IVE at T+61s
- Core 1: First non-zero IVN at T+35s, IVE at T+86s
- Core 1 SV=0.0 for ALL samples (velocity variance never computed)

The long delay for velocity innovations on the ground is normal — the vehicle is stationary
so GPS velocity innovations are near zero.

## No Hover Data Available

Since the vehicle never flew, there is no data for:
- Altitude hold performance
- In-flight vibration
- In-flight compass interference
- Motor output asymmetry
- ESC RPM / notch alignment
- Post-landing EKF divergence

## See Also
- [log11](log11.md) — previous flight (same session)
