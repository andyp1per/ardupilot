# Baro Thermal Drift Analysis

## Summary

Baro thermal drift is the dominant altitude error source for outdoor flights at
moderate altitude (>10m), where propwash effects are negligible.

## Key Findings from logtd5-7

### Vehicle Characteristics
- MOT_THST_HOVER = 0.125 (very overpowered)
- Baro temp starts ~42-45°C (electronics self-heating), drops to ~20°C in flight
- Temperature swing: **-17 to -21°C** from airflow cooling
- Effective range of TOF rangefinder: ~12m

### Temperature Effect on Baro

| Log | Temp Range | Temp Drop | Baro Drift | Notes |
|-----|-----------|-----------|------------|-------|
| [logtd5](../logs/logtd5.md) | 42→20°C | -22°C | 1.53m | LOITER at 20m |
| [logtd6](../logs/logtd6.md) | ~42→21°C | ~21°C | Similar | ALT_HOLD at 18m |
| [logtd7](../logs/logtd7.md) | Similar | Similar | Similar | Two-phase 27m/6m |

### TCAL_BARO_EXP Issue

The `TCAL_BARO_EXP` model adds `pow(T-25, expo)` to pressure. Analysis suggests
this may be calibrated for the wrong direction for this particular baro sensor:
- This sensor reads higher pressure when hot
- TCAL designed for ICM-20789 which reads low when hot
- May need sensor-specific TCAL calibration

## Impact on Altitude Hold

At 20m altitude:
- Propwash effect on baro: negligible
- Thermal drift: 1.5m over 3 minutes
- This drift directly maps to altitude error

## Compared to Other Error Sources

| Error Source | Typical Magnitude | Where It Matters |
|-------------|-------------------|-----------------|
| **Baro thermal drift** | **1.5m over 3 min** | **Outdoor, >10m alt** |
| Baro propwash | 2-7m | Indoor, <3m alt |
| PSC under-tuning | 40+cm std | Everywhere |
| Vibration rectification | 0.15 m/s² | Indoor no-GPS |

## Mitigation Options

1. **Re-calibrate TCAL** — Ensure baro temperature model matches actual sensor behavior
2. **Better thermal insulation** — Reduce airflow cooling of electronics
3. **Rangefinder fusion** — Use rangefinder where available to override baro
4. **Higher position gains** — PSC_POSZ_P=1.0 reduced impact from 47cm to 18cm std

## Related IMU Temperature Effects

From early SFD vehicle analysis (log4-8 series):
- IMU temp: 47°C → 36°C (11°C drop during indoor flight)
- IMU temp coefficient: ~0.021 m/s² per °C (IMU0), ~0.013 m/s²/°C (IMU1)
- INS_TCAL is enabled and working — residual drift is after TCAL correction
- The motor-induced AccZ shift (+0.10 m/s²) is 20x larger than temperature effect

## See Also
- [logtd5](../logs/logtd5.md), [logtd6](../logs/logtd6.md), [logtd7](../logs/logtd7.md)
