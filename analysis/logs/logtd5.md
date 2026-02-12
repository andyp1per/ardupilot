# logtd5 Analysis

## Metadata
- **Date**: 2025-02-12
- **Vehicle**: TD outdoor (user drone)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd5.bin

## Parameters
- `MOT_THST_HOVER = 0.125` (very overpowered)
- `FRAME_CLASS = 1`, `FRAME_TYPE = 1` (Quad BetaFlight-X)
- `RNGFND1_TYPE = 24` (TOF sensor), `RNGFND1_MAX_CM = 3000`
- `EK3_IMU_MASK = 2` (only IMU1 used by EKF)
- `PSC_POSZ_P = 0.3` (too low)
- `PSC_VELZ_P = 1.8` (too low)
- No BARO1_THST_SCALE (baseline)
- `INS_ACC_VRFB_Z = -0.531` on IMU0 (not used — wrong IMU)

## Flight Profile
- LOITER at ~20m for 2 minutes

## Performance Metrics
- Alt error std: **47.3cm**
- Alt error mean: -41.3cm (persistent undershoot)
- Baro-EKF divergence: 55cm std

## Sensor Status
- Rangefinder: NoData at 20m (beyond 12m effective range), Good below ~12m
- Baro temp: 42→20°C during flight (-22°C swing from airflow cooling)

## Analysis

### Baro Thermal Drift is Dominant Error Source

- 21°C temperature drop from board self-heating cooling in airflow
- 1.53m equivalent baro drift over 3-minute flight
- At 20m altitude, propwash effect on baro is negligible
- TCAL_BARO_EXP model may not match this sensor's thermal behavior

### PSC Gains Are the Biggest Lever

The primary cause of poor altitude hold is under-tuned position controller, not sensor issues:
- `PSC_POSZ_P = 0.3` (default is 1.0) — far too low
- `PSC_VELZ_P = 1.8` (default is 5.0) — far too low

### BARO1_THST_SCALE Estimation
- At 3-8m: estimated -225 to -300 Pa (altitude-detrended)
- At 15-22m: negligible correlation (propwash doesn't reach)
- At hover throttle 0.125, correction is only -12 to -19 Pa ≈ 0.1-0.2m
- Recommended conservative value: -100 to -150 Pa

### INS_ACC_VRFB_Z on Wrong IMU
- `INS_ACC_VRFB_Z = -0.531` was set on IMU0
- `EK3_IMU_MASK = 2` means only IMU1 is used by EKF
- The bias correction had no effect on flight

## Recommendations
- **Primary fix:** Increase PSC_POSZ_P to 1.0, PSC_VELZ_P to 5.0
- Reset INS_ACC_VRFB_Z to 0 (wrong IMU)
- Set BARO1_THST_SCALE = -100 (conservative)
- See logtd6 for results of tuning changes

## See Also
- [logtd6](logtd6.md) — improved tuning results
- [logtd7](logtd7.md) — two-phase flight
- [Baro thermal drift](../topics/baro_thermal_drift.md)
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
