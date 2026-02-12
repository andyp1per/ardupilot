# logtd6 Analysis

## Metadata
- **Date**: 2025-02-12
- **Vehicle**: TD outdoor (user drone)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd6.bin

## Parameters
- `PSC_POSZ_P = 1.0` (increased from 0.3)
- `PSC_VELZ_P = 5.0` (increased from 1.8)
- `INS_ACC_VRFB_Z = 0` (reset)
- `BARO1_THST_SCALE = -100`
- `EK3_IMU_MASK = 2` (only IMU1)

## Flight Profile
- ALT_HOLD at ~18m

## Performance Metrics
- Alt error std: **17.9cm** — dramatic improvement from logtd5's 47.3cm
- Alt error mean: -5.4cm (near zero — greatly improved)

## Sensor Status
- Similar to logtd5

## Analysis

### PSC Tuning Produced Dramatic Improvement

| Metric | logtd5 (P=0.3, V=1.8) | logtd6 (P=1.0, V=5.0) | Change |
|--------|------------------------|------------------------|--------|
| Alt error std | 47.3cm | **17.9cm** | 2.6x better |
| Alt error mean | -41.3cm | **-5.4cm** | 7.6x better |

### Hover Bias Learning Results
- Saved `INS_ACC2_VRFB_Z = -0.444` (IMU1, the active one)
- `INS_ACC_VRFB_Z` (IMU0) stayed at 0.0 — no EKF core on IMU0
- Correctly learning bias on the IMU actually used by EKF

### BARO_THST_SCALE at High Altitude
- At 18m in windy conditions:
  - Throttle: corr=0.333, 5.7% noise reduction
  - Current: corr=0.597, **19.8% noise reduction**
  - Throttle-current correlation was only 0.412
  - This is the one case where current outperformed throttle
  - 91% of current variation was NOT explained by throttle (wind gusts)

## Recommendations
- PSC_POSZ_P=1.0 and PSC_VELZ_P=5.0 confirmed as correct tuning
- Monitor INS_ACC2_VRFB_Z convergence over multiple flights
- At high altitude in wind, current-based compensation could help (but propwash is negligible there)

## See Also
- [logtd5](logtd5.md) — baseline comparison
- [logtd7](logtd7.md) — two-phase flight
- [Baro thermal drift](../topics/baro_thermal_drift.md)
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
