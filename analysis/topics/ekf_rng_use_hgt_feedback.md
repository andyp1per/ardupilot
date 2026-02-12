# EK3_RNG_USE_HGT Feedback Loop

## Summary

When `EK3_RNG_USE_HGT > 0`, the rangefinder height source switching uses EKF
altitude to decide whether to use the rangefinder. If baro noise corrupts the
EKF altitude estimate above the threshold, the rangefinder gets locked out —
creating a positive feedback loop.

## The Feedback Loop

```
Baro noise → Corrupted EKF altitude → Above RNG_USE_HGT threshold
    ↑                                           ↓
    └── EKF relies on corrupted baro ←── Rangefinder disabled
```

1. Baro noise corrupts EKF altitude → reports higher than actual
2. EKF altitude exceeds `EK3_RNG_USE_HGT` threshold
3. Rangefinder height source disabled
4. EKF relies entirely on corrupted baro
5. EKF altitude stays wrong → rangefinder stays locked out

## Evidence

### Discovery: logjk6 → logjk7 comparison

| Parameter | [logjk6](../logs/logjk6.md) | [logjk7](../logs/logjk7.md) |
|-----------|--------|--------|
| EK3_RNG_USE_HGT | 2.0 | **-1** |
| Flight duration | 22s | 74s |
| Alt std dev | **67.4cm** | **10.4cm** |
| Alt error mean | 39.4cm | 14.0cm |
| Alt error max | 129.4cm | 34.5cm |

Switching to `EK3_RNG_USE_HGT=-1` produced a **6.5x improvement** in altitude hold.

### Confirmation: logtd2

[logtd2](../logs/logtd2.md) with `EK3_RNG_USE_HGT=-1` achieved 6.2cm alt std — the
best result across all analyzed logs.

### Also observed in logjk4

In [logjk4](../logs/logjk4.md), ground effect flags cleared at t=17.0s when EKF
reported 0.8m altitude, but rangefinder showed vehicle was at 0.17m. The same
feedback mechanism applies to the ground effect altitude threshold.

## Code Path

`ArduCopter/baro_ground_effect.cpp:53`:
```cpp
if (gndeffect_state.takeoff_expected &&
    (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 ||
     height_above_takeoff_cm > gndeff_alt_cm)) {
    gndeffect_state.takeoff_expected = false;
}
```

The `height_above_takeoff_cm` is the EKF altitude estimate — which can be wrong.

## Fix: EK3_RNG_USE_HGT = -1

Setting `EK3_RNG_USE_HGT = -1` disables the automatic rangefinder height source
switching entirely. The rangefinder is still used by the EKF, but its contribution
isn't gated by the EKF's own altitude estimate.

## Potential Better Fixes

1. **Use rangefinder for threshold check** — when rangefinder data is available,
   use it instead of EKF altitude for the height threshold comparison
2. **Require consistent readings** — don't disable rangefinder based on a single
   altitude crossing; require sustained time above threshold
3. **Hysteresis** — different thresholds for enabling vs disabling rangefinder

## Recommendation

For vehicles with significant baro propwash effects (small indoor drones):
- Use `EK3_RNG_USE_HGT = -1`
- Combine with `BARO1_THST_SCALE` for steady-state correction
- Use `TKOFF_GNDEFF_ALT` for takeoff protection
