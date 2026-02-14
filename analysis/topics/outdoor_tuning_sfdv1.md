# Outdoor Tuning — SmallFastDronev1

Analysis from [log208](../logs/log208.md): outdoor hover with GPS, Loiter + Stabilize.

## Aggressive Takeoff

### Observation

Vehicle shoots to 0.92m in ~1 second with 139 cm/s peak climb rate, then reverses
to -122 cm/s. Significant bounce/overshoot on Loiter takeoff.

### Timeline

| Time | ThO | Alt (m) | CRt (cm/s) | Event |
|------|------|---------|------------|-------|
| 49.49 | 0.060 | -0.47 | -43 | Motors start |
| 49.59 | 0.126 | -0.47 | +28 | Thrust spike, BAlt=-5.67m ground effect |
| 49.89 | 0.074 | -0.02 | +139 | Peak climb rate |
| 50.69 | 0.048 | +0.92 | -42 | Peak altitude overshoot |
| 51.29 | 0.062 | +0.65 | -122 | Peak descent correction |

### Root Causes

1. **MOT_THST_HOVER mismatch:** MOT_THST_HOVER=0.125 but actual hover ThO=0.069.
   The position controller feedforward outputs 80% too much thrust before P/I even
   engage. ThO spikes to 0.126 (double actual hover) at first motor engagement.

2. **Very high thrust-to-weight ratio:** PSCD data shows actual vertical acceleration
   of -13.83 m/s^2 vs desired -2.50 m/s^2 (5.5x overshoot). RATE data shows
   A=1383 vs ADes=133 (10x in raw units). ESC RPMs doubled from 6000 to 12500 in
   200ms.

3. **Massive ground effect:** BAlt drops to -5.67m at motor startup. Even with
   BARO1_THST_SCALE=-20, this is a huge disturbance.

### Fix

- **MOT_THST_HOVER=0.07** — match actual flight data, cuts feedforward overshoot in half
- **TKOFF_SLEW_TIME=1.0-1.5** — slow motor ramp, reduce initial thrust spike
- Consider PILOT_TKOFF_ALT=1.0 for controlled takeoff ramp

## Roll Oscillation

### Observation

Discernible roll wobble during hover with sticks centered. Not present on pitch.

### Characterization

| Metric | Roll | Pitch |
|--------|------|-------|
| Dominant frequency | 8-10 Hz | Similar |
| Desired rate std | 3.13 deg/s | 2.85 deg/s |
| Actual rate std | 4.77 deg/s | 3.93 deg/s |
| Overshoot ratio (Act/Des) | **1.52** | 1.38 |
| ANG_P | 27.25 | 31.54 |
| RAT_P | 0.0562 | 0.0832 |
| RAT_D | 0.000661 | 0.001064 |
| Amplitude range | 2-11 deg/s RMS | — |

### Root Cause: Coupled Angle-Rate Limit Cycle

**Mechanism:**
1. Angle error of 0.1 deg -> ANG_P=27.25 commands 2.7 deg/s rate demand at 8-10 Hz
2. Rate loop responds but overshoots by 52% (Act std 5.27 vs Tar std 3.48)
3. Overshoot carries angle past target
4. Angle controller reverses rate demand -> cycle repeats
5. System is at marginal stability: 50/50 consecutive peak growth ratio

**Evidence it is pure tuning, NOT physical noise:**
- Vibrations low: 3.8-4.0 m/s^2 (well under 15 threshold)
- Zero throttle-oscillation correlation (r=-0.043)
- Zero roll-pitch rate correlation (r=-0.057, axes are independent)
- Perfectly symmetric distribution (skewness=-0.066)
- Strong 8 Hz autocorrelation structure (not broadband noise)
- D-term autocorrelation goes negative at half-period (structured, not noisy)

**Evidence the angle controller drives it:**
- Tar (desired rate) has 8-10 Hz content: 55% of peaks in 7-12 Hz band
- Tar-Act correlation peaks at 0.807 with 20ms lag (coupled oscillation)
- Roll angle error std=0.10 deg -> ANG_P=27.25 -> 2.7 deg/s rate demand
  (accounts for 80% of Tar RMS)

### Airframe Asymmetry Factor

The vehicle has:
- Battery aligned along roll axis -> lower roll moment of inertia
- Motors wider laterally than longitudinally -> more roll torque authority

Both factors give roll higher plant gain (torque_arm / inertia) than pitch. Despite
roll having 32% lower RAT_P (0.0562 vs 0.0832), the effective loop gain is higher
for roll because the plant gain difference exceeds 1.5x.

Autotune partially compensated (gave roll lower RAT_P) but not enough — the 1.52
overshoot ratio proves roll's effective loop gain is still too high.

### PID Term Energy (Roll, Hover)

| Term | Energy % | Notes |
|------|----------|-------|
| P | 49.1% | Dominant, drives oscillation |
| I | 30.1% | Persistent trim offset (mean=-0.004) |
| D | 18.4% | High D/P ratio (0.60), providing damping |
| DFF | 2.5% | — |

Pitch I-term has persistent bias (mean=-0.0135) indicating CG offset.

### Recommended Changes (minimum for performance vehicle)

**Priority 1 — ANG_P reduction (breaks resonance loop, preserves rate response):**

| Parameter | Current | Recommended | Change |
|-----------|---------|-------------|--------|
| ATC_ANG_RLL_P | 27.25 | 20 | -27% (wider arms, lower inertia need more reduction) |
| ATC_ANG_PIT_P | 31.54 | 27 | -14% |

**Priority 2 — RAT_P reduction if oscillation persists:**

| Parameter | Current | Recommended | Change |
|-----------|---------|-------------|--------|
| ATC_RAT_RLL_P | 0.0562 | 0.048 | -15% |
| ATC_RAT_RLL_I | 0.0562 | 0.048 | Match P |

**Do NOT change:**
- FLTE (adds phase lag at oscillation frequency, counterproductive)
- SMAX (leave 0 for performance vehicle)
- RAT_D (D/P ratio is reasonable, changing risks HF noise)

**Rationale for roll needing more reduction than pitch:**
ANG_P reduction lowers coupling gain. Roll has higher plant gain from the airframe
asymmetry, so the same percentage ANG_P reduction has less effect on roll. Roll
needs ~2x the percentage reduction to achieve the same stability margin as pitch.

At ANG_RLL_P=20: 1 deg error still commands 20 deg/s correction (48ms settling).
The vehicle will still feel snappy — rate loop bandwidth is unchanged.

## Other Notes

- **Hover descent bias:** CRt mean=-34.7 cm/s during hover suggests slight altitude
  controller issue or wind-driven position corrections creating net descent
- **ThO stability:** std=0.0013 during hover — extremely stable throttle output
- **INS_ACC_VRFB_Z sign change:** Went from +0.09 to -0.117. Verify this matches
  the actual vibration behavior (should typically be positive for upward-biased
  vibration rectification)
