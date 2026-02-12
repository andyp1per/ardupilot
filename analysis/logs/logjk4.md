# logjk4 Analysis

## Metadata
- **Date**: 2025-01-28
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk4.bin

## Parameters
- `EK3_SRC1_POSXY = 0`, `EK3_SRC1_VELXY = 5` (optical flow)
- `EK3_SRC1_POSZ = 1` (baro only), `EK3_SRC1_VELZ = 0`
- `RNGFND1_TYPE = 24` (rangefinder present, max ~17cm during flight)
- `INS_ACC_VRFB_Z = -0.199` (learned from previous flights)
- `TKOFF_GNDEFF_ALT = 0.8`
- `ACC_ZBIAS_LEARN = 2`

## Flight Profile
- **FAILED** — copter could not take off in AltHold
- Short flight attempt, pilot unable to maintain altitude
- Rangefinder showed vehicle only reached 9-17cm off ground

## Performance Metrics
- Baro motor-on noise std: **2.20m** (8.8x motors-off ratio)
- EKF reported 2.5m altitude while rangefinder showed 14-17cm
- BARO_THST_SCALE correlation: -0.37 (weak at 0-3m)
- BARO_THST_SCALE estimated: -155 Pa

## Sensor Status
- Rangefinder active but vehicle never left ground effect zone
- Optical flow active

## Analysis

### BUG: Ground Effect + Frozen Correction Conflict

**Two interacting problems caused takeoff failure:**

#### 1. Frozen correction creates phantom acceleration during ground effect

When the vehicle arms, the frozen correction immediately applies +0.199 m/s² downward. But on the ground, vibrations don't match the hover level the correction was learned at. Z-bias learning is inhibited during ground effect, so the EKF **cannot** absorb this error.

**VD acceleration from arm to throttle-up:**

| Time (s) | VD (m/s) | AZ bias | Note |
|----------|----------|---------|------|
| 7.50 | +0.032 | 0.01 | ARM — correction starts |
| 7.70 | +0.077 | 0.01 | +0.22 m/s² rate (matches correction) |
| 7.90 | +0.120 | 0.01 | AZ stuck at 0.01 — learning inhibited |
| 8.10 | +0.170 | 0.01 | VD growing unchecked |
| 8.30 | +0.221 | 0.01 | +0.25 m/s down before pilot adds throttle |

#### 2. Motor-induced baro noise (PRIMARY ISSUE)

Baro swings **-3m to +3m** while rangefinder shows vehicle only reached 9-17cm:

| Time (s) | Baro Alt (m) | EKF Alt (m) | Rangefinder (m) | Note |
|----------|-------------|-------------|-----------------|------|
| 8.76 | **-2.935** | -0.358 | 0.000 | Baro reads 3m below |
| 9.06 | +0.949 | -0.134 | 0.090 | Baro swings +4m in 0.3s |
| 9.76 | **+2.951** | 0.736 | 0.160 | Peak: baro 3m high, vehicle at 16cm |
| 10.26 | +1.690 | **2.096** | 0.140 | EKF at 2.1m, vehicle at 14cm |
| 10.56 | -1.718 | **2.534** | 0.000 | EKF peaks at 2.5m, vehicle back on ground |

#### Sequence of failure

1. **t=7.5s ARM**: Frozen correction applies +0.199 m/s² downward. Z-bias learning inhibited.
2. **t=7.5-8.5s**: VD drifts to +0.25 m/s (phantom sinking).
3. **t=8.5s Throttle up**: Baro explodes from prop wash (-3m to +3m swings).
4. **t=17.0s**: Ground effect flags clear at EKF alt=0.8m, but rangefinder shows 0.17m — **feedback loop!**
5. **t=17.0-17.8s**: Without protection, EKF trusts wild baro → altitude estimate reaches 2.5m.
6. **t=10.5s**: Altitude controller cuts throttle. Vehicle falls.

#### Post-disarm confirmation

After disarm, VD converges to -0.199 m/s — exactly the frozen correction value:
```
t=13.68  VD=-0.200  AZ=-0.01
t=14.06  VD=-0.194  AZ=-0.01
t=14.54  VD=-0.199  AZ=-0.01
```

### Proposed fix

Gate the frozen correction on ground effect state:
```cpp
// In correctDeltaVelocity():
const bool gndEffectActive = dal.get_takeoff_expected() || dal.get_touchdown_expected();
if (motorsArmed && !gndEffectActive) {
    delVel.z -= frontend->_accelBiasHoverZ_correction[accel_index] * delVelDT;
}
```

### Ground effect flags clear too early

`takeoff_expected` clears when EKF altitude crosses `TKOFF_GNDEFF_ALT`. But when baro noise is severe, EKF altitude is wrong — feedback loop where bad baro → wrong altitude → ground effect clears → EKF trusts bad baro.

**Potential fixes:**
1. Use rangefinder when available for ground effect threshold check
2. Add minimum time before allowing ground effect to clear
3. Require consistent altitude readings before clearing

## Recommendations
- Implement frozen correction gating on ground effect state
- Fix ground effect flag feedback loop
- Consider hardware baro relocation away from prop wash

## See Also
- [EK3 RNG_USE_HGT feedback loop](../topics/ekf_rng_use_hgt_feedback.md)
- [Baro thrust filter](../topics/baro_thrust_filter.md)
