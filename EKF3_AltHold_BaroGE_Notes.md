# EKF3 Altitude Hold — Baro Ground Effect Investigation

## Vehicle
- TD-Matek-5Inch: 5" quad, MatekH743-bdshot, baro sees -4 to -8m prop wash
- TD-MicoAir-2: MicoAir743AIOv2

## Problem Statement

Small copters with high disc loading produce severe barometric pressure
disturbance (prop wash) during motor spool-up and low-altitude flight.
On the TD-Matek-5 this is -4 to -8m of apparent baro altitude drop.
Multiple EKF3 code paths failed to handle this correctly, causing altitude
jumps, overshoot, and inability to take off.

## Root Causes Found (in discovery order)

### 1. resetHeightDatum() did not flush delay buffers (log15)

`resetHeightDatum()` zeroed `stateStruct.position.z` but did not update
the output buffer (`storedOutput[]`), complementary filter state
(`vertCompFiltState`), `baroHgtOffset`, or the baro observation delay
buffer (`storedBaro`). Stale pre-calibration samples in the ~200ms delay
buffer were fused immediately after the reset, pulling altitude away from
zero.

**Symptom**: -0.44m EKF altitude at arming despite datum reset.

### 2. ResetHeight() fired during ground effect (log26)

After fixing the delay buffer, fresh baro samples during motor spool-up
were still corrupt from prop wash. The existing 4x noise scaling
(`gndEffectBaroScaler`) was insufficient for -8m errors, and innovations
exceeded the height test gate. After 5s of failed tests,
`hgtTimeout` triggered `ResetHeight()`, forcing the EKF state to the
corrupt baro measurement.

**Symptom**: 3.25m position-down jump at ARM+2.7s.

### 3. resetHeightDatum() never called when home already set (log32)

The arming code only called `resetHeightDatum()` in the `!home_is_set()`
path. When home was already set (from GPS), the call was skipped. Baro
drift between home-set and arming accumulated as permanent altitude error.

**Symptom**: Vehicle would not take off — accumulated -8m baro drift locked
the AltHold target below ground.

### 4. Stale lastHgtPassTime_ms after datum reset (log33)

After `storedBaro.reset()` cleared the delay buffer, no baro data was
available for the time it takes to refill (~200ms). But
`lastHgtPassTime_ms` was not updated, so the elapsed time since the last
successful height fusion continued to grow. By the time fresh baro data
arrived, `hgtTimeout` was already true, and the corrupt ground-effect
baro was force-fused.

**Symptom**: 1.37m position-down jump at ARM+3.8s.

### 5. public_origin altitude not synced (log35)

`resetHeightDatum()` updated `EKF_origin.alt` but not `public_origin.alt`
(which is `frontend->common_EKF_origin`). The `getPosD()` function adds
`(public_origin.alt - EKF_origin.alt) * 0.01` to the output, so a stale
`public_origin` created a constant offset in reported altitude.

**Symptom**: +2.25m position-down offset on second arming cycle.

### 6. 4x baro noise scaling insufficient for extreme prop wash (log36)

The existing `gndEffectBaroScaler = 4.0` reduces the Kalman gain by ~4x,
but with -8m prop wash the dead zone floor of -0.5m still leaks through
at ~0.055m per fusion cycle. Over a full GE window this accumulates to
2-3m of contamination.

**Symptom**: +3.51m position-down jump from gradual baro contamination
during ground effect.

### 7. resetHeightDatum() guard used stale onGround flag (log37)

The `!onGround` guard in `resetHeightDatum()` prevented the reset from
executing during arming. `onGround` is set to `false` by
`detectFlight()` whenever `motorsArmed` is true, but `motorsArmed` is
updated from `dal.get_armed()` which reflects the arming state. During the
arming sequence, motors are armed before the datum reset is called, so
`onGround` transitions to false before the reset has a chance to run.

**Symptom**: `resetHeightDatum()` silently returned false on 2nd arm,
leaving +0.727m altitude offset.

### 8. hgtTimeout accumulated during intentional baro inhibition (log38)

When baro fusion was fully inhibited during ground effect
(EK3_GND_EFF_DZ < 0), `lastHgtPassTime_ms` was not updated. After the
5s GE window ended, hgtTimeout was immediately true, triggering
`ResetHeight()` with stale baro data.

**Symptom**: Vehicle rose to 5.16m with PILOT_TKOFF_ALT=150 (1.5m target).
ResetHeight() fired at GE boundary causing target shift.

### 9. Ground effect timeout counted from arming not liftoff (log39)

The GE timer started when throttle was raised, not when the vehicle
actually left the ground. With TKOFF_GNDEFF_TMO=1.5s, the timeout
expired during motor spool-up before the vehicle lifted off, leaving the
worst prop wash period unprotected.

**Symptom**: GE protection ended at T+1.5s but vehicle did not lift until
T+2.5s. Worst baro contamination occurred after protection ended.

### 10. Full baro inhibition causes IMU drift (log40)

Full baro inhibition (EK3_GND_EFF_DZ=-1 in the old behavior) prevented
all baro contamination but left the EKF running on IMU alone for ~1.6s
after liftoff. Accelerometer bias caused ~0.55m altitude drift. The
controller commanded continued climb thinking the vehicle was below
target. When baro resumed, the step correction caused overshoot.

**Symptom**: Vehicle reached 3.31m RFND with 1.5m target.

### 11. EK3_RNG_USE_HGT blending blocks resetHeightDatum on ground (log47)

`resetHeightDatum()` has an early-return guard:
`if (activeHgtSource == RANGEFINDER) return false`. When
`EK3_RNG_USE_HGT > 0`, the height source blending logic in
`selectHeightForFusion()` sets `activeHgtSource = RANGEFINDER` whenever
the vehicle is below the blend transition height and terrain data is
trusted. On the ground (0m AGL), this is always true when a rangefinder
is present — even though the configured primary source (`EK3_SRC1_POSZ`)
is baro.

In log 47, the user powered on, jogged around for ~100s (43m total
distance), then set the drone down and armed. During the jogging phase,
baro drifted -1.787m (from handling/altitude changes), accumulating
+1.488m of PD offset. At arming, `resetHeightDatum()` was called but
returned false because `activeHgtSource == RANGEFINDER` from blending.
Both EKF cores retained their pre-arm PD offset.

**Symptom**: PD=+1.488m at arming instead of ~0. Vehicle took ~10s to
climb from the offset to the target altitude, with RFND oscillating
between 1.3–2.9m (target 1.5m).

## Fixes Applied

### AP_NavEKF3: resetHeightDatum() improvements
- Flush output state buffer (`storedOutput[].position.z`, `.velocity.z`)
- Reset complementary filter state (`vertCompFiltState.pos/vel`)
- Zero vertical velocity (`stateStruct.velocity.z`)
- Clear baro offset tracker (`baroHgtOffset = 0`)
- Flush baro delay buffer (`storedBaro.reset()`)
- Reset height timeout (`lastHgtPassTime_ms`, `hgtTimeout`)
- Sync `public_origin.alt = EKF_origin.alt`
- Fix guard: use `!onGround && motorsArmed` instead of `!onGround`

### AP_NavEKF3: suppress ResetHeight() during ground effect
- When `hgtTimeout` fires during `takeoff_expected || touchdown_expected`,
  skip `ResetHeight()` and set `fuseHgtData = false`

### AP_NavEKF3: EK3_GND_EFF_DZ negative value as noise floor
- When negative, use `|value|` as the baro observation noise in metres
  (variance = value^2) instead of the fixed 4x scaler
- Also use `|value|` for the dead zone size
- E.g. -8 gives 64x variance increase, K≈0.008, limiting per-sample
  contamination to ~0.004m while maintaining a weak altitude anchor
- Updated parameter range to -10..10 and documentation

### Copter: always reset EKF height datum on arming
- Call `ahrs.resetHeightDatum()` unconditionally, not just when home
  is unset. Set `arming_altitude_m = 0` after reset.

### Copter: count ground effect timeout from liftoff
- Reset GE timer while `ap.land_complete` so the timeout begins at
  actual liftoff regardless of how long motors spool on the ground.

### AP_NavEKF3: fix resetHeightDatum blocked by rangefinder blending
- The `activeHgtSource == RANGEFINDER` guard blocked datum resets on the
  ground when `EK3_RNG_USE_HGT` blending was active
- Changed guard: when `onGroundNotMoving` is true AND the configured
  primary source (`EK3_SRC1_POSZ`) is not rangefinder, allow the reset
  even if `activeHgtSource` is currently rangefinder from blending
- When airborne or moving, the original `activeHgtSource` check is
  preserved to prevent resets during actual rangefinder-primary flight

## Test Flight Log Summary

| Log | Key Observation | Fix Applied |
|-----|-----------------|-------------|
| 15  | -0.44m at ARM from stale delay buffer | Fix 1: flush buffers |
| 17  | Compass earth-field check blocked rearm (unrelated) | None |
| 26  | 3.25m PD jump from ResetHeight() during GE | Fix 2: suppress ResetHeight in GE |
| 32  | resetHeightDatum never called, vehicle would not take off | Fix 3: always reset on arm |
| 33  | 1.37m PD jump from stale hgtTimeout + corrupt baro | Fix 4: reset timeout + reject baro on hgtTimeout during GE |
| 35  | 2.25m offset from public_origin mismatch | Fix 5: sync public_origin.alt |
| 36  | 3.51m PD jump from 4x noise insufficient for -8m wash | Fix 6: full baro inhibition option (EK3_GND_EFF_DZ<0) |
| 37  | 0.727m offset — resetHeightDatum returned false (stale onGround) | Fix 7: guard uses motorsArmed |
| 38  | 5.16m altitude — hgtTimeout at GE boundary triggered ResetHeight | Fix 8: keep lastHgtPassTime current during inhibition |
| 39  | GE timeout expired before liftoff | Fix 9: count timeout from liftoff |
| 40  | 0.55m IMU drift from full inhibition, overshoot to 3.31m | Fix 10: noise floor instead of full inhibition |
| 41  | Immediate takeoff (14s), DZ=-1, hover 1.60m (target 1.5m) | Comparative: DZ=-1 adequate only with short warm-up |
| 42  | 2-min delay (99s), DZ=-1, hover 1.89m (+0.4m error) | Comparative: DZ=-1 error grows with warm-up time |
| 43  | 5-min delay (238s), DZ=-1, hover 2.55m (+1.1m error) | Comparative: DZ=-1 catastrophically undersized |
| 46  | 75s delay, DZ=-8, hover 1.34m (-0.2m error) | Validates fix 10: noise floor works regardless of warm-up |
| 47  | Jogged 43m pre-arm, PD=+1.488m at ARM (datum reset blocked) | Fix 11: allow datum reset when onGroundNotMoving |
| 50  | Pre-arm movement, PD=+0.010m at ARM, hover 2.25m | Validates fix 11: datum reset succeeds with movement |
| 51  | Pre-arm movement, PD=+0.000m at ARM, hover 1.59m | Validates fix 11: datum reset succeeds, hover on target |

## Comparative Analysis: Warm-up Time vs AltHold Performance

Logs 41–43 were flown on firmware with fixes 1–9 (`EK3_GND_EFF_DZ=-1`) to
test the effect of power-on-to-takeoff delay. Log 46 validates fix 10
(`EK3_GND_EFF_DZ=-8`).

### Test Conditions

| Log | Boot→ARM | Firmware | EK3_GND_EFF_DZ | Flight Duration |
|-----|----------|----------|----------------|-----------------|
| 41  | 14s      | 33d99563 (fix 9) | -1  | 111s |
| 42  | 99s      | 33d99563 (fix 9) | -1  | 113s |
| 43  | 238s     | 33d99563 (fix 9) | -1  | 146s |
| 46  | 75s      | fcacccc9 (fix 10) | -8 | 38s  |

### Results

| Log | Baro Prop Wash | RFND Hover (target=1.5m) | Error | Baro Drift (hover) |
|-----|----------------|--------------------------|-------|--------------------|
| 41  | -6.9m drop     | 1.60m                    | +0.1m | +1.36 m/min        |
| 42  | -6.5m drop     | 1.89m                    | +0.4m | +0.53 m/min        |
| 43  | -6.6m drop     | 2.55m                    | +1.1m | +1.72 m/min        |
| 46  | -7.6m drop     | 1.34m                    | -0.2m | -2.81 m/min        |

### Key Finding: EK3_GND_EFF_DZ=-1 is catastrophically undersized

All four logs produce **-6.5 to -7.6m** baro errors from prop wash. With
`EK3_GND_EFF_DZ=-1`, the dead zone clips only 1m of a ~7m innovation.
The remaining **~6m of baro error passes through to fusion**, contaminating
the EKF position estimate during spool-up.

With `EK3_GND_EFF_DZ=-8` (log 46), the noise floor approach gives Kalman
gain K≈0.008, limiting per-sample contamination to ~0.004m. The vehicle
hovers within 0.2m of the 1.5m target despite having the **worst** baro
prop wash of all four logs (-7.6m).

### "Progressively worse" root cause

The hover altitude error increases with longer warm-up time (0.1m → 0.4m
→ 1.1m). The mechanism:

1. **Baro contamination during spool-up** biases the EKF position negative
   (thinks vehicle is higher than reality) due to the inadequate dead zone
2. **The EKF then corrects using baro during flight**, but the baro's
   thermal drift rate differs depending on warm-up time
3. Different thermal states produce different post-takeoff convergence
   trajectories — some recover toward the target altitude, others diverge

This is entirely caused by `EK3_GND_EFF_DZ=-1` being too small. With
`-8`, the baro has negligible influence during ground effect, eliminating
the contamination regardless of warm-up time.

### PD step at ARM+3s (rangefinder blend transition)

All four logs show a sudden 0.6–1.0m PD step approximately 1.0–1.5s after
liftoff. This coincides with the vehicle climbing through the rangefinder
blend threshold (`EK3_RNG_USE_HGT=3%` of 30m max = 0.9m AGL). The EKF
transitions from rangefinder-blended height to pure baro height. This step
is normal EKF behavior and appears in all logs including log 46.

| Log | Step Time | Step Size | PD After Step | RFND at Step |
|-----|-----------|-----------|---------------|--------------|
| 41  | ARM+2.7s  | -0.72m    | -1.454m       | 1.11m        |
| 42  | ARM+3.3s  | -0.57m    | -1.466m       | 1.16m        |
| 43  | ARM+3.0s  | -0.61m    | -1.483m       | 1.04m        |
| 46  | ARM+3.2s  | -0.98m    | -1.819m       | 1.26m        |

### Datum reset confirmation

All four logs confirm `resetHeightDatum()` works correctly at arming:

| Log | PD at ARM |
|-----|-----------|
| 41  | -0.004m   |
| 42  | +0.004m   |
| 43  | +0.012m   |
| 46  | +0.001m   |
| 47  | +1.488m (FAILED — datum reset blocked by rangefinder blending) |
| 50  | +0.010m (fix 11 validated — reset succeeds with pre-arm movement) |
| 51  | +0.000m (fix 11 validated — reset succeeds with pre-arm movement) |

### Analysis note: ARM message type captures both arm AND disarm

The ArduPilot `ARM` log message type fires for **both** arming and
disarming events. Analysis scripts must use `EV id=10` (ARMED event) to
find the actual arm time, not the ARM message type. Using the wrong
timestamp centers all analysis on the disarm event, producing misleading
results.

## Recommended Parameters

For a 5" quad with ~8m baro prop wash:
- `EK3_GND_EFF_DZ = -8` (noise floor matching max prop wash)
- `TKOFF_GNDEFF_TMO = 1.5` (GE timeout after liftoff, seconds)
- `TKOFF_GNDEFF_ALT = 1.5` (GE altitude threshold, metres)
