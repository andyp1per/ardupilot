# EKF3 Analysis - CLAUDE.md

This file tracks our EKF3 analysis work, including methodology, implementation notes, and rules.

**Per-log analysis and cross-cutting findings have been moved to [`analysis/`](../../analysis/README.md).**

## Plan

*Current objectives and next steps for EKF3 analysis.*

- [x] Initial exploration of EKF3 architecture
- [x] Understand state vector and covariance structure
- [x] Document log message formats
- [x] Learn to use Replay tool for EKF analysis
- [x] Analyze altitude hold issue in log4.bin
- [x] Understand ground effect compensation mechanism
- [x] **Investigate improvement options for low-altitude hover ground effect**
- [x] **Implemented: TKOFF_GNDEFF_ALT parameter**
- [x] Compare altitude estimators: IMU integration, DCM (AHR2), EKF
- [x] Analyze EKF velocity drift without GPS
- [x] **Investigate IMU temperature effects on EKF accel bias**
- [x] **Analyze takeoff abruptness and recommend parameter changes**
- [x] **Test EK3_ABIAS_P_NSE via Replay** - No effect without velocity sensor
- [x] **Analyze log5.bin** - Velocity drift is root cause
- [x] **Analyze vibration compensation** - Won't help indoor no-GPS
- [x] **Ground motor test (log6.bin)** - +0.084 m/s² Z-axis shift from motors
- [x] **Inhibit Z-axis bias learning during ground effect**
- [x] **Flight test (log8.bin)** - Z-bias inhibition dramatically improved altitude hold
- [x] **Stationary zero velocity fusion** - Zero velocity when on ground and disarmed
- [x] **Fix hover Z-bias not loaded at boot** - Deferred loading via `one_hz_loop()`
- [x] **Move hover Z-bias learning to Copter** - Learning in ArduCopter/Attitude.cpp
- [x] **Validate hover Z-bias end-to-end** - log2/log3 confirm full cycle
- [ ] **Fix post-landing EKF divergence** - Error accumulates during landing ground effect
- [ ] **Fix ground effect + frozen correction conflict** - See [logjk4 analysis](../../analysis/logs/logjk4.md)
- [ ] Investigate velocity sensor options (optical flow, rangefinder)
- [ ] Map out sensor fusion flow
- [ ] Document key algorithms and their purposes

## EKF Analysis Methodology — MUST READ

**Rule 1: No theories without data.** Do not speculate about EKF behavior based on code reading alone. The EKF is a complex dynamic system where multiple states interact. Theories MUST be validated against actual log data before being presented as explanations.

**Rule 2: Always cross-check with multiple sensors.** When analyzing altitude/position issues, compare ALL available sources:
- EKF estimate (XKF1 PD/VD)
- Barometer (BARO.Alt)
- Rangefinder (RFND.Dist)
- GPS altitude (GPS.Alt)
- Raw IMU (IMU.AccZ)

If your theory predicts the vehicle is at 2.5m but the rangefinder shows 17cm, your theory is wrong.

**Rule 3: Extract data first, theorize second.** Before forming hypotheses:
1. Extract the relevant log messages
2. Align timestamps and create comparison tables
3. Identify anomalies in the DATA
4. Only then form hypotheses that explain ALL observations

**Rule 4: Use Replay for controlled experiments.** To test parameter/code changes:
1. Run original log through Replay for baseline
2. Modify parameter/code
3. Run Replay again and compare XKF outputs

**Rule 5: Check ground truth.** Rangefinder > GPS > baro during ground effect.

**Log analysis checklist:**
- [ ] Extract PARM values for relevant parameters
- [ ] Extract ARM/DISARM events and flight mode changes
- [ ] Extract multiple altitude sources (BARO, RFND, XKF1.PD, GPS if available)
- [ ] Extract XKF4 status flags (takeoff_expected, touchdown_expected)
- [ ] Cross-check all sources before forming theories

## EKF3 Reference

### State Vector (24 states)

| Index | State | Description | Units |
|-------|-------|-------------|-------|
| 0-3 | `quat` | Quaternion (w,x,y,z) - rotation NED to body | - |
| 4-6 | `velocity` | Velocity NED (N,E,D) | m/s |
| 7-9 | `position` | Position NED (N,E,D) | m |
| 10-12 | `gyro_bias` | Gyro bias body (x,y,z) | rad |
| 13-15 | `accel_bias` | Accel bias body (x,y,z) | m/s |
| 16-18 | `earth_magfield` | Earth magnetic field NED | Gauss |
| 19-21 | `body_magfield` | Body magnetic field (x,y,z) | Gauss |
| 22-23 | `wind_vel` | Wind velocity NE | m/s |

### Log Messages

| Message | Description |
|---------|-------------|
| **XKF1** | Main outputs: attitude, velocity NED, position NED, gyro bias |
| **XKF2** | Accel bias, wind, magnetic field, drag innovations |
| **XKF3** | Innovations: velocity, position, mag, yaw, airspeed |
| **XKF4** | Variances/health: sqrt variances, tilt error, resets, faults, timeouts |
| **XKF5** | Optical flow, rangefinder, HAGL, error magnitudes |
| **XKFM** | On-ground-not-moving diagnostics |
| **XKFS** | Sensor selection: indices, source set, fusion state |
| **XKV1** | State variances V00-V11 |
| **XKV2** | State variances V12-V23 |

### XKF4 Status Fields

**TS (Timeout Status):** Bit 0=Position, 1=Velocity, 2=Height, 3=Mag, 4=Airspeed, 5=Drag

### Log Analysis Rules

1. **Core Index (C field):** All XK* messages have `C` field for EKF core (0 or 1). Always filter by core.
2. **Timestamp Units:** TimeUS is microseconds since boot.
3. **Angles:** Roll/Pitch/Yaw in XKF1 are centidegrees. GX/GY/GZ are milliradians.
4. **Primary Core:** XKF4.PI indicates which core is primary.

## Tools

### Replay Tool

```bash
./waf configure --board sitl
./waf --targets tool/Replay
./build/sitl/tool/Replay --force-ekf3 ./logfile.bin
```

Output log in `logs/` — contains original (C=0,1) and replayed (C=100,101) data.

Use `--parm NAME=VALUE` for parameter overrides. Limitations:
- Only EKF parameters (not Copter-level like TKOFF_GNDEFF_ALT)
- Ground effect flags are NOT re-computed during replay

### Z-Bias Analysis Script

```bash
python3 libraries/AP_NavEKF3/tools/ekf_bias_analysis.py <logfile.bin>
python3 libraries/AP_NavEKF3/tools/ekf_bias_analysis.py <logfile.bin> --plot
```

## Implementation Notes

### Ground Effect Compensation

**Key file:** `ArduCopter/baro_ground_effect.cpp`

The EKF has two ground effect mechanisms:
1. **Innovation flooring:** Limits negative baro corrections (line ~1014 in PosVelFusion.cpp)
2. **Noise scaling:** Baro noise × 4x (`gndEffectBaroScaler = 4.0`)

Both only active when `takeoff_expected` OR `touchdown_expected` is true.

**TKOFF_GNDEFF_ALT parameter:** Configurable altitude threshold for ground effect re-activation.

### Z-Axis Bias Inhibition

**Problem:** Motor thrust creates +0.08 m/s² AccZ offset on ground. EKF learns this as bias, which is incorrect for flight.

**Fix (commit `6bc5565643`):** Inhibit Z-axis accel bias learning (state 15) during ground effect:
```cpp
const bool gndEffectActive = dal.get_takeoff_expected() || dal.get_touchdown_expected();
const bool zAxisInhibit = (i == 15) && gndEffectActive;
```

**Current Z-bias learning state by phase:**

| Scenario | Z-bias Learning | Mechanism |
|----------|-----------------|-----------|
| Stationary on ground, disarmed | Enabled via zero velocity fusion | Strongly observable |
| Armed on ground (motors spinning) | Inhibited | Ground effect flag |
| Takeoff (below TKOFF_GNDEFF_ALT) | Inhibited | Ground effect flag |
| Hover (above TKOFF_GNDEFF_ALT) | Enabled | Weakly observable from baro |
| Flying with GPS Z velocity | Enabled | Strongly observable |
| Landing (slow descent) | Inhibited | Ground effect flag |

### Stationary Zero Velocity Fusion

**Problem:** In AID_RELATIVE mode (optical flow), Z-bias was unobservable when disarmed.

**Fix (commit `0ff35c20b4`):** Fuse synthetic zero velocity when `onGround && !motorsArmed`, regardless of aiding mode. Makes Z-bias strongly observable when stationary.

### Hover Z-Bias Learning (INS_ACCx_VRFB_Z)

Captures EKF's learned accel bias during stable hover, saves for subsequent flights.

**Data flow:**
1. **Boot**: INS params → Copter's `_hover_bias_learning[]` → EKF's frozen correction (deferred via `one_hz_loop()`)
2. **Armed**: Frozen correction applied at IMU level in `correctDeltaVelocity()`
3. **Hover**: Copter captures total bias = EKF_residual + frozen_correction (2s filter)
4. **Disarm**: Total bias saved to INS params (if `ACC_ZBIAS_LEARN=2`)

**Key files:**
- `ArduCopter/Attitude.cpp` — learning logic
- `AP_AHRS.h/cpp` — abstraction layer
- `AP_NavEKF3_core.cpp:correctDeltaVelocity()` — correction application
- `AP_InertialSensor.cpp` — parameter storage (INS_ACC_VRFB_Z, INS_ACC2_VRFB_Z, etc.)

**Known bug:** Frozen correction applied during ground effect when vibrations don't match hover. See [logjk4 analysis](../../analysis/logs/logjk4.md) for details and proposed fix.

### Post-Landing EKF Divergence (TODO)

After landing with ground effect, innovation clamping accumulates position error. When clamping stops at disarm, the large error drives velocity drift. Potential solutions:
1. Gradual innovation limit release after landing
2. Position reset on landing
3. Stronger zero velocity fusion when stationary
4. Extended ground effect protection after landing

### Ground Effect Rules

1. Ground effect compensation is controlled by Copter, not EKF
2. Ground effect causes positive pressure (baro reads LOWER than actual)
3. When in doubt, enable compensation (sluggish but stable > divergent)
4. Ground effect height is vehicle-specific (0.3-1.0m for small multirotors)

### EKF Source Rules

1. Source parameters only matter when sensor provides data
2. Timeout flags are informational, not necessarily a problem
3. Accel bias observability requires velocity measurements
4. Without GPS or optical flow Z velocity, vertical bias is poorly observable

## Flight Log Analysis

All per-log analysis and cross-cutting findings are in the [`analysis/`](../../analysis/README.md) directory:
- Per-log files: `analysis/logs/logjk1.md` through `logjk9.md`, `logtd1.md` through `logtd7.md`
- Topics: `analysis/topics/` — throttle vs current, baro thermal drift, RNG_USE_HGT feedback, THST_FILT
- Earlier development logs (log1-log12): summarized in `analysis/README.md`
