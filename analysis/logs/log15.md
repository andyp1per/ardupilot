# log15 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log15.bin
- **Sensors**: GPS (u-blox SAM-M10Q), baro, dual IMU, rangefinder, optical flow (no data)
- **Flight #**: 9 (longest AltHold flight)

## Parameters — Changes from Log14

Only learned values changed:

| Parameter | Log14 | Log15 | Notes |
|-----------|-------|-------|-------|
| MOT_THST_HOVER | 0.1426 | **0.1509** | +5.8% (learned up, now highest) |
| INS_ACC_VRFB_Z | -0.3744 | **-0.377** | IMU0 stable |
| INS_ACC2_VRFB_Z | -0.2401 | **-0.201** | IMU1 converging toward zero |

## Flight Timeline

| Offset (from ARM) | Event |
|--------------------|-------|
| **+0.0s** | **ARMED** (AltHold) — EKF alt = **-0.44m** (negative!) |
| +0→+6s | **Takeoff difficulty** — controller locked negative alt target, output hover throttle while on ground |
| +7→+9s | Pilot lowered/reversed stick (frustration) |
| **+10→+11s** | **Pilot jammed stick to ~90%** — forced ThO to 17%, vehicle finally airborne |
| +12s | Pilot reduced to mid-stick, vehicle climbed to ~2.7m and stabilized |
| **+3.4s** | **EKF3 lane switch C0→C1** |
| +4.0s | EKF3 origin set |
| +11.7s | Ground mag anomaly (both cores) |
| **+15.3s** | **In-flight yaw alignment** (IMU0) — compass innovations drop to zero |
| +19.0s | In-flight yaw alignment (IMU1) |
| +168.0s | GPS 3D fix (5 sats, HDop=95 — marginal) |
| **+172.8s** | **Mode: Stabilize** (landing) |
| **+177.2s** | **DISARMED** |

Total flight: 177s. AltHold: **173s** — longest AltHold flight on this vehicle.

**GPS nearly absent** — 3D fix only at +168s (9s before landing), marginal quality.
Velocity innovations frozen for nearly entire flight. EKF operated on
baro + rangefinder + compass only.

## Altitude Hold Performance

### CTUN Statistics (stable hover, ~150s after settling)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | ~1.8 m | — | — | — |
| ThO | 0.152 | — | — | — |
| **Alt Error std** | **0.071 m** | — | — | — |

**7.1cm alt error std — 54% improvement over log11, slightly better than log14 (7.7cm).**
Best altitude hold performance measured on this vehicle. Achieved without GPS velocity
fusion for nearly the entire flight.

## Vibrations

| IMU | VibeX Mean | VibeY Mean | VibeZ Mean | Clip |
|-----|-----------|-----------|-----------|------|
| IMU0 | 3.6 | 5.1 | 2.7 | **6** |
| IMU1 | — | — | — | **3** |

Slight clipping appeared (9 total events) — first time since log7. Not critical but
a change from the zero-clip flights in log11 and log14.

## XKF3 Innovations — COMPASS BREAKTHROUGH

### Velocity Innovations — Frozen (no GPS)

Velocity innovations frozen from ~T+10s for both cores — no GPS velocity fused for
nearly the entire flight (GPS fix only at +168s with marginal quality).

### Magnetometer Innovations — NEAR ZERO (MASSIVE IMPROVEMENT)

| Field | Log11 | Log14 | Log15 | Change |
|-------|-------|-------|-------|--------|
| IMX | +103 | +108 | **+0.2** | **-103 mG** |
| IMY | -43 | +14 | **-0.4** | **-43 mG** |
| IMZ | -263 | -252 | **-1.4** | **-261 mG** |

**Compass innovations dropped from ~275 mGauss residual to ~1.5 mGauss during hover.**
This is a fundamental change in EKF behavior.

### What Changed

The key difference is the **MAG_FUSION mode transition**:

1. T+0 to T+8s: YawOnly (pre-flight)
2. T+8.9s to T+13.5s: **3D fusion** briefly (detects large innovations)
3. **T+15.3s onward: back to YawOnly for entire hover**

In YawOnly mode, the EKF uses only the yaw component of the compass, effectively
ignoring the motor-interference-corrupted IMX/IMZ components. The innovations are near
zero because the EKF yaw estimate and compass yaw agree well.

The in-flight yaw alignment at +15.3s successfully resolved the ground mag anomaly,
and MAG_CAL=7 correctly settled into YawOnly mode for the rest of the flight.

**In log11 and log14, the EKF was apparently stuck in 3D fusion mode, trying (and
failing) to reconcile the full 3D compass field with enormous motor interference.
In log15, MAG_CAL=7 correctly detected the interference and downgraded to YawOnly.**

Raw motor interference is still ~293 mGauss — the compass hardware situation hasn't
changed. The EKF is just handling it correctly now.

SM (mag variance) peaked at 1.62 during the first 5s anomaly period (54/250 samples >1.0
in first 25s), then settled to 0.03 during hover.

## XKF4 Variance Ratios

| Core | SH mean | SH max | SM max (hover) |
|------|---------|--------|----------------|
| C0 | normal | — | 0.03 |
| C1 | normal | — | 0.03 |

**SH > 1.0: zero during hover.** SM peaks only during initial ground mag anomaly.

## Rangefinder

| Metric | Value |
|--------|-------|
| Status Good | **100%** |
| Dist mean | 1.89 m |

Rangefinder healthy throughout hover. Contributing to altitude hold via EK3_RNG_USE_HGT=-1.

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 52 C | 61 C | 9 C |
| IMU0 | 37 C | 46 C | 9 C |
| IMU1 | 39 C | 48 C | 9 C |

Vehicle was warm from prior flights. 9°C range during flight.

## Motor Outputs

Similar pattern to previous flights — C1/C3 high, C2/C4 low.

## ESC RPM

Mean hover RPM: ~11,790 (197 Hz fundamental). Consistent with log14.
**ESC current telemetry: 0.00A** — ESCs not reporting current. COMPASS_MOTCT=2 falls
back to throttle-based scaling.

## Battery

- Hover voltage: 20.28V (6S, ~3.38V/cell)
- Hover current: 8.01A
- Total used: 379 mAh

## AccZ Bias

| Core | AZ mean | VRFB |
|------|---------|------|
| C0 | -0.024 | -0.377 |
| C1 | +0.026 | -0.201 |

Small AZ values, well-managed. VRFB continuing to converge toward zero.

## Post-Landing

- BAlt swung **8.5m** during descent (-6.66m to +1.87m) — severe baro-thrust coupling
- EKF alt diverged to -3.4m (C0) / -2.7m (C1) at disarm
- **Recovered** to -1.0m (C0) / -0.6m (C1) within 2s as baro stabilized
- The "drift" is actually EKF **recovery** from landing-induced baro errors

## Takeoff Difficulty — Negative EKF Altitude at ARM

### Problem

Pilot reported difficulty getting off the ground in AltHold. Raising throttle stick
to mid/high produced minimal response — vehicle barely lifted off after ~10s of struggle.

### Root Cause

**EKF altitude was -0.44m at ARM.** The barometer consistently reported negative altitude
before and after ARM (-0.33m to -0.39m pre-ARM). AltHold locked this negative value as the
desired altitude target (DAlt = -0.44m). The controller output hover throttle (~13%) to
maintain the negative target — which meant it was fighting the pilot's climb commands.

### Takeoff Sequence

| Time | Stick (PWM) | ThO | Alt (m) | DAlt (m) | RFND (m) | Notes |
|------|-------------|-----|---------|----------|----------|-------|
| +0.0s | 987 | 0.000 | -0.44 | -0.44 | 0.00 | ARM, idle |
| +1.5s | 1490 | 0.000 | -0.38 | -0.38 | 0.00 | Stick raised, spooling |
| +4.0s | 1481 | 0.123 | -1.13 | -1.18 | 0.13 | ThO at hover, alt went MORE negative |
| +5→6s | 1481 | 0.12 | -0.8 | -1.04 | 0.16-0.29 | Bouncing, controller resists |
| +7→9s | 1481 | ~0 | -0.6 | -1.03 | 0.00 | Pilot LOWERED stick |
| **+10s** | **1953** | 0.033 | -0.55 | -0.69 | 0.01 | **Pilot jammed to ~90%** |
| +11s | 1953 | 0.168 | -0.66 | — | 0.17 | Vehicle airborne, 118 cm/s climb |

### Contributing Factors

1. **Baro offset**: Baro reported -0.35m on ground. Prop wash increased pressure further
   (+4 Pa ≈ -0.3m) after motors spooled, driving alt estimate more negative
2. **No GPS**: NSats=0, so no absolute altitude correction available
3. **Rangefinder correctly showed 0.00m** on ground, but EKF altitude estimate was
   dominated by baro during this phase
4. **EKF origin likely set from bad GPS altitude** during initialization (GPS.Alt=-17m)

### Implications

This explains why the pilot needed ~90% stick to take off — the altitude controller was
actively resisting climb because it thought it was already above the target altitude. Once
airborne and above the negative target, the controller worked normally (settling at ~15%
hover throttle). This is a pre-ARM baro calibration issue, not a vehicle performance issue.

---

## Key Findings

### Major Breakthrough: Compass Innovations Near Zero

**The #1 issue on this vehicle (compass interference) appears resolved at the EKF level.**
Compass innovations dropped from 275 mGauss (log14) to 1.5 mGauss (log15). The raw motor
interference is unchanged (~293 mGauss) — the improvement is entirely due to MAG_CAL=7
correctly transitioning to YawOnly fusion mode after the initial ground mag anomaly.

This validates the MAG_CAL=7 approach: the EKF detects that 3D compass fusion produces
large innovations, downgrades to yaw-only, and the system works well. The inconsistency
previously seen (log7 where MAG_FUSION froze, log11/14 where innovations stayed stuck)
suggests this behavior is not yet 100% reliable.

### Other Findings

1. **Takeoff difficulty** — EKF alt -0.44m at ARM, pilot needed ~90% stick to overcome altitude controller resistance (baro calibration issue)
2. **7.1cm alt error std** — best performance, achieved without GPS velocity fusion
3. **173s AltHold** — longest flight on this vehicle
4. GPS nearly absent (3D fix only 9s before landing, marginal quality)
5. Slight clipping appeared (9 events total) — first since log7
6. Lane switch C0→C1 at +3.4s — earlier than log11 (+10s)
7. Post-landing: BAlt swung 8.5m, EKF diverged to -3.4m, recovered in 2s
8. ESC current telemetry not working (0.00A) — COMPASS_MOTCT=2 using throttle fallback
9. Optical flow still zero samples

## See Also
- [log14](log14.md) — previous flight (compass still stuck)
- [log11](log11.md) — earlier flight for comparison
