# log7 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log7.bin
- **Sensors**: GPS, baro, dual IMU, rangefinder (Orient=25, downward)
- **Flight #**: 5

## Parameters

Same as log5/log6 — consecutive flight, no parameter changes.

| Parameter | Value | Notes |
|-----------|-------|-------|
| ACC_ZBIAS_LEARN | 3 | Always learn |
| INS_ACC_VRFB_Z | **-0.3896** (boot) | Learned from log6 — NOT zeroed |
| TKOFF_GNDEFF_TMO | 0 | NOT APPLIED (should be 3.0) |
| EK3_MAG_CAL | 7 | OK |
| COMPASS_MOTCT | 2 | OK |
| ARMING_CHECK | 64 | NOT APPLIED (should be 1) |
| MOT_THST_HOVER | 0.1459 | Learned from previous flights |
| BARO1_THST_SCALE | -100 | Active |
| INS_HNTCH_FREQ | 150 Hz | Notch center frequency |
| INS_HNTCH_MODE | 3 | ESC telemetry tracking |
| INS_HNTCH_OPTS | 6 | Multi-source + double notch |

## INS_ACC_VRFB_Z Evolution

| Phase | Value |
|-------|-------|
| Boot | **-0.3896 m/s^2** (carried from log6) |
| Disarm save | **-0.2963 m/s^2** |

The bias moved toward zero during this flight (from -0.39 to -0.30).

## Flight Timeline

| Time (s) | Event |
|----------|-------|
| 5.2 | Boot, STABILIZE mode |
| 5.3 | Motors emergency stopped (initial state) |
| 8.4 | Switched to ALT_HOLD (RC) |
| 13.8 | Arm failed — "Need Alt Estimate" + "EKF3 Yaw inconsistent 88 deg" |
| 17.4 | EKF3 both cores initialized |
| 17.5 | Initial yaw alignment, EKF yaw reset |
| 19.0 | Tilt alignment complete |
| **21.8** | **ARMED**, fence enabled, EKF yaw reset |
| 22.5 | SET_HOME |
| 25.6-25.8 | Ground mag anomaly, yaw re-aligned (both IMUs) |
| 27.5 | EKF origin set (both IMUs) |
| 32.0-32.1 | Second ground mag anomaly, yaw re-aligned |
| **35.4** | **IMU0 in-flight yaw alignment complete** |
| **42.4** | **IMU1 in-flight yaw alignment complete** |
| 76.1 | EKF3 IMU1 now using GPS |
| **141.8** | Switched to STABILIZE (landing) |
| 143.8 | Landing BAlt spike to **-6.44 m** |
| 144.3 | LAND_COMPLETE |
| **145.1** | **DISARMED**, EKF_ALT_RESET, field elevation 144m |

Total armed: ~123s. ALT_HOLD hover: ~120s (21.8s to 141.8s).

**Key differences from log6:**
- Two ground mag anomaly yaw resets (25s and 32s)
- IMU0 yaw alignment at +13.6s from arm (vs +7.4s in log6)
- IMU1 did not start using GPS until 76.1s (54s after arming)
- No EKF Failsafe — stable flight throughout

## Altitude Hold Performance

### CTUN Statistics (hover, t=35s to t=140s)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 2.369 m | **0.125 m** | 2.065 | 2.616 |
| DAlt (desired) | 2.456 m | 0.002 | 2.400 | 2.456 |
| BAlt (baro) | 2.290 m | **0.398 m** | 1.200 | 3.120 |
| ThO | 0.143 | 0.004 | 0.119 | 0.153 |
| CRt (cm/s) | 0.93 | 11.85 | -23 | 48 |
| Alt Error mean | **-0.087 m** | — | — | — |
| **Alt Error std** | **0.125 m** | — | — | — |

**12.5 cm altitude error std — best outdoor performance achieved on this vehicle.**

The EKF smooths the noisy baro (std=0.398m) down to 0.125m altitude std. The baro-to-EKF
noise rejection ratio is 3.2:1. ThO mean of 0.143 matches MOT_THST_HOVER of 0.146 closely.

## EKF Dual-Core Divergence

| Time (s) | C0 PD | C1 PD | Divergence |
|-----------|-------|-------|------------|
| 35 | -2.22 | -1.96 | 0.26 m |
| 50 | -2.43 | -2.35 | 0.09 m |
| 65 | -2.41 | -2.37 | 0.04 m |
| 80 | -2.26 | -2.00 | 0.26 m |
| 95 | -2.34 | -2.15 | 0.19 m |
| 110 | -2.51 | -2.52 | 0.01 m |
| **120** | **-2.57** | **-3.12** | **0.56 m** |
| 130 | -2.37 | -2.72 | 0.35 m |
| 140 | -2.36 | -1.98 | 0.38 m |

**Max hover divergence: 0.558 m** (at t=120s)

Core 1 shows much more altitude wander than Core 0:
- C0 PD range: 0.55 m (tight)
- C1 PD range: **1.36 m** (much noisier)

## XKF3 Innovations — IVD

**Core 0 IVD is completely stuck at -0.99 m/s from t=33s onward.** The IVN (0.37) and
IVE (-0.18) are also frozen. Position innovations (IPN, IPE, IPD) and yaw (IYAW)
continue to update normally. This suggests the velocity fusion update in Core 0 stopped
executing while position fusion continued.

| Core | Phase | IVD | Notes |
|------|-------|-----|-------|
| Core 0 | t=33s→end | **-0.99 (frozen)** | Worse than log4's -0.54 |
| Core 1 | t=35-75s | -0.97 (stuck) | Before GPS fusion |
| Core 1 | t=76-140s | -0.21 to +0.08 | **Healthy** (after GPS) |
| Core 1 | hover mean | -0.40 | std=0.48 |

**This is a regression** — Core 0 IVD is worse than log4's -0.54 constant. However,
Core 1 recovered after GPS fusion and showed healthy dynamic behavior.

## XKFS MAG_FUSION State Transitions

| Time (s) | C0/C1 State | Event |
|-----------|-------------|-------|
| 6.1 | 0 (inactive) | Pre-init |
| 17.6 | 2 (learning) | After yaw alignment |
| 24.9 | **1 (frozen)** | Takeoff freeze |
| **24.9→145.1** | **1 (frozen)** | **Stayed frozen entire flight** |
| 145.2 | 2 (learning) | Post-disarm resume |

**MAG_CAL=7 not working as expected in this flight.** Mag fusion froze at takeoff (state=1)
and never resumed to learning (state=2) during flight, despite successful in-flight yaw
alignments at t=35.4 and t=42.4. The expected post-yaw-init resumption did not occur.

This is different from log5/log6 where MAG_CAL=7 worked correctly.

## XKF2 Compass Offsets

| Time (s) | C0 MX/MY/MZ | C1 MX/MY/MZ |
|-----------|-------------|-------------|
| 10 | 0/0/0 | 0/0/0 |
| 25 | 37/-48/-5 | 36/-49/-5 |
| 40-140 | **37/-48/-5** | **36/-49/-5** |

**Compass offsets completely static during hover.** Locked in at t=25s and never changed.
This is consistent with MAG_FUSION remaining in frozen state (1) — no learning occurred.
Compare with log6 where offsets reached MY~300, MZ~-280 by convergence.

## Rangefinder

| Metric | Value |
|--------|-------|
| Messages | 2830 |
| Hover Dist min | 1.30 m |
| Hover Dist max | 3.24 m |
| Hover Dist mean | 2.34 m |
| Hover Dist std | 0.50 m |
| Status | 4 (good, all samples) |

Rangefinder active and providing good data, but not used by EKF (EK3_RNG_USE_HGT=-1).
The 0.50m std suggests either horizontal drift over uneven terrain or sensor noise.

## Temperature

| Sensor | Hover Min | Hover Max | Range |
|--------|-----------|-----------|-------|
| BARO | 50.9 C | 53.8 C | 2.9 C |
| IMU0 | 36.6 C | 39.0 C | 2.4 C |
| IMU1 | 37.6 C | 40.5 C | 2.9 C |

Small temperature ranges — vehicle was thermally soaked from previous flights.

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip |
|-----|----------------|----------------|----------------|------|
| IMU0 | 4.95 / 9.44 | 5.09 / 6.41 | 2.67 / 4.77 | 0 |
| IMU1 | 3.57 / 7.19 | 3.34 / 4.52 | 2.17 / 4.50 | 0 |

Slightly higher than log5/6 but well within limits. No clipping.

## GPS Quality

| Metric | Mean | Min | Max |
|--------|------|-----|-----|
| NSats | 20.6 | 15 | 21 |
| HDop | 0.56 | 0.53 | 1.05 |
| Status | 3 | 3 | 3 |

Excellent GPS throughout.

## Post-Landing

Landing BAlt spike to **-6.44m** at t=143.8s (ground effect). EKF smoothed to only -0.25m.

Post-disarm (1.4s of data):

| Time (s) | C0 PD | C0 VD | C1 PD | C1 VD | PD div |
|-----------|-------|-------|-------|-------|--------|
| 145.1 | +0.11 | -0.28 | +0.14 | -0.09 | 0.03 |
| 145.5 | -0.03 | -0.47 | +0.12 | -0.20 | 0.14 |
| 146.0 | -0.38 | -0.65 | -0.01 | -0.29 | 0.37 |
| **146.5** | **-0.73** | **-0.79** | **-0.16** | **-0.35** | **0.58** |

Post-disarm altitude drift still present. C0 drifts faster (-0.79 m/s VD) than C1 (-0.35).
CTUN Alt reaches +0.664m at end of log (EKF thinks it's climbing while on ground).
Divergence between cores: 0.576m and accelerating.

## Comparison: log3 → log4 → log5 → log6 → log7

| Metric | log3 | log4 | log5 | log6 | log7 |
|--------|------|------|------|------|------|
| Hover duration | 101s | 92s | 30s | 83s | **120s** |
| Alt error std | 14.2cm | 29.1cm | 37.5cm* | 23.5cm | **12.5cm** |
| IVD C0 | — | -0.54 stuck | +0.5 | -0.39 osc | **-0.99 stuck** |
| IVD C1 | — | — | — | -0.33 osc | -0.40 (healthy after GPS) |
| Core div (max) | 20cm | 69cm | 89cm | 140cm | **56cm** |
| MAG_FUSION | correct | — | correct | correct | **Stayed frozen** |
| BAlt spike | -6.15m | -6.80m | — | — | **-6.44m** |
| Post-land drift | +0.94m | -2.32m | -0.80m | -0.87m | 0.58m+ |
| Outcome | Good | Good | Failsafe | Good | **Best** |

*log5 was cut short by failsafe

## Key Findings

**Positive:**
1. **Best altitude performance: 12.5cm std** — beats all previous outdoor flights
2. 120s stable hover with no EKF issues
3. EKF noise rejection ratio 3.2:1 (baro std → alt std)
4. Vibrations excellent, zero clipping
5. VRFB learning moving toward zero (-0.39 → -0.30)

**Concerning:**
1. **Core 0 IVD frozen at -0.99** — velocity fusion appears stuck in Core 0
2. **MAG_FUSION stayed frozen** — MAG_CAL=7 did not resume learning after yaw init
3. Compass offsets static (37/-48/-5) vs log6's converged values (83/301/-228)
4. Post-disarm altitude drift persists (0.58m divergence in 1.4s, accelerating)
5. IMU1 GPS incorporation delayed to 76.1s (54s after arm)

## Recommendations

1. **Investigate Core 0 velocity fusion freeze.** This is a new issue — IVD/IVN/IVE
   all froze at t=33s while position innovations continued. May be related to the
   MAG_FUSION freeze.

2. **Investigate MAG_FUSION not resuming.** In log5/6 it worked; in log7 it froze at
   takeoff and never resumed. The difference may be related to the two ground mag
   anomaly events (vs one in log5/6).

3. Apply the still-missing changes: TKOFF_GNDEFF_TMO=3.0, ARMING_CHECK=1,
   INS_ACC_VRFB_Z=0 (explicit reset).

4. Despite the EKF anomalies, altitude performance was excellent. The controller
   and baro compensation are working well.

## See Also
- [log5](log5.md) — same vehicle, flight 3 (EKF failsafe)
- [log6](log6.md) — same vehicle, flight 4
- [log3](log3.md) / [log4](log4.md) — pre-update baseline
