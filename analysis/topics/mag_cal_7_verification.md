# EK3_MAG_CAL=7 (GROUND_AND_INFLIGHT) Verification

Analysis from [log208](../logs/log208.md): first flight with EK3_MAG_CAL=7.

## Design Intent

EK3_MAG_CAL=7 was created to address fleet-wide compass interference at takeoff.
The mode provides:
- Compass body offset learning ON GROUND (handles battery-swap mag field changes)
- Learning FROZEN during takeoff and initial climb (protects vulnerable phase)
- Learning RESUMES after first in-flight yaw alignment (safe heading reference)

### Implementation

In AP_NavEKF3_Control.cpp `setWindMagStateLearningMode()`:

```cpp
bool magCalRequested = ...
    ((effectiveMagCal == MagCal::GROUND_AND_INFLIGHT) &&
     (!inFlight || (finalInflightYawInit && finalInflightMagInit)));

bool magCalDenied = !use_compass() || (effectiveMagCal == MagCal::NEVER) ||
    (onGround && effectiveMagCal != MagCal::ALWAYS &&
     effectiveMagCal != MagCal::GROUND_AND_INFLIGHT);
```

### MAG_CAL Mode Comparison

| Phase | 0:FLYING | 3:AFTER_CLIMB | 4:ALWAYS | **7:GROUND+INFLIGHT** |
|-------|----------|---------------|----------|----------------------|
| On ground | No | No | Yes | **Yes** |
| Takeoff/climb | Yes | No | Yes | **No** |
| After yaw init | Yes | Yes | Yes | **Yes** |
| Steady flight | Yes | Yes | Yes | **Yes** |

Mode 7 uniquely combines ground learning (for battery swaps) with takeoff protection
(for motor interference).

## Verification: MAG_FUSION State Transitions

XKFS MAG_FUSION values: 0=not fusing, 1=fuse yaw only (offsets frozen),
2=fuse mag fully (offsets learning).

### All Transitions (both cores identical timing)

| Time | Event | MAG_FUSION | Expected | Status |
|------|-------|-----------|----------|--------|
| 4.0s | Boot | 0 | Off | PASS |
| 5.1s | EKF init (ground) | 0->2 | Learn on ground | PASS |
| 50.2s | Takeoff detected | 2->1 | Freeze at takeoff | PASS |
| 107.6s | Land flight 1 | 1->2 | Learn on ground | PASS |
| 120.6s | Takeoff flight 2 | 2->1 | Freeze at takeoff | PASS |
| 128.4s | After IMU1 yaw alignment (128.3s) | 1->2 (C1 only) | Learn after init | PASS |
| 149.2s | Disarm | 1->2 (C0) | Return to ground | PASS |

All 10+ transitions match design specification exactly.

### Core Asymmetry

- Core 0: never received in-flight yaw alignment -> offsets frozen entire flight
- Core 1: yaw alignment at t=128.3s -> offsets resumed learning 0.1s later
- This asymmetry is an EKF decision based on innovation checks, not a MAG_CAL issue

## Verification: Body Offset Behavior (XKF2 MX/MY/MZ)

### Core 1 Detail — Takeoff Freeze

```
t=49.5  MX=0  MY=-6  MZ=-3    (motors start)
t=49.6  MX=0  MY=-6  MZ=-3    (ground effect, BAlt=-5.67m)
t=49.7  MX=0  MY=-6  MZ=-3    (climbing rapidly)
t=49.8  MX=0  MY=-5  MZ=-3    (1 mG step — last ground update)
t=50.0+ MX=0  MY=-5  MZ=-3    (FROZEN — stays constant through entire flight 1)
```

Total offset change during takeoff+climb: 1 mG (essentially zero, vs log201's 56 mG
runaway on Core 1 MX with MAG_CAL=4).

### Core 1 Detail — Post-Yaw Resume

```
t=128.3  MX=-8  MY=0   MZ=-3   (yaw alignment message logged)
t=128.4  MX=-8  MY=-5  MZ=-6   (learning resumes — MY and MZ jump immediately)
t=129.2  MX=-6  MY=-5  MZ=-6   (MX begins drifting)
t=131.5  MX= 0  MY=-7  MZ=-5   (learning continues)
t=134.5  MX=+8  MY=-9  MZ=-5   (converging toward in-flight values)
```

MX drift of +23 mG post-alignment indicates residual motor interference not fully
compensated by COMPASS_MOTCT=2. But learning happens AFTER yaw alignment when the
EKF has a good heading reference — so it converges correctly rather than diverging.

### Per-Phase Summary

| Phase | C0 Total Movement | C1 Total Movement | Learning? |
|-------|-------------------|-------------------|-----------|
| Pre-arm ground (5-44s) | 10 mG | 12 mG | Yes |
| Armed ground (44-49s) | 3 mG | 4 mG | Yes |
| Takeoff+climb (49-55s) | 0 mG | 1 mG | **No (frozen)** |
| Hover flight 1 (55-102s) | 0 mG | 0 mG | **No (frozen)** |
| Ground between (102-115s) | 6 mG | 11 mG | Yes |
| Flight 2 pre-yaw (115-128s) | 1 mG | 6 mG | Ground portion only |
| Flight 2 post-yaw (128-149s) | 0 mG (C0 frozen) | 38 mG (C1 learning) | C1 only |

## Yaw Divergence Comparison

### Log201 (MAG_CAL=4) vs Log208 (MAG_CAL=7)

| Metric | Log201 (MAG_CAL=4) | Log208 (MAG_CAL=7) |
|--------|-------------------|-------------------|
| Max yaw divergence | **24 deg** | **5.3 deg** |
| Yaw inconsistency error | YES | NO |
| Takeoff offset change | C1 MX: +2 to -58 mG | **0 mG (frozen)** |
| Ground learning | Yes | Yes |
| Core divergence mechanism | Motor interference runaway | Gyro bias drift |

### Divergence by Phase (Log208)

| Phase | C0-C1 Yaw | Drift Rate | Cause |
|-------|-----------|------------|-------|
| Pre-arm | 0 to -0.3 deg | 0.006 deg/s | Both learning, small drift |
| Flight 1 (both frozen) | -0.5 to -3.3 deg | 0.058 deg/s | Pure gyro bias difference |
| Ground between | -3.3 to -3.7 deg | 0.032 deg/s | Both learning |
| F2 pre-yaw (both frozen) | -3.7 to -3.3 deg | Improving | Slight convergence |
| F2 post-yaw (C1 learns) | -3.3 to +2.4 deg | 0.27 deg/s | C1 learning, C0 frozen |

The 0.058 deg/s drift during flight 1 is the natural limit when both cores have
frozen offsets — coming from IMU gyro bias differences between the two chips.

## Observations and Future Work

1. **Defense in depth:** MAG_CAL=7 + COMPASS_MOTCT=2 provides two layers of
   protection. MAG_CAL=7 prevents learning bad offsets during takeoff. MOTCT=2
   removes bulk interference, allowing cleaner post-alignment learning.

2. **Core alignment asymmetry:** Only Core 1 (IMU1) received in-flight yaw
   alignment in ~100s of flight. Core 0 stayed frozen at ground-learned values.
   For long flights, this means Core 0 accuracy degrades while Core 1 adapts.
   The EKF lane switching handles this, but it limits the benefit of dual cores.

3. **Post-yaw offset drift:** Core 1 learned +23 mG on MX after alignment,
   suggesting COMPASS_MOTCT=2 calibration needs refinement (non-linear
   current-interference relationship, or compass offset changes with altitude).

4. **Battery swap handling confirmed:** Ground learning was active between flights
   (t=107-120s) with offsets adapting. This validates the design goal of handling
   different battery magnetic signatures without power cycling.

## Verdict

EK3_MAG_CAL=7 works exactly as designed. Every state transition is correct, takeoff
protection is effective (0 mG offset change vs 56 mG runaway with MAG_CAL=4), and
the improvement is dramatic (5.3 vs 24 degree divergence). Ready for PR alongside
the Z-bias learning commits on pr-z-bias-squashed.
