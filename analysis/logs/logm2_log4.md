# logm2_log4 Analysis

## Metadata
- **Date**: 2026-02-17
- **Vehicle**: TD-MicoAir-2 (MicoAir743v2, APJ=1179)
- **Firmware**: V4.6.3v2-SFD (33d99563)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log4.bin (20 MB)
- **Sensors**: Optical flow (EK3_SRC1_VELXY=5), baro, dual IMU, rangefinder (MAVLink, 7m max)
- **No GPS, no compass** — fully GPS-denied
- **Board orientation**: YAW45 (AHRS_ORIENTATION=1)
- **Purpose**: AltHold test after parameter changes from logm2_4/5/6 sessions

## Outcome

**AltHold failure — vehicle hit ceiling.** Core 1 (the forced primary) lost optical flow
aiding within 4 seconds of takeoff and flew dead reckoning for 38 seconds. When the EKF
detected the problem and reset, the altitude jump caused the controller to command a
full-throttle climb into the ceiling. Pilot saved by switching to Stabilize.

## Parameters — Changes from logm2_6

| Parameter | logm2_6 | This flight | Reason |
|-----------|---------|-------------|--------|
| EK3_IMU_MASK | 1 | **3** | Use both IMUs (logm2_5 showed IMU0 clipping) |
| EK3_PRIMARY | (default 0) | **1** | Force Core 1 / IMU1 as primary |
| EK3_RNG_USE_HGT | -1 | **10** | Rangefinder below 10% max range (0.7m) |
| ACC_ZBIAS_LEARN | 0 | **3** | Enable Z-bias learning |
| TKOFF_GNDEFF_ALT | (default) | **0.5** | Ground effect altitude |
| TKOFF_GNDEFF_TMO | 0 | **3.0** | Ground effect timeout |
| PSC_JERK_Z | (default 5) | **30** | 6x default vertical jerk |
| EK3_ACC_P_NSE | (default 0.35) | **0.05** | 7x lower — trusts IMU more |

Full parameter dump:

```
EK3_PRIMARY      = 1        (forced Core 1 / IMU1)
EK3_IMU_MASK     = 3        (both IMUs active)
EK3_RNG_USE_HGT  = 10       (should be -1)
EK3_SRC1_POSZ    = 1        (baro)
EK3_SRC1_VELZ    = 0        (NONE)
EK3_SRC1_VELXY   = 5        (optical flow)
EK3_SRC1_POSXY   = 0        (none)
EK3_SRC1_YAW     = 0        (none — GSF yaw)
EK3_ALT_M_NSE    = 1.5
EK3_HGT_I_GATE   = 150
EK3_GND_EFF_DZ   = 7
EK3_ACC_P_NSE    = 0.05
ACC_ZBIAS_LEARN  = 3
INS_ACC_VRFB_Z   = 0.019
BARO1_THST_SCALE = -147
PSC_JERK_Z       = 30
PSC_POSZ_P       = 1.25
PSC_VELZ_P       = 6.5
MOT_THST_HOVER   = 0.3814
RNGFND1_TYPE     = 10       (MAVLink)
RNGFND1_MAX_CM   = 700
GPS1_TYPE        = 0
COMPASS_ENABLE   = 0
ARMING_CHECK     = 64       (RC only)
```

## Flight Timeline

| Time (boot) | Offset (ARM) | Event |
|-------------|-------------|-------|
| 4.0s | — | EKF3 IMU0/IMU1 initialised |
| 5.3s | — | Tilt alignment complete (both cores) |
| **4–105s** | — | **User jogs with drone for ~100 seconds** |
| 64.7s | — | **EKF3 IMU1 forced reset** (reinitialised) |
| 90.5s | — | EKF3 IMU1 fusing optical flow |
| 91.2s | — | EKF3 IMU0 fusing optical flow |
| 107.8s | -0.1s | Both cores: started relative aiding |
| **107.9s** | **+0.0s** | **ARMED** (Stabilize) |
| ~109.5s | +1.6s | Takeoff begins (Stabilize) |
| **109.9s** | **+2.0s** | **Core 1 height reset — terrain lockout begins** |
| ~112s | +4s | Vehicle airborne (~1.5m), Core 1 flow NI=255 saturated |
| ~115s | +7s | Core 1 gndOffsetValid expires — flow fusion blocked |
| ~118s | +10s | Mode: AltHold, DAlt locks at 4.63m (EKF alt, not real) |
| 118–147s | +10–39s | Core 1 dead reckoning: PE diverges from 0 to 356m |
| **147.8s** | **+39.9s** | **EKF3 IMU1 stopped aiding / started relative aiding** |
| 147.8s | +39.9s | PD jump → DAlt spike to 7.5m, ThO=0.77 |
| ~149s | +41s | **Vehicle hits ceiling** (~4m), climbing at 2.6 m/s |
| ~150s | +42s | **Pilot switches to Stabilize** (saves vehicle) |
| 164.3s | +56.4s | DISARMED |

Total flight: ~56s. AltHold: ~30s. Core 1 dead reckoning: **38s**.

## Root Cause: Terrain Estimator Lockout on Core 1

### Overview

The failure is a cascading lockout of the per-core terrain estimator in the EKF3 optical
flow fusion path. Each EKF core maintains an **independent** terrain state estimate. When
Core 1's terrain state diverged at takeoff, flow innovations exceeded the gate, which
prevented terrain state updates, which prevented flow fusion — a self-reinforcing loop.

The `EK3_RNG_USE_HGT=10` setting is the critical enabler: once HAGL exceeded 0.7m, the
rangefinder stopped being the primary height source, and `gndOffsetValid` became dependent
on terrain estimator updates that were already failing.

### Phase 1: Baro Corruption from Jogging (0–105s)

The user jogged with the drone for ~100 seconds. Baro was corrupted by motion-induced
pressure changes. At 64.7s, **Core 1 was forced-reset** (reinitialised mid-jog), giving
it different accelerometer biases and state from Core 0.

During the final put-down (101–104s), Core 1 responded much more aggressively to the
baro transients than Core 0:

| Time | C0 PD (m) | C0 VD (m/s) | C1 PD (m) | C1 VD (m/s) | PD diff (m) |
|------|-----------|-------------|-----------|-------------|-------------|
| 101s | 3.14 | -0.18 | 4.77 | -0.22 | -1.64 |
| 102s | 3.46 | +0.02 | 3.54 | **-1.20** | -0.08 |
| 103s | 3.36 | -0.90 | 1.59 | **-2.57** | +1.76 |
| 104s | 3.36 | -0.15 | 1.17 | **-1.55** | **+2.18** |
| ARM | 3.40 | -0.01 | 1.86 | **-0.33** | **+1.54** |

Core 1 developed VD = -2.57 m/s (thinks it's climbing at 2.6 m/s while sitting on the
ground). By ARM time, the two cores disagreed on altitude by **1.54m**.

### Phase 2: Takeoff Height Reset Breaks Terrain State (109.9s)

At takeoff, the EKF resets PD (position down) but **does not reset terrainState**. For
Core 1, this created an instantaneous HAGL error:

```
Before reset (109.7s): PD=+1.85  terrainState=2.22  HAGL=0.41m  RI=0.13  NI=6 (OK)
After reset  (109.9s): PD=-1.74  terrainState=2.22  HAGL=4.00m  RI=3.89  NI=255 (SATURATED)
```

HAGL jumped from 0.41m to **4.00m** in one timestep because terrainState stayed at 2.22
while PD moved 3.6m. The flow innovations immediately saturated (NI=255) because the
wrong HAGL produces wrong LOS rate predictions.

HAGL continued to grow as PD dropped further during climb:

| Time | C1 PD (m) | C1 HAGL (m) | C1 rng (m) | C1 RI (m) | C1 NI | C1 FIY (cdeg/s) |
|------|-----------|-------------|------------|-----------|-------|-----------------|
| 109.7s | +1.85 | 0.41 | 0.12 | 0.13 | 6 | -135 |
| 109.9s | **-1.74** | **4.00** | 0.11 | **3.89** | **255** | -281 |
| 111.3s | -2.51 | 4.73 | 0.41 | 4.32 | 255 | +1,509 |
| 114.5s | -5.02 | **7.19** | 2.08 | **5.15** | 255 | +46 |
| 114.9s | -4.88 | **1.96** | 1.96 | 0.00 | 168 | -701 |

At 114.9s the terrain state finally snapped to match the rangefinder (offset jumped from
+2.21 to -2.89), but by then Core 1's velocity had already diverged.

### Phase 3: The RNG_USE_HGT Feedback Loop (110–115s)

This is the same feedback loop documented in
[ekf_rng_use_hgt_feedback.md](../topics/ekf_rng_use_hgt_feedback.md):

1. HAGL jumps to 4.0m → exceeds `EK3_RNG_USE_HGT=10` threshold (10% × 700cm = **0.7m**)
2. Rangefinder stops being primary height source
3. `gndOffsetValid` now depends on terrain estimator updates being < 5 seconds old
4. Terrain estimator needs valid auxiliary flow innovations to update `gndHgtValidTime_ms`
5. Auxiliary flow innovations fail gate (wrong HAGL → wrong LOS → huge innovation)
6. `gndHgtValidTime_ms` stales → after 5 seconds, `gndOffsetValid = false`
7. **Flow fusion completely blocked** in `SelectFlowFusion()` (line 42):
   ```cpp
   bool gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000)
       || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER);
   ```
8. With `EK3_RNG_USE_HGT=-1`, the second clause would have been true, preventing lockout

### Phase 4: Dead Reckoning Divergence (115–147.8s)

With no flow corrections, Core 1 flew pure IMU dead reckoning within AID_RELATIVE mode:

| Time | C1 VN (m/s) | C1 VE (m/s) | C1 PE (m) | C1 FIY (cdeg/s) |
|------|-------------|-------------|-----------|-----------------|
| 115s | 1.01 | 2.94 | 8 | -1,043 |
| 120s | 0.48 | 3.83 | 27 | -3,731 |
| 125s | 1.99 | 8.27 | 57 | -3,837 |
| 130s | 1.67 | 9.82 | 103 | -4,181 |
| 135s | 2.82 | 13.96 | 156 | -6,958 |
| 140s | 2.66 | 16.43 | 227 | -8,518 |
| 145s | 3.63 | 18.49 | 299 | -7,068 |
| 147.8s | 2.77 | **20.66** | **331** | -6,671 |

Core 1 thought it was travelling at **20.66 m/s east** and was **331m** from the origin —
while hovering indoors. Flow innovations (FIY) grew to -9,372 cdeg/s (93 deg/s error).
The innovations were being computed but massively failing the gate, confirming the
positive feedback: wrong velocity → wrong LOS prediction → huge innovation → gated →
velocity drifts further.

**Core 0 stayed within ±5m of origin** with stable flow fusion the entire flight.

### Phase 5: Aiding Reset and Ceiling Hit (147.8s)

At 147.8s the EKF detected Core 1 was completely lost:

```
147.8s  EKF3 IMU1 stopped aiding
147.8s  EKF3 IMU1 started relative aiding
147.8s  EKF3 IMU1 fusing optical flow
```

The AID_RELATIVE → AID_NONE → AID_RELATIVE transition reset Core 1's position and
velocity states. Since EK3_PRIMARY=1, the altitude controller used Core 1's states:

- PD jumped ~1.5m (from -4.69 to -6.18)
- DAlt jumped from 4.6m to **7.5m**
- ThO spiked to **0.77** (near full throttle)
- **PSC_JERK_Z=30** (6x default) allowed the altitude target to spike unconstrained
- Vehicle climbed from ~2m to ceiling (~4m) in ~2 seconds at 2.6 m/s
- Pilot was NOT commanding climb (Ch3=1517, near mid-stick)
- Pilot saved vehicle by switching to Stabilize at ~150s

## Why Core 0 Survived But Core 1 Didn't

Both cores experienced a height reset at takeoff. The critical difference was their
starting PD from the 105-second jog:

| Core | PD at ARM | terrainState | HAGL on ground |
|------|-----------|-------------|----------------|
| C0 | 3.40 | unknown (XKF5 not logged for C0) | — |
| C1 | 1.86 | 2.22 | 0.36m |

Core 1's forced reset at 64.7s (mid-jog) left it with different state evolution. By ARM
time, Core 1's PD was 1.54m higher than Core 0's. This larger offset caused a more
severe HAGL jump at the takeoff height reset, pushing Core 1's terrain estimator past
the point of recovery while Core 0's apparently recovered.

Core 0 was not the primary, so the altitude controller never used its (correct) states.

## Vibrations

| IMU | VibeZ Mean | Notes |
|-----|-----------|-------|
| IMU0 | ~10+ m/s² | Same as logm2_5/6 (2x recommended limit) |
| IMU1 | ~1.7 m/s² | Good — this is why EK3_PRIMARY=1 was set |

The choice to force IMU1 as primary was correct in isolation — IMU1 has much better
vibration. The failure was in the EKF state initialization and terrain estimator, not
IMU quality.

## Contributing Factors

| Factor | Value | Impact |
|--------|-------|--------|
| **EK3_RNG_USE_HGT=10** | threshold 0.7m | **PRIMARY** — enabled terrain lockout feedback loop |
| **105s pre-ARM jog** | — | Corrupted baro, created 1.54m PD difference between cores |
| **EK3_PRIMARY=1** | forced Core 1 | Prevented fallback to healthy Core 0 |
| **IMU1 forced reset** | at 64.7s | Different state evolution, more sensitive to baro transients |
| **PSC_JERK_Z=30** | 6x default | Allowed dangerous altitude target spike at aiding reset |
| **EK3_SRC1_VELZ=0** | no VelZ source | Nothing to constrain vertical velocity drift |
| **EK3_ACC_P_NSE=0.05** | 7x below default | EKF trusted IMU more, corrected slower |

## Recommended Changes

### Critical

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| **EK3_RNG_USE_HGT** | 10 | **-1** | Always use rangefinder — prevents terrain lockout |
| **EK3_PRIMARY** | 1 | **0** | Let EKF choose healthiest core |
| **PSC_JERK_Z** | 30 | **5** (default) | Prevent dangerous altitude target spikes |

### Important

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| EK3_ACC_P_NSE | 0.05 | **0.35** (default) | Allow EKF to correct from observations faster |

### Investigation

- **Terrain state not reset on height datum change** — the EKF resets PD at takeoff but
  not terrainState, creating an instantaneous HAGL error. This is an EKF3 code issue that
  should be fixed to reset terrainState alongside PD.
- **gndOffsetValid expiry cascade** — when both auxiliary flow and rangefinder are rejected,
  `gndHgtValidTime_ms` stales and locks out ALL terrain-dependent fusion. Consider making
  the terrain estimator more robust to transient errors.
- **Pre-ARM jog sensitivity** — should the EKF reject baro during large-motion periods?
  Or should ARM require a recent stable-on-ground baro calibration?

## Key Code References

The terrain lockout occurs in `AP_NavEKF3_OptFlowFusion.cpp`:

- **`SelectFlowFusion()` line 42**: `gndOffsetValid` check gates all flow fusion
- **`EstimateTerrainOffset()` lines 218/253**: auxiliary flow innovation gate controls
  `gndHgtValidTime_ms` updates
- **`FuseOptFlow()` line 692**: per-core innovation gate (`flowTestRatio < 1.0`)

Each EKF core maintains independent: `terrainState`, `gndHgtValidTime_ms`,
`gndOffsetValid`, `flowFusionActive`, `flowTestRatio[]`, `flowInnov[]`.

## See Also
- [logm2_4](logm2_4.md) — same vehicle, flight 1 (2.5cm alt error, excellent)
- [logm2_5](logm2_5.md) — same vehicle, flight 2 (IMU0 clipping discovered)
- [logm2_6](logm2_6.md) — same vehicle, flight 3 (altitude failure from AccZ bias)
- [ekf_rng_use_hgt_feedback](../topics/ekf_rng_use_hgt_feedback.md) — the RNG_USE_HGT
  feedback loop (first identified in logjk6→logjk7)
- [TD-MicoAir-2](../vehicles/TD-MicoAir-2.md) — vehicle profile
