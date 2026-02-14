# Indoor Loiter — SmallFastDronev1 Vehicle

Cross-cutting analysis from [log197](../logs/log197.md) (stable hover) and
[log198](../logs/log198.md) (crash) on the SmallFastDronev1 BF_X quad indoors with optical
flow + rangefinder.

## Vehicle Characteristics

- Frame: BF_X quad
- Firmware: V4.6.3v2-SFD (e0cb6544)
- Hover throttle: 0.082 (very low — high power-to-weight)
- Position source: Optical flow (EK3_SRC1_VELXY=5)
- Height sensor: Rangefinder (0.14–0.88m hover altitude)
- Baro: DPS310 with INS_TCAL enabled
- Vibration: VibeZ ≈ 3.2 m/s² RMS (moderate)

## Crash Root Cause: Terrain Offset Feedback Loop

The log198 crash followed the same failure mode as [logjk6](../logs/logjk6.md)
on the SFD vehicle, but caused by baro error rather than deliberate parameter change.

**Chain of failure:**
1. Baro reads 4m too low (zero THST_SCALE + thermal drift + ground effect)
2. With `EK3_RNG_USE_HGT=3.0`, EKF terrain offset jumps to +4.1m at takeoff
3. HAGL = EKF_height - terrain_offset becomes 2–4x actual height
4. Flow velocity = angular_rate × HAGL → velocities amplified 2–4x
5. Loiter controller fights phantom velocity → progressive backward lean → crash

**Fix:** `EK3_RNG_USE_HGT=-1` eliminates the terrain offset feedback loop entirely.
Flow fusion reads rangefinder directly — no lag, no overshoot, no amplification.
This was already proven on both SFD (logjk7) and TD (logtd2) vehicles.

## Three Separate Baro Problems

This vehicle has three distinct baro error sources that must be addressed independently:

### 1. Baro Thermal Drift (dominant)

The DPS310 baro cools 8.5°C during a 54-second hover from prop airflow. This causes
altitude drift that is NOT throttle-proportional and NOT correctable by THST_SCALE.

| Metric | Value |
|--------|-------|
| Temperature drop | 8.5°C over 54s |
| Baro drift rate | ~0.015 m/s |
| Total baro error | ~0.6m |
| Inter-flight persistence | Yes — log198 started 0.81m off from log197 cooling |

### 2. Ground Effect (transient)

Motor start creates massive pressure increase at ground level:
- BAlt drops from +0.17m to **-6.38m** when motors start
- Effect is highly altitude-dependent (negligible above ~0.5m)
- Not correctable by THST_SCALE (nonlinear, altitude-dependent)
- Handled by EKF ground effect dead zone (EK3_GND_EFF_DZ=4.0m default)

**Ground effect timing problem:** With TKOFF_GNDEFF_ALT=0.5m and TKOFF_GNDEFF_TMO=0,
the EKF ground effect protection expires almost immediately because the EKF altitude
(which is wrong from baro error) exceeds 0.5m within milliseconds.

**Fix:** `TKOFF_GNDEFF_TMO=3.0` keeps protection active for at least 3 seconds.

### 3. Hover Propwash (minor)

At hover altitude, propwash slightly decreases baro pressure:
- Estimated THST_SCALE: **-20 Pa** (from thermal-corrected regression)
- Correction at hover throttle 0.082: only 14cm
- Negligible compared to thermal drift

## INS Temperature Calibration — Upside-Down Discovery

### The Problem

Despite INS_TCAL1_ENABLE=1, AccZ showed -0.045 m/s²/°C thermal drift (using baro
temperature as proxy). Investigation revealed the TCAL is **making drift worse**:

| | Drift rate | Drift over flight |
|---|---|---|
| Raw sensor (no TCAL) | -0.014 m/s²/°C | 0.09 m/s² |
| TCAL overcorrection | +0.053 m/s²/°C applied | 0.35 m/s² added |
| Net (observed) | -0.067 m/s²/°C | **0.45 m/s²** |

### Root Cause: Scale Factor Drift Sign Reversal

The board was mounted **upside down** during TCAL oven calibration. Accelerometer
thermal drift has two components:

- **Offset drift:** Same regardless of orientation
- **Scale factor drift:** Proportional to applied acceleration (gravity)

When upside down, AccZ sees +g. When right-side up, it sees -g. The scale factor
component flips sign:

```
Upside-down cal learns:    offset + scale × (+g)  =  +0.053/°C
Right-side-up flight has:  offset + scale × (-g)  =  -0.014/°C
Overcorrection error:      2 × scale × g          =   0.067/°C
```

Solving: **offset drift = +0.020 m/s²/°C, scale factor drift = 0.34%/°C**

### Evidence

The Z-axis polynomial coefficient `INS_TCAL1_ACC1_Z = 55844` is:
- 1861x larger than X-axis (30)
- 41x larger than Y-axis (1362)

X and Y axes are unaffected because they see ~0g in both orientations.

### Cross-Validation

The thermal model predicted log198's ground AccZ from log197's model to within
0.047 m/s² across a 7°C ambient temperature difference — confirming consistency.

### Fix

Either zero out Z-axis coefficients:
```
INS_TCAL1_ACC1_Z = 0
INS_TCAL1_ACC2_Z = 0
INS_TCAL1_ACC3_Z = 0
INS_TCAL2_ACC1_Z = 0
INS_TCAL2_ACC2_Z = 0
INS_TCAL2_ACC3_Z = 0
```

Or redo the TCAL calibration with the board right-side up (preferred).

## Vibration Rectification Factor (VRF)

### Estimation Method

Compared raw IMU AccZ between ground (motors at idle, VibeZ ≈ 0.2 m/s²) and hover
(VibeZ ≈ 3.2 m/s²). Used linear regression against IMU temperature to separate
VRF (constant at constant vibration) from residual thermal drift:

```
AccZ_offset(hover - ground) = VRF + thermal_coeff × temp_change
```

### Results

| Source | VRF | Thermal Coeff | R² |
|--------|-----|---------------|-----|
| Log197 (50s hover) | **+0.089 m/s²** | -0.045 m/s²/°C | 0.82 |
| Log198 (18s, noisy) | +0.048 m/s² | -0.016 m/s²/°C | — |

**Best estimate: VRF = +0.09 m/s²** → `INS_ACC_VRFB_Z = 0.09`

Positive VRF means vibration makes AccZ less negative (less upward specific force).
Without correction, the EKF interprets this as downward acceleration → altitude
estimate drifts down.

### VRF vs TCAL Interaction

With the upside-down TCAL active, the EKF Z-bias absorbed VRF + TCAL overcorrection:
- VRF component: +0.089 m/s²
- TCAL overcorrection: +0.359 m/s² (over 8.5°C cooling)
- Total AccZ shift: +0.448 m/s²
- EKF tracked 69% (AZ bias: -0.510 → -0.201)

In log198, the Z-bias learning converged to INS_ACC_VRFB_Z = -0.066 (wrong sign)
because baro thermal drift contaminated the height reference. With TCAL fixed, the
learning should converge to the correct +0.09 value.

## Recommended Parameters

### Immediate (fixes crash, reduces thermal drift)

```
# Fix terrain offset feedback loop
EK3_RNG_USE_HGT = -1

# Fix TCAL overcorrection (zero Z-axis until recalibrated)
INS_TCAL1_ACC1_Z = 0
INS_TCAL1_ACC2_Z = 0
INS_TCAL1_ACC3_Z = 0
INS_TCAL2_ACC1_Z = 0
INS_TCAL2_ACC2_Z = 0
INS_TCAL2_ACC3_Z = 0

# Position controller gains
PSC_POSZ_P = 1.0

# Ground effect timeout
TKOFF_GNDEFF_TMO = 3.0

# Vibration rectification pre-load
INS_ACC_VRFB_Z = 0.09

# Baro thrust compensation (modest)
BARO1_THST_SCALE = -20
```

### After TCAL Redo (right-side up)

After recalibrating TCAL with the board right-side up:
- Re-enable Z-axis coefficients (will be correct this time)
- Set `INS_ACC_VRFB_Z = 0` and let the Z-bias learning re-converge
- The learning should find +0.09 within a few flights
- Monitor with `ACC_ZBIAS_LEARN = 3`

## Follow-Up Flights: Log201 and Log202

After applying fixes from the log197/198 analysis, two follow-up flights were performed.

### Log201 — Stabilize Sanity Check

See [log201](../logs/log201.md). Key outcomes:

1. **TCAL fix confirmed:** After recalibrating right-side up (ACC1_Z: 55844 → -3597),
   AccZ bias remained stable through a 5.3°C temperature swing. The upside-down
   calibration issue is fully resolved.

2. **Yaw inconsistency discovered:** 24° yaw divergence between EKF cores during flight.
   Root cause: COMPASS_MOTCT=0 + EK3_MAG_CAL=4 (ALWAYS) — motor interference at
   -236 mG/throttle on X caused Core 1 MX body offset to run away from +2 to -58 mG
   while Core 0 MX stayed stable at -5 to +6 mG. This led to developing EK3_MAG_CAL=7.

### Log202 — Loiter with Updated Parameters

See [log202](../logs/log202.md). Key outcomes:

1. **Altitude improved:** BAlt mean 0.39m (vehicle held altitude, no crash).
   The PSC_POSZ_P=1.0 + TKOFF_GNDEFF_TMO=3.0 changes were effective.

2. **Yaw twitchiness:** ATC_ANG_YAW_P=17.8 (4x default of 4.5) chases compass noise
   aggressively, causing visible twitchiness. In-flight yaw alignment at t=57s caused
   18° heading snap. Recommend reducing to 4.5-6.0.

3. **Terrain offset still noisy:** ±1.7m noise confirms need for EK3_RNG_USE_HGT=-1.

### Parameter Progression

| Parameter | Log197/198 | Log201/202 | Log208 (outdoor) | Notes |
|-----------|-----------|-----------|-------------------|-------|
| EK3_MAG_CAL | 4 | 4 | **7** | New GROUND_AND_INFLIGHT mode |
| COMPASS_MOTCT | 0 | 0 | **2** | Current-based motor compensation |
| EK3_RNG_USE_HGT | 3.0 | 3.0 | **-1** | Fixed terrain offset feedback |
| INS_TCAL ACC_Z | Wrong (upside-down) | **Recalibrated** | Recalibrated | Fixed in log201 |
| PSC_POSZ_P | default | **1.0** | 1.0 | Improved altitude hold |
| TKOFF_GNDEFF_TMO | 0 | **3.0** | 3.0 | Prevents premature ground effect timeout |
| ATC_ANG_YAW_P | 17.8 | 17.8 | 17.8 | Still needs reduction to 4.5-6.0 |

## Comparison with Other Vehicles

| | SFD Indoor | TD Indoor | **SmallFastDronev1 Indoor** |
|---|---|---|---|
| THST_SCALE | -550 Pa | -147 Pa | **-20 Pa** |
| VRF (IMU0) | +0.073 | — | **+0.089** |
| Hover throttle | 0.32 | 0.45 | **0.082** |
| Baro thermal drop | 2°C | — | **8.5°C** |
| TCAL status | Not enabled | — | **Enabled but inverted Z** |
| Best alt hold | 10.4cm (logjk7) | 6.2cm (logtd2) | **TBD** |
| Critical fix | RNG_USE_HGT=-1 | RNG_USE_HGT=-1 | **RNG_USE_HGT=-1 + fix TCAL** |

Key difference: SmallFastDronev1 has very low hover throttle (0.082 vs 0.32–0.45), so propwash
is much less significant. The dominant error source is thermal drift, amplified by the
inverted TCAL calibration.
