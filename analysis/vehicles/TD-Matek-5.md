# TD-Matek-5 — Vehicle Notes

## Vehicle Description

- **Frame**: Quad BetaFlight-X (FRAME_CLASS=1, FRAME_TYPE=1)
- **FC**: MatekH743-bdshot
- **Firmware**: V4.6.3v2-SFD (SmallFastDrone-4.6-AltHold branch)
- **IMU**: Dual IMU (EK3_IMU_MASK=3)
- **Baro**: On-board DPS310 (exposed to prop wash and thermal soak, runs 50-54 C)
- **Rangefinder**: TOF sensor (RNGFND1_TYPE=24, MAX_CM=3000), Orient=25 (downward)
- **GPS**: u-blox SAM-M10Q, active for outdoor flights (20+ sats typical)
- **Compass**: Internal only (no external) — significant motor interference
- **Battery**: LiPo (switching to LiIon — different cells have different magnetic signatures)
- **MOT_THST_HOVER**: 0.126–0.151 (very high thrust-to-weight)
- **Notch filter**: ESC telemetry tracking, 150 Hz, double notch (INS_HNTCH_MODE=3, OPTS=6)

## Flight Log Summary

| # | Log | Date | Location | Alt (m) | Alt Err Std | Key Finding |
|---|-----|------|----------|---------|-------------|-------------|
| 1 | [log3](../logs/log3.md) | Feb 16 | Outdoor | 2.0 | **14.2 cm** | 9.4°C baro thermal rise; post-land EKF +0.94m; yaw reset 98s |
| 2 | [log4](../logs/log4.md) | Feb 16 | Outdoor | 3.0 | 29.1 cm | IVD=-0.54 stuck; 0.69m core divergence; post-land EKF -2.3m |
| 3 | [log5](../logs/log5.md) | Feb 16 | Outdoor | ~1.4 | 37.5 cm* | **EKF Failsafe** — GPS+compass error cascade; massive compass interference |
| 4 | [log6](../logs/log6.md) | Feb 16 | Outdoor | ~1.9 | **23.5 cm** | IVD no longer stuck; compass offsets MY~300 MZ~-280; 61° post-disarm yaw |
| 5 | [log7](../logs/log7.md) | Feb 16 | Outdoor | ~2.4 | **12.5 cm** | **Best outdoor** — C0 IVD frozen; MAG_FUSION stayed frozen; VRFB learning |
| 6 | [log11](../logs/log11.md) | Feb 16 | Outdoor | ~1.9 | **15.5 cm** | GNDEFF_TMO+ARMING applied; compass stuck IMZ=-263; no failsafe; zero clips |
| 7 | [log12](../logs/log12.md) | Feb 16 | Ground | — | — | Ground session (no flight); baro drift -1.17m over 20°C rise; vehicle rotated |
| 8 | [log14](../logs/log14.md) | Feb 16 | Outdoor | ~1.9 | **7.7 cm** | 50% improvement; no lane switch; GPS absent 88s; compass still stuck |
| 9 | [log15](../logs/log15.md) | Feb 16 | Outdoor | ~1.9 | **7.1 cm** | **BEST** — compass innovations 275→1.5 mG; MAG_CAL=7 YawOnly; 173s flight |

*log5 cut short by EKF failsafe after 30s of ALT_HOLD

## Parameter Change History

### Flights 1-2: log3, log4 — Baseline (Feb 16)

**Starting config:**
```
EK3_RNG_USE_HGT     = -1
BARO1_THST_SCALE    = -100
BARO1_THST_FILT     = 1.0
EK3_MAG_CAL         = 3        (AFTER_CLIMB)
COMPASS_MOTCT        = 0        (no motor compensation)
INS_ACC_VRFB_Z      = -0.343   (corrupted — wrong sign)
ACC_ZBIAS_LEARN     = 0        (disabled)
TKOFF_GNDEFF_TMO    = 0        (no timeout)
TKOFF_GNDEFF_ALT    = 1.0
MOT_THST_HOVER      = 0.126
PSC_POSZ_P          = 1.0
PSC_VELZ_P          = 5.0
EK3_ALT_M_NSE       = 2.0
EK3_HGT_I_GATE      = 300
INS_HNTCH_ENABLE    = 1
```

**Results:** log3=14.2cm std, log4=29.1cm std.

**Critical issues found:**
1. INS_ACC_VRFB_Z=-0.343 corrupted (wrong sign)
2. ACC_ZBIAS_LEARN=0 → stuck IVD=-0.54 in log4
3. TKOFF_GNDEFF_TMO=0 → no takeoff baro protection
4. MAG_CAL=3 → 98s yaw convergence
5. COMPASS_MOTCT=0 → no motor interference compensation

**Changes planned for next flight:**
```
ACC_ZBIAS_LEARN     = 2        → actual: 3
INS_ACC_VRFB_Z      = 0        → actual: -0.011 (nearly zero)
TKOFF_GNDEFF_TMO    = 3.0      → actual: 0 (NOT APPLIED)
EK3_MAG_CAL         = 7        → actual: 7 ✓
COMPASS_MOTCT        = 2        → actual: 2 ✓
ARMING_CHECK        = 1        → actual: 64 (NOT APPLIED)
```

---

### Flights 3-5: log5, log6, log7 — Post-Update (Feb 16)

**Actual config changes applied:**
```
EK3_MAG_CAL         = 7        ✓ (was 3)
COMPASS_MOTCT        = 2        ✓ (was 0)
ACC_ZBIAS_LEARN     = 3        (was 0 — set to 3 instead of planned 2)
INS_ACC_VRFB_Z      ≈ -0.01    (was -0.343 — nearly zeroed but not exact)
```

**NOT applied (still at old values):**
```
TKOFF_GNDEFF_TMO    = 0        (should be 3.0)
ARMING_CHECK        = 64       (should be 1)
```

**Results across three consecutive flights:**

| Flight | Log | Alt Err Std | Outcome |
|--------|-----|-------------|---------|
| 3 | log5 | 37.5 cm* | EKF Failsafe at +56s (compass interference) |
| 4 | log6 | 23.5 cm | Stable 83s hover, IVD improved |
| 5 | log7 | **12.5 cm** | Best outdoor performance, 120s stable hover |

**Key observations across log5/6/7:**

1. **IVD=-0.54 fixed**: The persistent stuck velocity innovation from log4 is eliminated.
   Clearing VRFB and enabling ZBIAS_LEARN confirmed as the fix.

2. **Compass interference discovered**: Internal compass shows 100-300 mGauss motor
   interference. Caused EKF Failsafe in log5 when GPS was first incorporated. In log6,
   compass offsets MY~300, MZ~-280 were learned. In log7, MAG_FUSION froze and offsets
   were static at 37/-48/-5.

3. **INS_ACC_VRFB_Z learning trajectory**: -0.011 (log5 boot) → -0.390 (log6 end) →
   -0.296 (log7 end). The value moved away from zero then started returning. This
   negative bias is real and large.

4. **MAG_CAL=7 inconsistent**: Worked correctly in log5/6 (learning→frozen→learning).
   In log7, froze at takeoff and never resumed — possibly triggered by two consecutive
   ground mag anomaly events.

5. **Core 0 velocity freeze**: In log7, Core 0 IVD/IVN/IVE all froze at t=33s
   (-0.99/-0.18/0.37) while position innovations continued. New issue not seen before.

6. **Performance improved despite EKF anomalies**: log7 achieved 12.5cm std (best outdoor)
   even with frozen Core 0 velocity innovations and frozen MAG_FUSION.

---

### Flight 6: log11 — GNDEFF_TMO + ARMING_CHECK Applied (Feb 16)

**Changes applied:**
```
TKOFF_GNDEFF_TMO    = 3.0      ✓ (was 0)
ARMING_CHECK        = 1        ✓ (was 64)
```

**NOT changed (now considered correct):**
```
INS_ACC_VRFB_Z      = -0.3948  (learned bias — should NOT be zeroed)
INS_ACC2_VRFB_Z     = -0.2221  (learned bias — should NOT be zeroed)
```

**Results:** 15.5cm alt error std, 100s stable hover, zero clipping.

**Key observations:**
1. Both pending params now applied — all arming checks enabled, ground effect timeout active
2. Compass interference persists: IMX/IMY/IMZ innovations stuck at +103/-43/-263 mGauss
3. Motor compensation applies 400-650 mGauss corrections but residual still 100-263 mGauss
4. EKF lane switch C0→C1 at +10s (compass-related), back to C0 at landing
5. Ground mag anomaly triggered at +17s after ARM
6. GPS velocity fusion delayed to +51.5s (very late)
7. No velocity fusion freeze (log7 Core 0 issue not present)
8. AccZ bias learning working: both cores converging toward zero
9. Post-landing drift much milder than before — no EKF failsafe
10. Optical flow configured but zero samples — sensor not providing data

---

### Flight 7: log12 — Ground Session (Feb 16)

Ground-only session, 206s. Vehicle never armed. Same params as log11.
Baro thermal drift: -1.17m over 20°C temperature rise (0.058 m/°C).
Vehicle physically rotated at T+200s (compass calibration or inspection).

---

## Performance Progression

```
log3   [====]        14.2 cm  — baseline, 2m, suboptimal params
log4   [=========]   29.1 cm  — baseline, 3m, stuck IVD exposed
                               ── param update 1 ──
log5   [============] 37.5 cm  — EKF Failsafe (30s only), compass crisis
log6   [=======]     23.5 cm  — stable, IVD improved, compass learning
log7   [====]        12.5 cm  — BEST OUTDOOR, 120s stable
                               ── param update 2 ──
log11  [=====]       15.5 cm  — GNDEFF+ARMING applied, zero clips, compass still stuck
log12  [ground]        —      — ground session, no flight
log14  [==]           7.7 cm  — 50% improvement, no lane switch, GPS absent
log15  [==]           7.1 cm  — BEST: compass innovations near zero, 173s flight
```

## Current Configuration (as of log15)

```
# Height source
EK3_RNG_USE_HGT     = -1
EK3_SRC1_POSZ        = 1        (baro)
EK3_SRC1_VELZ        = 3        (GPS)
EK3_ALT_M_NSE       = 2.0
EK3_HGT_I_GATE      = 300

# Baro compensation
BARO1_THST_SCALE    = -100
BARO1_THST_FILT     = 1.0

# Position controller
PSC_POSZ_P          = 1.0
PSC_VELZ_P          = 5.0

# Accel bias
INS_ACC_VRFB_Z      = -0.377   # Learned value (converging toward zero)
INS_ACC2_VRFB_Z     = -0.201   # Learned value (converging toward zero)
ACC_ZBIAS_LEARN     = 3        # Always learning

# Ground effect
TKOFF_GNDEFF_ALT    = 1.0
TKOFF_GNDEFF_TMO    = 3.0      # Applied ✓

# Compass
EK3_MAG_CAL         = 7        # YawOnly works when it transitions correctly
COMPASS_MOTCT        = 2        # Falls back to throttle (ESCs report 0A)
COMPASS_MOT_X       = -61.4
COMPASS_MOT_Y       = 1.0
COMPASS_MOT_Z       = 89.5

# Arming
ARMING_CHECK        = 1        # Applied ✓

# Motor
MOT_THST_HOVER      = 0.1509   (learned, rising — now highest value)
INS_HNTCH_ENABLE    = 1
INS_HNTCH_FREQ      = 150
INS_HNTCH_MODE      = 3        (ESC telemetry)
INS_HNTCH_OPTS      = 6        (multi-source + double notch)
```

## Pending Changes (for next flight)

**Compass situation improving but not fully reliable.** MAG_CAL=7 YawOnly mode produced
excellent results in log15 (1.5 mG innovations) but failed to activate in log11 and log14
(275 mG stuck innovations). The transition depends on the EKF correctly detecting the ground
mag anomaly and completing in-flight yaw alignment.

| Priority | Action | Reason |
|----------|--------|--------|
| **Critical** | **Fix COMPASS_MOTCT calibration** | MOTCT=2 (current) but ESCs report 0A — coefficients are wrong-scale. Switch to MOTCT=1 (throttle) and recalibrate, or fix ESC current reporting |
| **Critical** | **Test with LiIon battery** | Different cells have different magnetic signatures — the right battery may dramatically reduce compass interference at the source |
| Investigate | GPS antenna/reception | log14: 88s no fix, log15: 168s; was 14-20 sats in log3-7 |
| Investigate | Optical flow (FLOW_TYPE=6) | Zero samples in all recent flights |
| Consider | External compass on mast | Would eliminate root cause of interference |

## Known Issues

1. **Compass interference — partially resolved by MAG_CAL=7 YawOnly mode.** Raw motor
   interference is still ~293 mGauss (MOX=-470, MOZ=+685 mGauss corrections applied).
   In log15, MAG_CAL=7 correctly downgraded to YawOnly fusion, reducing innovations from
   275 mGauss to 1.5 mGauss. However, this behavior is **not yet reliable** — log11 and
   log14 stayed in 3D fusion mode with stuck innovations. The transition to YawOnly depends
   on the EKF detecting the ground mag anomaly and completing in-flight yaw alignment.
   An external compass would eliminate the root cause.

2. **GPS reception unreliable** — log14 had no 3D fix for 88s, log15 only got fix 9s before
   landing. The u-blox SAM-M10Q is producing 0-5 sats in some flights vs 14-20 in others
   (log3-7). May be antenna obstruction, interference, or sky view dependent. Velocity
   innovations are frozen for most of the flight when GPS is absent.

3. **Post-landing EKF divergence** — baro transients during landing cause BAlt spikes of
   -6.7 to -9.3m. EKF alt diverges to -3.4m at disarm, then recovers to ~-1.0m within 2s
   as baro stabilizes. Worse with aggressive landings (log14: +1.59m drift). The EKF
   "recovery" post-disarm is actually correcting landing-induced baro errors.

4. **Baro thermal drift** — baro runs 39-62°C. Ground test (log12) measured ~0.058 m/°C.
   Cold starts see 11-20°C rise. Not dominant after warm-up.

5. **Optical flow not working** — FLOW_TYPE=6 configured but zero flow samples in all
   flights (log11, log14, log15). Sensor not providing data.

6. **COMPASS_MOTCT=2 current calibration is wrong** — COMPASS_MOTCT=2 uses current-based
   compensation, but ESCs report 0.00A. The motor compensation coefficients (MOT_X=-61.4,
   MOT_Z=89.5) were calibrated against current that doesn't exist, so the compensation
   is either inactive or using throttle fallback with wrong-scale coefficients. This
   explains the enormous residual interference (293 mGauss). **Fix: either switch to
   COMPASS_MOTCT=1 (throttle) and recalibrate, or fix ESC current reporting.**

7. **EKF lane switch on takeoff** — occurs in most flights (C0→C1 at +3-10s after ARM),
   triggered by compass interference during motor startup. Not causing failure but
   indicates compass instability affects core selection.

8. **CG offset** — C3 consistently +2-3% above mean, C4 -2-3% below. Slight forward CG
   bias. Not flight-critical.

## Vehicle-Specific Observations

| Metric | log3 | log4 | log5 | log6 | log7 | log11 | log14 | log15 |
|--------|------|------|------|------|------|-------|-------|-------|
| Hover throttle | 0.131 | 0.127 | 0.136 | 0.138 | 0.143 | 0.130 | 0.141 | 0.152 |
| Baro temp range | +9.4°C | +8.5°C | 11.8°C* | **1.6°C** | 2.9°C | 5.5°C | 11.0°C | 9.0°C |
| Max BAlt spike | -6.15 m | -6.80 m | — | — | -6.44 m | -6.72 m | -9.26 m | -6.66 m |
| Post-land drift | +0.94 m | -2.32 m | -0.80 m | -0.87 m | 0.58 m+ | -0.67 m | +1.59 m | -3.4→-1.0 |
| IVD (C0) | — | -0.54 stuck | +0.5 | -0.39 osc | -0.99 frozen | -0.15 OK | +0.07 frozen | frozen |
| Mag innov |I| | — | — | — | — | — | **286 stuck** | **275 stuck** | **1.5 OK** |
| IMU clips | — | — | — | — | — | 0/0 | 0/0 | 6/3 |
| GPS fix delay | — | — | — | — | — | 51.5s | **88.3s** | **168s** |
| AltHold (s) | 108 | 108 | 30 | 83 | 120 | 100 | 88 | **173** |
| Outcome | Good | Good | **Failsafe** | Good | Best | Good | Good | **BEST** |

*Full log including boot heating

## See Also
- [MAG_CAL=7 verification](../topics/mag_cal_7_verification.md) — proven on SFDv1
- [Baro thermal drift](../topics/baro_thermal_drift.md) — cross-vehicle analysis
