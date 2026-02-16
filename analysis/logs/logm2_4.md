# logm2_4 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-MicoAir-2 (MicoAir743v2)
- **Firmware**: V4.6.3v2-SFD (883e416a)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log4.bin
- **Sensors**: Optical flow, baro, dual IMU, rangefinder (MAVLink, 7m max)
- **No GPS, no compass** — fully GPS-denied
- **Flight #**: 1

## Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| FRAME_CLASS/TYPE | 1/1 | Quad X |
| EK3_SRC1_POSXY | 0 (None) | No XY position source |
| EK3_SRC1_VELXY | 5 (OpticalFlow) | Flow provides XY velocity |
| EK3_SRC1_POSZ | 1 (Baro) | Baro primary height |
| EK3_SRC1_VELZ | 0 (None) | No Z velocity source |
| EK3_SRC1_YAW | 0 (None) | No yaw source (GSF) |
| EK3_RNG_USE_HGT | -1 | Rangefinder height disabled |
| EK3_IMU_MASK | 1 | **Single core (IMU0 only)** |
| EK3_ALT_M_NSE | 1.5 | Elevated baro noise |
| EK3_HGT_I_GATE | 150 | Wide height gate |
| EK3_GND_EFF_DZ | 7 | Ground effect dead zone 7m |
| BARO1_THST_SCALE | -147 | Strong thrust-baro compensation |
| PSC_POSZ_P | 1.25 | |
| PSC_VELZ_P | 6.5 | Aggressive |
| INS_ACC_VRFB_Z | 0.041 | IMU0 VRF learned |
| ACC_ZBIAS_LEARN | 0 | **Disabled** |
| TKOFF_GNDEFF_TMO | 0 | No timeout |
| COMPASS_ENABLE | 0 | **Compass disabled** |
| GPS1_TYPE | 0 | **No GPS** |
| MOT_THST_HOVER | 0.3814 | ~38% hover |
| MOT_HOVER_LEARN | 0 | Learning disabled |
| INS_HNTCH_FREQ | 475 Hz | Very high (small props, high KV) |
| INS_HNTCH_BW | 235 Hz | Wide bandwidth |
| INS_HNTCH_MODE | 1 | Throttle-based tracking |
| INS_HNTCH_OPTS | 6 | Multi-source + loop rate update |
| ARMING_CHECK | 64 | Minimal (RC only) |
| AHRS_ORIENTATION | 1 | Board YAW45 |
| SURFTRAK_MODE | 0 | Surface tracking in AltHold |
| RNGFND1_TYPE | 10 | MAVLink rangefinder |
| RNGFND1_MAX_CM | 700 | 7m max |

## Flight Timeline

| Offset (from arm) | Event |
|--------------------|-------|
| -4.1s | Boot: V4.6.3v2-SFD, MicoAir743v2, QUAD/X |
| -4.1s | Lua error: Turtle_Mode.lua:14 (repeats every 10s) |
| -2.3s | EKF3 IMU0 initialized, EKF3 active |
| -2.2s | Fusing optical flow |
| -2.1s | Tilt alignment complete |
| **+0.0s** | **ARMED** (STABILIZE) |
| +0.7s | AUTO_ARMED |
| +1.0s | NOT_LANDED (takeoff) |
| +5.3s | EKF3 started relative aiding |
| **+7.0s** | **Mode: ALT_HOLD** |
| +16.9s | EKF origin set, "GPS Denied Environment" |
| +16.9s | SET_HOME |
| **+114.8s** | **Mode: STABILIZE** (descent) |
| +117.9s | LAND_COMPLETE |
| **+118.1s** | **DISARMED**, EKF stopped aiding |
| +119.1s | EKF variance: position lost, FAILSAFE_EKFINAV |

Total armed: 118s. AltHold: ~108s.

## Altitude Hold Performance

### CTUN Statistics (AltHold, +7s to +115s)

1078 samples.

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 2.494 m | 0.282 | 2.126 | 2.818 |
| DAlt (desired) | 2.512 m | 0.275 | 2.248 | 2.803 |
| BAlt (baro) | 2.441 m | 0.312 | 1.790 | 2.970 |
| ThO | 0.383 | 0.005 | 0.330 | 0.439 |
| SAlt (rangefinder) | 1.295 m | 0.286 | 0.800 | 1.800 |
| Alt Error mean | **-0.017 m** | — | -0.122 | +0.041 |
| **Alt Error std** | **0.025 m** | — | — | — |

**2.5 cm alt error std — excellent performance.** ThH constant at 0.3814 (learning disabled).

The large Alt std (0.282m) reflects a terrain step event, not poor control. Phase-separated:

| Phase | Alt mean | SAlt mean | ThO mean |
|-------|----------|-----------|----------|
| Before step (20-68s) | 2.227 m | 1.068 m | 0.383 |
| After step (72-120s) | 2.791 m | 1.488 m | 0.384 |
| **Step** | **+0.564 m** | **+0.420 m** | — |

Vehicle drifted over terrain ~0.55m lower. Surface tracking maintained stable AGL.

## Rangefinder

2156 samples during hover, **100% Status=4 (good).**

| Metric | Value |
|--------|-------|
| Dist mean | 1.304 m |
| Dist std | 0.285 m |
| Range | 0.81 - 1.81 m |

RFRF fusion data shows FTypes=8 (range height fusion) dominant with 42,579 messages.
XKF5 shows HAGL=1.354m mean, Herr=0.063m, RI (range innovation) mean=0.031m — well-behaved.

**Despite EK3_RNG_USE_HGT=-1, the rangefinder appears to be actively contributing to
height estimation through terrain offset tracking.** The surface tracking behavior and
tight altitude hold strongly suggest rangefinder-aided height.

## XKF3 Innovations

**All velocity innovations stuck at constant values during hover:**

| Innovation | Value |
|-----------|-------|
| IVN | +0.47 (constant) |
| IVE | -0.40 (constant) |
| IVD | **-0.43 (constant)** |

No variation at 2-decimal log resolution. Position innovations (IPD etc.) do vary normally.

## EKF (Single Core)

EK3_IMU_MASK=1 — only Core 0 (IMU0). No dual-core comparison possible.

Core 0 PD: stable within each phase, clear terrain step at t=69s. Post-disarm: dramatic
2.5m PD jump followed by EKF position lost error.

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 24.8 C | 57.1 C | **32.3 C** |
| IMU0 | 21.1 C | 53.8 C | 32.6 C |
| IMU1 | 23.1 C | 57.7 C | 34.6 C |

**Massive 32°C temperature rise from cold start.** This drives significant baro offset
drift (visible in XKF5 offset changing from -0.61 to -1.33). Rangefinder compensates.

## Vibrations

IMU0 during hover:

| Axis | Mean | Max |
|------|------|-----|
| VibeX | 2.31 | 5.98 |
| VibeY | 2.47 | 4.69 |
| VibeZ | **7.23** | **18.76** |
| Clip | 0 | 0 |

Z-axis vibration mean 7.23 is elevated (recommended <5). No clipping.

## Motor Outputs

| Motor | Mean | Std | Offset |
|-------|------|-----|--------|
| C1 | 1573 | 10 | +0.5% |
| C2 | 1531 | 9 | **-2.2%** |
| C3 | 1591 | 11 | **+1.7%** |
| C4 | 1565 | 11 | 0.0% |

ESC RPM: 30-33k RPM mean. Slight CG offset (M2 low, M3 high).
Pitch trim +2.44 deg confirms forward CG bias.

## Post-Landing

- PD jumped from -0.82 to +1.68 at disarm (2.5m EKF reset)
- BAlt jumped from -2.01 to +1.32 (3.3m baro swing from prop wash stopping)
- "EKF variance: position lost" and FAILSAFE_EKFINAV at +1s after disarm

## Key Findings

1. **Excellent altitude hold** (2.5cm std) — best performance of any outdoor flight
2. Rangefinder providing dominant height fusion despite EK3_RNG_USE_HGT=-1
3. Surface tracking correctly handled terrain step
4. XKF3 velocity innovations stuck (may be log resolution issue)
5. 32°C cold start temperature rise — needs pre-warm or longer stabilization
6. Z-axis vibration elevated but not clipping
7. Post-disarm EKF collapse typical for GPS-denied vehicle

## See Also
- [logm2_5](logm2_5.md) — same vehicle, flight 2
- [logm2_6](logm2_6.md) — same vehicle, flight 3 (altitude failure)
