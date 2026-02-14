# Arming Scenarios

Arming and launch sequence tests that validate EKF3 behavior across different prop sizes
and pre-flight conditions (immediate launch, delayed launch, carry-to-launch).

## Prop Size Matrix

Each arming scenario is parameterized across five prop configurations:

| Prop | MOT_THST_HOVER | SIM_ACC1_RND | SIM_VIB_MOT_MAX | SIM_BARO1_WCF_DN |
|------|---------------|-------------|-----------------|------------------|
| 2.5" | 0.09 | 5.0 | 400 | -0.001 |
| 3.5" | 0.15 | 3.0 | 300 | -0.0008 |
| 5"   | 0.25 | 1.5 | 200 | -0.0005 |
| 7"   | 0.40 | 0.8 | 150 | -0.0003 |
| 8"   | 0.50 | 0.5 | 120 | -0.0002 |

Smaller props produce higher vibration and lower hover throttle. The wash-on-baro
coefficient (`SIM_BARO1_WCF_DN`) scales with disc loading.

## Scenarios

1. [Immediate Launch](01_immediate_launch.md) - Arm and take off immediately (FULL SITL)
2. [Delayed Launch](02_delayed_launch.md) - Arm, wait 30-60s, then take off (FULL SITL)
3. [Backpack Carry](03_backpack_carry.md) - Arm, carry on back, then launch (PARTIAL SITL)
4. [Hip Carry](04_hip_carry.md) - Arm, carry at hip, then launch (PARTIAL SITL)
5. [100m Infil Run](05_100m_infil_run.md) - Arm, sprint 100m, then launch (PARTIAL SITL)

## Key Log Signals

- `NKF1.PD` - EKF position down estimate
- `NKF4.SH` - height innovation test ratio (should stay < 1.0)
- `BARO.Alt` - raw barometric altitude
- `CTUN.ThO` - throttle output at launch
- `IMU.AccZ` - Z accelerometer during carry/run
- `VIBE.VibeX/Y/Z` - vibration levels per prop config
