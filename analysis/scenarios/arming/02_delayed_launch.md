# Arming: Delayed Launch

## Purpose

Test EKF stability when there is a significant delay (30-60s) between arming and takeoff.
During the delay, baro drift, IMU bias drift, and thermal effects can accumulate. The EKF
must maintain accurate state estimates through the idle period so altitude hold works
correctly at launch.

## Real-World Procedure

1. Place drone on level surface
2. Wait for GPS fix and EKF healthy
3. Arm in STABILIZE
4. Wait 30-60s without taking off (simulating operator preparation)
5. Take off to 10m
6. Switch to ALT_HOLD, hover for 30s
7. Land

Repeat for each prop size: 2.5", 3.5", 5", 7", 8"

## SITL Simulation

**Feasibility:** FULL - simply add a delay between arm and takeoff in the test sequence.

### Parameters (per prop size)

Same as immediate launch:

| Prop | MOT_THST_HOVER | SIM_ACC1_RND | SIM_VIB_MOT_MAX | SIM_BARO1_WCF_DN |
|------|---------------|-------------|-----------------|------------------|
| 2.5" | 0.09 | 5.0 | 400 | -0.001 |
| 3.5" | 0.15 | 3.0 | 300 | -0.0008 |
| 5"   | 0.25 | 1.5 | 200 | -0.0005 |
| 7"   | 0.40 | 0.8 | 150 | -0.0003 |
| 8"   | 0.50 | 0.5 | 120 | -0.0002 |

### Test Sequence (per prop config)

1. Set prop-specific parameters
2. Reboot SITL
3. Arm in ALT_HOLD
4. Wait 45s (simulated delay)
5. Take off to 10m
6. Hover for 30s
7. Check altitude stability
8. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Successful arm | Yes | No |
| EKF healthy after 45s wait | Yes | No |
| Takeoff to target alt | Within 3m in 15s | > 5m error or > 30s |
| Alt hold drift (30s) | < 2m | > 3m |
| No initial altitude jump | < 1m | > 2m |
| Height innovation (SH) | < 0.8 | > 1.5 |

## Key Log Signals

- `NKF1.PD` - EKF position down during idle period (should stay ~0)
- `BARO.Alt` - baro drift during idle
- `CTUN.Alt` at takeoff - initial altitude estimate accuracy
- `NKF4.SH` - height innovation before and after takeoff
- `IMU.GyrZ` - gyro bias stability during idle

## Related Logs

- Any flight log with visible idle time between arm and takeoff
