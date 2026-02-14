# Arming: Immediate Launch

## Purpose

Test the baseline arm-and-fly sequence with no delay between arming and takeoff. This
validates that the EKF is ready for flight immediately after arming across different prop
sizes (different vibration profiles and hover throttle points).

## Real-World Procedure

1. Place drone on level surface
2. Wait for GPS fix and EKF healthy
3. Arm in STABILIZE
4. Immediately raise throttle and take off to 10m
5. Switch to ALT_HOLD, hover for 30s
6. Land

Repeat for each prop size: 2.5", 3.5", 5", 7", 8"

## SITL Simulation

**Feasibility:** FULL - each prop config is simulated via vibration, hover throttle, and
wash parameters.

### Parameters (per prop size)

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
4. Immediately take off to 10m
5. Hover for 30s
6. Check altitude stability
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Successful arm | Yes | No |
| Takeoff to target alt | Within 3m in 15s | > 5m error or > 30s |
| Alt hold drift (30s) | < 2m | > 3m |
| No EKF failsafe | Yes | No |
| Height innovation (SH) | < 0.8 | > 1.5 |

## Key Log Signals

- `VIBE.VibeX/Y/Z` - vibration levels (vary by prop)
- `CTUN.ThO` - hover throttle (should match MOT_THST_HOVER)
- `CTUN.Alt` vs `CTUN.DAlt` - altitude tracking
- `NKF4.SH` - height innovation test ratio
- `IMU.AccZ` - Z accelerometer noise

## Related Logs

- First-flight logs for each prop configuration
