# Arming: 100m Infiltration Run

## Purpose

Test EKF behavior when the drone is armed and the operator sprints 100m to a launch
position before flying. This is the most extreme carry scenario - high acceleration,
high vibration from running, and significant position displacement while armed.

## Real-World Procedure

1. Arm drone on level surface
2. Pick up and sprint 100m to launch position (~30s at running pace)
3. Place on ground (or hand-launch)
4. Take off to 10m in ALT_HOLD
5. Hover for 30s
6. Land

## SITL Simulation

**Feasibility:** PARTIAL - `SIM_SHOVE` provides a sustained forward velocity but auto-resets.

### Parameters

```
SIM_SHOVE_X    = 3       # Forward velocity ~3m/s (jogging speed)
SIM_SHOVE_TIME = 10000   # 10s shove duration (~30m displacement)
```

### Limitations

- `SIM_SHOVE` auto-resets after duration - maximum ~30m displacement, not 100m
- No running vibration model (real running adds ~2-3g vertical oscillation at ~3Hz)
- No GPS position update during shove (SITL GPS tracks actual position)
- Displacement is a useful approximation but not fully representative

### Test Sequence

1. Arm in ALT_HOLD
2. Trigger SIM_SHOVE with forward velocity
3. Wait for shove to complete
4. Take off to 10m
5. Hover for 30s
6. Check altitude and position stability
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| No EKF failsafe during run | Yes | No |
| EKF tracks GPS position | Within 10m after settle | > 20m error |
| Alt hold after run | < 3m drift | > 5m drift |
| Height innovation (SH) | < 1.0 | > 1.5 |
| Successful takeoff after run | Yes | No |

## Key Log Signals

- `GPS.Lat/Lng` vs `NKF1.PN/PE` - GPS vs EKF position during run
- `IMU.AccX` - forward acceleration profile
- `NKF4.SP/SH` - position and height innovation ratios
- `NKF1.PD` - altitude estimate stability during run

## Related Logs

- High-dynamics pre-flight movement logs
