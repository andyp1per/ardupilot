# Baro AltHold: Heat Gun Hot Boot

## Purpose

Test altitude hold when the flight controller is pre-heated (e.g., heat gun applied to
the board for rapid warmup, or drone sitting on hot tarmac in sun). The barometer starts
hot and cools as airflow from props increases convection. This is the inverse thermal
transient of the cold boot scenario.

## Real-World Procedure

1. Pre-heat flight controller to ~50C using heat gun or hot environment
2. Power on and wait for GPS fix
3. Arm and take off to 15m in ALT_HOLD
4. Hover for 60s while board cools from flight airflow
5. Monitor altitude for thermal drift (opposite direction from cold boot)
6. Land

## SITL Simulation

**Feasibility:** FULL - SITL thermal model works for hot start with faster cooldown.

### Parameters

```
SIM_TEMP_START    = 50     # Start temperature 50C (pre-heated)
SIM_TEMP_BRD_OFF  = 20     # Board runs 20C above atmospheric (less offset when hot)
SIM_TEMP_TCONST   = 30     # 30s cooldown time constant (faster due to prop airflow)
SIM_TEMP_BARO_FAC = 2.0    # Temperature sensitivity factor
```

### Temperature Model

With these params:
- T0 = 50C (hot start)
- T_sensor = ambient + 20C offset
- Time constant = 30s (shorter - prop wash accelerates cooling)
- Drift direction is opposite to cold boot

### Test Sequence

1. Set temperature simulation parameters
2. Arm in ALT_HOLD mode
3. Take off to 15m
4. Hover for 60s
5. Check altitude stays within +/-3m despite thermal drift
6. Verify EKF tracks the drift without failsafe
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Alt hold drift (60s) | < 3m | > 5m |
| Height innovation ratio (SH) | < 1.0 | > 1.5 |
| No EKF failsafe | Yes | No |
| Baro drift direction | Consistent with cooling | Erratic |

## Key Log Signals

- `BARO.Temp` - temperature falling from 50C toward steady state
- `BARO.Alt` - raw baro altitude (drift opposite to cold boot)
- `CTUN.Alt` vs `CTUN.DAlt` - actual vs desired altitude
- `NKF1.PD` - EKF position down
- `NKF4.SH` - height innovation test ratio

## Related Logs

- Hot-day flight logs, especially first flight after sun exposure
