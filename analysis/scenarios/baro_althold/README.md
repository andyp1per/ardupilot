# Baro AltHold Scenarios

Barometric altitude hold tests across different environmental conditions. These validate
that EKF3 Z-bias learning and altitude hold remain stable under thermal transients,
confined spaces, and baseline open-air conditions.

## Scenarios

1. [Open Space](01_open_space.md) - Baseline outdoor alt hold (FULL SITL)
2. [Narrow Hall](02_narrow_hall.md) - Indoor confined-space alt hold (PARTIAL SITL)
3. [Freezer Cold Boot](03_freezer_cold_boot.md) - Cold-start thermal transient (FULL SITL)
4. [Heat Gun Hot Boot](04_heat_gun_hot_boot.md) - Hot-start thermal transient (FULL SITL)

## Key Log Signals

- `BARO.Alt` - barometric altitude
- `BARO.Temp` - barometer temperature
- `NKF1.PD` - EKF position down (negative = up)
- `NKF4.SH` - innovation test ratio for height
- `XKF4.SH` - EKF3 innovation test ratio for height
- `CTUN.Alt` - desired vs actual altitude
- `CTUN.ThO` - throttle output
