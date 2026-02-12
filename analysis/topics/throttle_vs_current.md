# Throttle vs Current for Baro Thrust Compensation

## Summary

Compared throttle command vs battery current as predictors of baro pressure
error across 17 logs from two vehicles (indoor logjk drone and outdoor logtd drone),
at multiple altitude bands.

**Conclusion: Stick with throttle for BARO1_THST_SCALE.** The theoretical advantage
of current (captures real thrust variations) only appears at high altitude in wind —
exactly where baro thrust compensation is irrelevant.

## Results at 0-3m (where thrust compensation matters most)

| Log | N | Thr→Baro | Curr→Baro | Thr-Curr corr | Winner |
|------|------|----------|-----------|---------------|--------|
| [logjk1](../logs/logjk1.md) | 53 | -0.724 | -0.735 | 0.923 | ~same |
| [logjk2](../logs/logjk2.md) | 312 | -0.491 | -0.413 | 0.952 | THR |
| [logjk3](../logs/logjk3.md) | 154 | -0.506 | -0.208 | 0.432 | THR |
| [logjk4](../logs/logjk4.md) | 22 | -0.578 | -0.399 | 0.754 | THR |
| [logjk5](../logs/logjk5.md) | 47 | -0.788 | -0.480 | 0.521 | THR |
| [logjk6](../logs/logjk6.md) | 100 | -0.615 | -0.495 | 0.537 | THR |
| [logjk8](../logs/logjk8.md) | 42 | -0.800 | -0.516 | 0.570 | THR |
| [logtd1](../logs/logtd1.md) | 46 | +0.093 | +0.075 | 0.997 | ~same |
| [logtd2](../logs/logtd2.md) | 1775 | -0.106 | -0.112 | 0.974 | ~same |
| [logtd3](../logs/logtd3.md) | 43 | +0.232 | +0.591 | 0.695 | CURR |
| [logtd4](../logs/logtd4.md) | 54 | -0.387 | -0.302 | 0.988 | THR |
| [logtd5](../logs/logtd5.md) | 47 | -0.422 | -0.256 | 0.968 | THR |
| [logtd6](../logs/logtd6.md) | 40 | -0.352 | -0.249 | 0.986 | THR |
| [logtd7](../logs/logtd7.md) | 104 | -0.165 | -0.213 | 0.939 | CURR |

**Score: Throttle 9, Current 2, Same 3**

## Results at 3-8m

**Score: Throttle 4, Current 0, Same 4**

Notable: logjk2 (thr=-0.740 vs curr=-0.534), logjk3 (thr=-0.775 vs curr=-0.151),
logtd1 (thr=-0.807 vs curr=-0.647)

## At 15-30m hover

Current outperformed throttle only in logtd6 LOITER at 18m:
- Throttle: corr=0.333, 5.7% noise reduction
- Current: corr=0.597, 19.8% noise reduction
- Throttle-current correlation was only 0.412 (windy conditions)
- 91% of current variation was NOT explained by throttle

## Why Throttle Wins at Low Altitude

1. **Current sensor noise** — battery shunt sensors have significant measurement noise
   that masks the baro pressure correlation
2. **Non-motor current** — avionics, servos, LEDs draw current but produce no thrust
3. **Throttle is already linearized** — motor mixer applies MOT_THST_EXPO, so throttle
   output ≈ thrust (no need for current to capture the nonlinearity)
4. **No wind indoors** — throttle accurately predicts thrust without wind disturbance
5. **Motor inertia** — current spikes during acceleration don't immediately produce
   thrust/propwash

## Why Current Wins at High Altitude in Wind

- Wind gusts change actual thrust (current) without changing throttle command
- Battery voltage sag affects thrust-per-throttle ratio
- But this only matters where propwash doesn't affect the baro anyway
