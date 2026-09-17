# Tray landing accuracy on LightDynamix show drones

Analysis sessions, 2026-09-14 (tray landings, firmware 73e18022 on
lightdynamix-motherboard, APJ 5281), 2026-09-15 (fast oscillation,
firmware 8e72a9cb on lightdynamix-pixelmb, APJ 5282) and 2026-09-16
(windy landings, the landing hold and a six-drone sweep, firmware
995f251d, af81d153 and 069d6dbf on lightdynamix-pixelmb) and 2026-09-17
(landing timing in emergencies, descent speed after the hold).
ArduCopter V4.6.3-lightdynamix, drone
show mode (127), u-blox ZED-F9P RTK at 5 Hz, GPS as the height source, no
rangefinder.

The complaint was that drones did not land accurately enough to seat in
their charging trays. The flight logs are real customer flights and are
not committed; they are referred to by name below.

## Outcome

With the settings at the end of this note, single drones seated in the
tray within 0.3-1.9 cm on every tray flight at LAND_SPEED 30, the land
detector fired 1.2-1.9 s after contact, and a three-drone show landed on
normal ground 4.6-8.2 cm from the show end points in a 1.1-1.4 m/s^2 wind
load.

Three separate problems were found, in order of size:

1. The LAND handover froze a 19.6 cm position overshoot into the landing
   target.
2. A lateral push in the last ~0.6 m that the default position controller
   rejected too slowly, made much worse by a 10 cm/s descent.
3. At 10 cm/s the land detector was slow, and the position integrator
   kept pushing a drone that was already sitting on the tray. One drone
   tipped off the tray edge and was disarmed by the crash check.

The EKF was never the limiting error. RTK fix losses on one airframe are
an open hardware item.

A follow-up found that two of the settings recommended for problem 1, the
stiffer XY shaper and ATC_INPUT_TC 0.10, made the show's 10 Hz position
commands drive a visible 10 Hz oscillation on fast show segments.
SHOW_VEL_FF_GAIN 1.0 with SHOW_CTRL_RATE 25 removed it and halved show
tracking error (section 6).

A fourth problem appeared once the drones flew in wind: the wind drops
off in the last 25 cm and the velocity integrator lags it, pushing the
drone upwind as it lands. Pausing the landing at 25 cm until the position
error settles removes it (section 7).

## Logs

"Contact" and "rest" are RTK positions against the takeoff spot (sweep
flights) or the show end point (show flights, which landed elsewhere).
"Target" is the LAND target against the show end point. "Veh" is the
vehicle against its target at contact.

| Log | Drone | Settings | Target | Veh | Contact | Rest | Notes |
|-----|-------|----------|--------|-----|---------|------|-------|
| 6   | 196 | original, LAND_SPEED 50 | 19.6 | 6.5 | 15.9 | 7.2 | mis-seated, 14 deg tilt |
| 8   | 196 | LAND_SPEED 10 | 19.6 | 20.5 | 12.0 | 6.1 | mis-seated, 11 deg tilt |
| 55  | 196 | JERK 40, ACCEL 500, LAND_SPEED 10 | 2.8 | 23.7 | 25.0 | 43.7 | fell off tray, crash disarm |
| 13  | 193 | step 1 gains, LAND_SPEED 30 | 2.3 | 1.8 | 4.7 | 0.3 | seated |
| 15  | 193 | step 2 gains, ATC_INPUT_TC 0.10 | 2.2 | 0.5 | 2.8 | 0.9 | seated |
| 43  | 193 | step 2 | 2.2 | 3.1 | 8.3 | 0.9 | seated, 4 RTK fix losses |
| 17  | 192 | step 2 | 0.4 | 1.2 | 4.0 | 0.7 | seated |
| 18  | 192 | step 2, EKF noise 0.2/0.15/0.2 | 0.5 | 0.7 | 4.0 | 0.3 | seated |
| 44  | 193 | step 2, EKF noise 0.2/0.15/0.2 | 2.1 | 2.7 | 4.1 | 1.9 | seated, glitch msg after landing |
| 22  | 191 | show, as 18 | 0.0 | 7.3 | 8.3 | 8.1 | ground landing, windy |
| 20  | 194 | show, as 18 | 0.0 | 6.0 | 6.8 | 8.2 | ground landing, windy |
| 19  | 195 | show, as 18 | 0.1 | 4.5 | 5.1 | 4.6 | ground landing, windy |

All values in cm. Drone identity was confirmed from the board serial in
the boot messages, not only SYSID_THISMAV: log 43 is the same airframe as
logs 13 and 15.

## 1. The LAND handover target error

The show ends with a fast traverse (6 m at 1.9 m/s in logs 6 and 8) and
LAND starts 0.03-0.09 s after the last guided command.
ModeDroneShow::landing_start() calls ModeLand::init(), and ModeLand runs
velocity-only horizontal control (input_vel_accel_xy with zero input). The
position controller's shaped desired position was still moving at about
20 cm/s, so it coasted to a stop 19.6 cm past the show end point and the
drone landed there. The TODO in landing_start() describes this case.

The residual velocity came from the guided position/velocity shaper.
SHOW_VEL_FF_GAIN 0.6 sends 60% of the trajectory velocity with the full
position, and PSC_JERK_XY 20 with WPNAV_ACCEL 800 gives a position
correction gain of 0.5 * 20 / 8 = 1.25 /s. The desired position lagged the
show by up to 0.77 m and overshot the end point.

A Python port of the AC_PosControl shaping (shape_pos_vel_accel_xy,
sqrt_controller, the ModeLand handover) driven by the logged GUIP commands
reproduced the logged desired position to under 1 mm (log 6) and 2 mm
(log 8). Predictions from it, worst case over handover timing:

| Change | LAND target error |
|--------|-------------------|
| As flown | 20 cm |
| SHOW_VEL_FF_GAIN 1.0 alone | 32 cm (worse: more velocity at handover) |
| PSC_JERK_XY 40, WPNAV_ACCEL 500 | 3 cm |
| PSC_JERK_XY 80, WPNAV_ACCEL 1000 (motherboard defaults.parm) | 7 cm |
| Hold >= 2 s at the show end | < 2 cm |
| Land toward the show end point (code) | 0.2 cm |

Log 55 flew PSC_JERK_XY 40 / WPNAV_ACCEL 500 and measured 2.8 cm, as
predicted. The later show held 27 s at the end and measured 0.0-0.1 cm.
The effective jerk is also capped by the attitude acceleration limits in
AC_PosControl::set_max_speed_accel_xy(), which the port includes.

The drones that flew logs 6, 8 and 55 did not carry the values in this
directory's defaults.parm (PSC_JERK_XY 80, WPNAV_ACCEL 1000, different
rate gains), so check what is actually loaded on each airframe.

SHOW_VEL_FF_GAIN 1.0 is only worse here because these shows ended while
still moving. With a hold at the end of the show it is the better setting;
see section 6.

## 2. EKF position and height

GPS height was the right choice. The barometer read 2.9 m at 2 m altitude
and dropped by up to 1.4 m in the tray's ground effect, but it is not
fused while EK3_SRC1_POSZ is GPS.

Against RTK the EKF was at the centimetre level throughout. In the log 6
descent the error was N 0.1 +/- 2.0, E -1.2 +/- 0.8, D -1.5 +/- 0.6 cm,
with innovation test ratios near zero. Two traps in measuring this:
compare against the EKF origin (ORGN Type 0), not home, which differed by
2-3 cm; and use RTK-fixed samples only.

EKF3 treats GPS position as no better than EK3_POSNE_M_NSE (0.5 m, and
1.5x that for height) and GPS velocity as no better than EK3_VELNE_M_NSE /
EK3_VELD_M_NSE, however good RTK claims to be.

- Replay of log 8, whose baseline replay matched the flight exactly:
  0.2 / 0.15 / 0.2 cut the horizontal EKF-RTK RMS from 2.5-2.8 to
  1.5-1.9 cm, reduced the height disagreement between cores from about
  +/-3 cm to +/-1.2 cm, did not raise output velocity noise, and raised
  the maximum test ratio from 0.17 to 0.27.
- Flight A/B on drone 192 (log 17 default, log 18 tighter, RTK fixed in
  both): descent RMS 1.6-1.8 to 0.9 cm, height offset on the tray -4.4 /
  -5.3 to -0.1 / -1.4 cm, velocity noise 1.0-1.3 to 0.7-0.9 cm/s, test
  ratios still below 0.2.

The cost shows up when RTK degrades. Drone 193 lost fix four times in
log 43 while corrections were still arriving every 0.03 s and the
receiver's AGC and jamming indicators were normal, which points at sky
view, multipath or the antenna installation. Sitting still on the tray in
DGPS for 2 s, its reported position wandered 20.5 cm with HAcc 0.21 m;
the EKF at the 0.5 floor followed 11-13 cm of it. At a 0.2 floor the
reported HAcc sets the weighting, about 5.7x more weight on GPS, so it
would follow more. That is an argument from the code: log 43 could not
be replayed faithfully (below).

The tighter floor also tightens the glitch gate. On the ground a GPS
position jump of about 1.4 m fails the innovation test at 0.2, against
about 3.5 m at 0.5, and the filter resets to GPS after 10 s. Log 44 (drone
193, tighter settings) logged "GPS Glitch or Compass error" and "PreArm:
GPS glitching" 47 s after landing. Logging had stopped at +15 s, so the
cause (the drone being moved, or a GNSS jump) is unknown.

The show (three drones, tighter settings) had no glitches, maximum test
ratios of 0.58 (velocity) and 0.08 (position), and one 1 s DGPS dropout
on drone 191 that the EKF handled (4.4 cm off when fix returned).

Log 8, logged from boot, replayed exactly. Log 43 began 6 s before
takeoff and its baseline replay diverged from the flight by metres, so
log from boot (LOG_DISARMED=1) when a replay may be wanted. Replay also
reads slowly from /mnt/c on WSL; copy logs to a local disk first (14 min
against 3 s).

## 3. Descent speed and the ground phase

At LAND_SPEED 10:

- Log 8 converged onto its (wrong) target at altitude, then spent about
  3.5 s below 30 cm and drifted 20 cm south-west before contact.
- The land detector needs the motors at their lower limit. With the
  target descending at 10 cm/s the throttle unwinds slowly: 2.9 s from
  contact to LAND_MAYBE in log 8.
- Until LAND_MAYBE, XY control runs at full authority. In log 55 the
  drone touched down 25 cm off, the position integrator wound up to a
  7.5 deg roll demand, the drone rolled off the tray edge at near-zero
  throttle, and the crash check disarmed it (AngErr 34 > 30). The large
  angle request also blocked the land detector.

At LAND_SPEED 30 contact to LAND_MAYBE was 1.2-1.9 s on every flight.

LAND_SPEED_HIGH cannot be set below LAND_SPEED to descend slowly high up
and quickly near the ground: land_run_vertical_control() clamps the high
descent rate to at least LAND_SPEED.

## 4. The near-ground lateral push

The external horizontal acceleration was recovered as dV/dt minus the
lean-angle acceleration (PSCN/PSCE at 10 Hz) and binned by RTK height.
Log 55 was flown in calm air:

| Log 55 | Error vs target | Push fluctuation | Mean push N | Roll error |
|--------|-----------------|------------------|-------------|------------|
| Show hover | 2.7 cm | 0.09 m/s^2 | 0.0 | 0.2 deg |
| 0.25-1.0 m | 6-9 cm | 0.20-0.24 m/s^2 | +0.1 to +0.4 | 0.8-1.2 deg |
| 0.06-0.25 m | 8-21 cm | 0.19-0.26 m/s^2 | -0.2 to -0.4 | ~0.9 deg |

The drones stay stable (about 1 deg attitude error) and the EKF agrees
with RTK, so this is an external force that the position loop rejects
slowly. A calm day and a push that reverses direction within 30 cm of
height point to propwash off the tray. That is probable, not proven; one
landing onto flat open ground at the same site would settle it.

The show landings had a different mechanism. Holding over the landing
points the wind load was 1.1-1.4 m/s^2 (sweep flights: 0.45-0.87). Between
1.5 m and 6 cm its east component collapsed from about 1.0 to 0.04-0.19
m/s^2 on all three drones, the integrators lagged, and all three reached
the ground 5-7 cm west, which is upwind. Propwash would not line up with
the wind on three drones 3 m apart. Ground landings have no tray funnel;
in the sweeps the funnel pulled 5-8 cm contact errors into the seat.

## 5. Position controller tuning

### Model

The XY loop was modelled per axis: AC_P_2D position P into AC_PID_2D
velocity PID (filtered error and derivative, integrator, velocity
feed-forward), driving a point mass through the measured attitude
response (first-order lag plus delay). Each flight's external
acceleration was fed back in. With the flown gains the model reproduced
the logged position error with correlation 0.93-1.00 on logs 6, 8 and 55,
and 0.72-0.99 on later flights where errors were near its 1-2 cm floor.
It assumes the push does not depend on the gains, and does not model
anything after contact.

### Attitude lag is ATC_INPUT_TC

ATC_INPUT_TC shapes the position controller's thrust-vector input
(AC_AttitudeControl::input_thrust_vector_heading), not only pilot input.
Identified from TAN/TAE against AN/AE:

| ATC_INPUT_TC | Measured lag (lag + delay) |
|--------------|----------------------------|
| 0.15 | 0.14 s (logs 6, 8, 13; both rate tunes) |
| 0.10 | 0.08-0.12 s (logs 15, 17, 18, 43, 44, show) |

Drone 196 carries an older rate tune than the rest of the fleet
(ATC_RAT_RLL_P 0.074 vs 0.134, ATC_RAT_PIT_P 0.111 vs 0.186, ATC_ANG_*_P
20 / 30 vs 15, MOT_THST_EXPO 0.52 vs 0.62, MOT_SPIN_MIN 0.15 vs 0.09),
but its lag at the same ATC_INPUT_TC was the same.

### Gain sweep

Disturbances from logs 8, 13, 15 and 55 at 30 cm/s. Margin is phase
margin with the measured lag, and in brackets with 50 ms of extra delay.
Sensitivity peak is the loop's worst-case disturbance amplification.

| POSXY_P / VELXY_P / VELXY_I, INPUT_TC | Margin | Sens. peak | Worst error < 0.6 m (mean) |
|---------------------------------------|--------|------------|----------------------------|
| 1 / 2 / 1, 0.15 (default) | 61 (55) deg | 1.22 | 7.1 cm |
| 1.5 / 3 / 1.5, 0.15 (step 1) | 54 (44) deg | 1.29 | 4.5 cm |
| 2 / 4 / 2, 0.10 (step 2) | 60 (46) deg | 1.41 | 3.2 cm |
| 2.5 / 5 / 2.5, 0.10 | 53 (36) deg | 1.51 | 2.7 cm |
| 3 / 6 / 3, 0.10 | 46 (26) deg | 1.64 | 2.2 cm |

- Step 2 without the ATC_INPUT_TC change has only 45 (33) deg.
- PSC_VELXY_D 0.25 cost margin (45 deg). 1.0 raised the sensitivity
  peak to 1.75 and velocity-noise gain by about 70%.
- Descent speed mattered more than gains at 10 cm/s. On log 55's
  disturbance: default gains at 10 cm/s 23.6 cm, at 30 cm/s 6.1 cm, step
  1 at 30 cm/s 4.4 cm.

### Flown

- Log 13 (step 1, drone 193) and log 15 (step 2, same drone and tray)
  both seated. Replaying each flight's disturbance through the other gain
  set, step 2 was better on both: 2.9 to 2.3 cm, and 5.6 to 3.1 cm.
- Step 2 health across logs 15, 17, 43: no resonant peak in the
  position-error or lean-command spectra (averaged over all quiet show
  time), lean activity raised broadly around 0.5-0.9 Hz (ratio to the
  0.1-0.3 Hz band 0.18-0.24, against 0.14 on step 1), roll rate output up
  about 50%, no motor saturation.
- In log 15's contact error budget the vehicle against its target was
  0.4 cm, the smallest term. Beyond step 2 the gain sweep buys about
  0.5 cm per step, inside the model's error, at a steep margin cost.
- In the windy show, the model gives 0.6-1.5 cm back from LAND_SPEED 50
  or 2.5 / 5 / 2.5.

## 6. Fast oscillation at the show command rate (2026-09-15)

Two pixelmb drones flew the same show with RATE, ANG and PID logging at
400 Hz, which the tray logs did not have:

| Log | Drone | Show settings | Other differences |
|-----|-------|---------------|-------------------|
| 29 | 162 | SHOW_VEL_FF_GAIN 0.6, SHOW_CTRL_RATE 10 | PSC_POSXY_P 3, PSC_JERK_XY 50, MOT_SPIN_MIN 0.30 (defaults problem) |
| 72 | 165 | SHOW_VEL_FF_GAIN 1.0, SHOW_CTRL_RATE 25 | PSC_POSXY_P 2, PSC_JERK_XY 40, MOT_SPIN_MIN 0.09 |

Both had WPNAV_ACCEL 500, ATC_INPUT_TC 0.10, PSC_VELXY_D 0.25 and drone
193's rate gains. The show peaks at 2.8 m/s with 23 s above 1 m/s.

### What it was

In log 29 the oscillation was a narrow line at exactly 10.0 Hz, with a
20 Hz harmonic, present only while the show was moving. Above 1 m/s show
speed the rate demand at 9-11 Hz was 12.7 dps RMS and the rate controller
output 8-11% of full authority. The airframe rolled about +/-0.3 deg at
10 Hz. With the show stationary the same band was 0.3-0.5 dps and under 1%.

It was not the rate loop. In steady flight the actual roll and pitch rates
have no bump near 8-10 Hz. rate_response.py reported the rate loops as
under-damped at 8-9 Hz, but that was noise at the edge of its trustworthy
band.

### Mechanism

The show manager sends guided position and velocity at SHOW_CTRL_RATE with
no acceleration, and scales the velocity by SHOW_VEL_FF_GAIN. Guided mode
propagates its target with that velocity between updates. At 0.6 the
target falls behind and jumps forward by 0.4 * speed / SHOW_CTRL_RATE at
every update. Measured jumps per update were 2.9 cm at 0.5-1 m/s, 5.6 cm
at 1-2 m/s and 9.6 cm at 2-3 m/s, matching that formula. The XY shaper
turns each jump into an acceleration pulse, which goes straight to the
lean target and becomes a sawtooth in rate demand locked to the commands.

The shaper port from section 1, plus a first-order ATC_INPUT_TC, driven by
log 29's commands reproduced the 10 Hz lean ripple: 0.028 m/s^2 against
0.026 logged, and a lean-target rate above 5 Hz of 15.9 dps against
15.3-16.2 logged depending on the window. Varying the settings on the
same commands:

| Settings | Ripple (dps) |
|----------|--------------|
| PSC_JERK_XY 20, WPNAV_ACCEL 800, ATC_INPUT_TC 0.15 (original) | 2.7 |
| PSC_JERK_XY 40-50, WPNAV_ACCEL 500, ATC_INPUT_TC 0.15 | 10.7 |
| PSC_JERK_XY 40-50, WPNAV_ACCEL 500, ATC_INPUT_TC 0.10 (as flown) | 15.9 |

With ATC_ACCEL_R/P_MAX 500000 the effective jerk is capped at 32.7 m/s^3,
so PSC_JERK_XY 40 and 50 are the same. The PSC gains do not enter this
path.

Trade-off on the same commands at ATC_INPUT_TC 0.10. "Lag" is how far the
shaped target trails the show trajectory, mean over the fast segments:

| JERK_XY / WPNAV_ACCEL | FF gain | Command rate | Ripple | Lag |
|-----------------------|---------|--------------|--------|-----|
| 40 / 500 | 0.6 | 10 Hz | 15.9 dps | 26 cm |
| 40 / 500 | 0.6 | 50 Hz | 4.8 dps | 22 cm |
| 40 / 500 | 1.0 | 10 Hz | 9.5 dps | 5 cm |
| 40 / 500 | 1.0 | 25 Hz | 7.8 dps | 4 cm |
| 20 / 800 | 1.0 | 10 Hz | 3.7 dps | 26 cm |
| 20 / 800 | 1.0 | 25 Hz | 3.1 dps | 24 cm |

Sending acceleration with the commands could not be evaluated: a spline
through 10 Hz samples gives unusable acceleration, and the real trajectory
would have to come from the show file.

### Flown

Log 72 flew SHOW_VEL_FF_GAIN 1.0 at 25 Hz with the 40 / 500 shaper:

| Show speed > 1 m/s | Log 29 | Log 72 |
|--------------------|--------|--------|
| Target jump per update | 5.6-9.6 cm | 0.5 cm |
| Rate demand at command rate | 12.7 dps | 1.9 dps |
| Rate output at command rate | 8-11% | 2-3% |
| Airframe attitude ripple at command rate | ~0.3 deg | ~0.007 deg |
| Lean-target rate above 5 Hz | 15.3 dps | 4.8 dps |
| SHOW.HDist mean / p95 / max | 15.8 / 31 / 41 cm | 6.9 / 16 / 19 cm |

- Measured ripple beat the model's 7.8 dps prediction.
- Scheduler load was 47-50% in both logs with no long loops, so 25 Hz
  commands cost nothing measurable.
- Hover error with the show stationary rose from 1.5 to 3.1 cm mean, a
  different drone and day and PSC_POSXY_P 3 to 2. That is in line with the
  step 2 tray and windy show flights.
- Lean activity at 0.5-0.9 Hz fell back to step 2 levels (ratio 0.60 to
  0.20). The earlier level came from PSC_POSXY_P 3 with PSC_VELXY_D 0.25,
  which at the measured 0.10 s lag leaves about 31 deg of phase margin.
- PSC_VELXY_D 0.25 is ArduCopter's firmware default; the tray drones
  carried 0.5 from their parameter files. At 2 / 4 / 2 it gives 40 deg of
  margin (28 deg with 50 ms extra delay) against 55 (41) at 0.5.

With SHOW_VEL_FF_GAIN 1.0, section 1's handover replay gives up to 4.6 cm
of LAND target error with the 40 / 500 shaper if a show ends while still
moving, and up to 32 cm with 20 / 800. A hover of at least 2 s at the end
keeps it under 2 cm either way. This show descended to the ground itself,
so it was not affected.

Log 72 also showed two single GPS records with no fix and 0 satellites,
4 s apart mid-show, each with a dip in GPS UART receive bytes and RTK fixed
either side; the EKF and position were unaffected. Before flight it
reported a low magnetic field (140 against a minimum of 185) and a ground
magnetic anomaly realignment at takeoff.

## 7. Windy landings and the landing hold (2026-09-16)

Drones 161, 162, 165 and 193 flew repeated tray landings in wind giving
0.7-1.7 m/s^2 of hover load, about 1.4x the earlier sweeps. Seating was
not clearly worse (4.5 cm median contact against the seat, against 4.0 cm
in calm air, and the funnel still pulled them to 0.6 cm) but tracking
was: the vehicle sat 4.2 cm from its own target at contact against 1.5 cm
before, and on 11 of 13 landings that error pointed upwind.

### The wind falls off near the ground

External horizontal acceleration during the descent, as a fraction of
each flight's hover wind load, over all 16 windy landings:

| Height (m) | 1.5-2 | 1-1.5 | 0.6-1 | 0.4-0.6 | 0.25-0.4 | 0.15-0.25 | 0.08-0.15 | contact |
|---|---|---|---|---|---|---|---|---|
| push / hover load | 0.83 | 0.74 | 0.70 | 0.55 | 0.56 | 0.44 | 0.25 | 0.19 |

Half the fall happens in the last 25 cm. The velocity integrator still
holds the lean the wind needed higher up, which pushes the drone upwind
as it lands: the median along-wind error is under 1 cm between 0.15 and
0.4 m, then 2.9 cm at 0.08-0.15 m and 3.7 cm at contact.

### What the model says to do about it

The section 5 model was re-fitted to nine of the windy flights; the other
seven are dominated by RTK position steps, which are not forces and which
the model cannot reproduce. Median error against the landing target at
contact, from 1.0 m:

| Change | Model, contact error |
|--------|----------------------|
| as flown, P2 / VP4 / VI2, LAND_SPEED 15 | 4.4-4.7 cm |
| VELXY_I 3 | 3.7-4.0 |
| VELXY_I 4 | 3.2-4.0 |
| LAND_SPEED 10 | 2.5-2.7 |
| LAND_SPEED 5 (19 s of descent) | 2.6-3.7 |
| hold 2 s at 30 cm, then 30 cm/s | 2.5-3.4 |
| hold at 20 cm until the error settles, then 30 cm/s | 1.3-2.2 |

The pair of numbers is the spread between two ways of splitting the
recorded disturbance into a height-bound part, which follows a re-timed
descent, and gusts, which do not. Descending slowly the whole way is not
the answer: the quasi-static error does fall with descent speed, but gust
exposure and land-detect time grow with it, and at LAND_SPEED 5-10 the
tip-off risk of section 3 returns. Settling at one low height and then
finishing at LAND_SPEED helps in the calm-day disturbances as well
(1.7-1.8 cm against 2.2-2.3 cm at a constant 30 cm/s).

### The hold

ModeDroneShow::landing_hold_needed() pauses the descent at SHOW_HOLD_ALT
while XY control keeps running, and releases when the horizontal error
has been below SHOW_HOLD_ERR for 0.5 s, never before SHOW_HOLD_TMIN and
never after SHOW_HOLD_TMAX. It skips the hold if the position estimate is
lost, and reports one line per landing, for example "Landing: settled
after 3.3 s, error 1 cm". Only show landings use it; failsafe and RTL
landings are untouched. LAND mode takes the pause through a descent hold
that is separate from land_pause, which clears itself after 4 s.

The descent takes about 0.3 s to stop, so the trigger leads by that much
of the current descent rate. Without the lead the drone stopped 5.5 cm
low at 21 cm/s and 7 cm low at 24 cm/s.

SITL, real show file, tray gain set, 5 m/s wind decaying to the ground
(SIM_WIND_T 0, SIM_WIND_T_ALT 2.0, which brackets the measured profile),
eight wind directions:

| Configuration | Contact error, median (worst) | Descent |
|---------------|-------------------------------|---------|
| no hold, LAND_SPEED 30 | 6.5-7.4 cm (7.9) | 5.5 s |
| hold at 25 cm, LAND_SPEED 30 | 1.4-1.5 cm (1.8) | 9.5 s |
| no hold, LAND_SPEED 15 | 4.1 cm (4.6) | 9.5 s |

The third row matters: taking the same total time by descending slowly
only gets half the gain. Edge cases behave - a hold altitude above the
show end height holds for SHOW_HOLD_TMIN and lands normally, an error
threshold that never settles ends at SHOW_HOLD_TMAX, and SHOW_HOLD_TMAX 0
lands as if the hold were off.

### Retry and time limit

The hold runs once, so a gust in the last 25 cm still lands the drone
where it was pushed. SHOW_HOLD_AERR is the error during the final descent
that sends it back up to the hold altitude at 20 cm/s to settle again,
and SHOW_HOLD_TOUT limits the landing as a whole so holding can never
leave a drone hovering over its tray.

Retries stop below 40% of the hold altitude, and once the land detector
has seen anything. SITL found why: a retry triggered at 2 cm lifted a
drone that had already touched down, and the landing logged two ground
contacts.

In SITL with the wind stepped from 5 to 15 m/s as the hold released, the
error reached 8 cm, the drone climbed back, settled in 4.1 s and landed
4.3 cm from its target; a landing without the gust is unchanged at
2.4 cm. Forcing retries with a 0.5 cm threshold ends at SHOW_HOLD_TOUT
with "out of time, descending" and a single touchdown. SITL slews its
wind over SIM_WIND_TC, 5 s by default, so a gust test needs that set to
about 0.3 s or nothing happens.

### Flown

| Log | Drone | Hold settings | Wind load | Hold | Veh | Contact | Rest |
|-----|-------|---------------|-----------|------|-----|---------|------|
| 83 | 165 | 0.25 / 0.20 / - / 5 | 0.60 | 1.0 s | 3.2 | 6.5 | 1.9 |
| 63 | 193 | 0.15 / 0.03 / - / 5 | 0.49 | 1.0 s | 0.9 | 3.4 | 2.8 |
| 48 | 162 | 0.25 / 0.03 / 2 / 5 | 1.07 | 3.3 s | 1.0 | 3.3 | 1.2 |

Settings are SHOW_HOLD_ALT / ERR / TMIN / TMAX; wind load in m/s^2, the
rest in cm as in the first log table.

- Log 83 ran with SHOW_HOLD_ERR at 0.20 m. The parameter is in meters, so
  the error was under the threshold on arrival and the hold ended at its
  minimum. It still recovered the error from 4.5 to 2.7 cm, about this
  drone's 2.3 cm hover error, before the last 20 cm put 2 cm back.
- Log 63 arrived with 3.3 cm of error, already below its 0.03 threshold,
  and again released after a second. This is what SHOW_HOLD_TMIN was
  added for: the error dips below the threshold while the integrator is
  still moving.
- Log 48 is the first with both fixes and the first in real wind. The
  descent built 9.1 cm of error by 0.35 m; the hold ran 3.3 s at 23.5 cm
  and took it to 1.1 cm, and the final 25 cm only put 1.5 cm back. It
  seated 3.3 cm from the seat, rested 1.2 cm off and flat, and the land
  detector fired 1.4 s after contact.

During the hold the drone drifts up a few cm (5 cm on log 48). Harmless,
but the hold is not stationary vertically.

### VELXY_I 3

Eight flights, four drones, the same settings apart from PSC_VELXY_I and
with the hold at 0.25 / 0.03 / 2 / 5:

| Log | Drone | I | Load | Hold | Veh | Contact | Rest | Detect |
|-----|-------|---|------|------|-----|---------|------|--------|
| 64 | 161 | 2 | 0.79 | 2.0 | 2.0 | 4.4 | 1.1 | 1.4 |
| 65 | 161 | 3 | 0.65 | 2.0 | 0.9 | 0.7 | 1.3 | 1.5 |
| 51 | 162 | 2 | 1.04 | 2.1 | 1.3 | 0.9 | 0.8 | 1.6 |
| 52 | 162 | 3 | 0.90 | 2.0 | 2.1 | 1.4 | 1.3 | 1.9 |
| 94 | 165 | 2 | 0.55 | 1.9 | 2.6 | 2.9 | 1.8 | 1.8 |
| 95 | 165 | 3 | 0.48 | 2.0 | 1.0 | 3.2 | 1.0 | 1.7 |
| 69 | 193 | 2 | 0.67 | 1.9 | 1.4 | 5.5 | 1.9 | 1.5 |
| 70 | 193 | 3 | 0.52 | 2.0 | 0.9 | 2.9 | 2.1 | 1.4 |

Three of the four pairs improved, median vehicle against target 1.7 cm at
I 2 and 0.95 cm at I 3, but every I 3 flight happened to be in 8-17%
lighter wind, so the pairs are confounded. Replaying each flight's own
disturbance through both gain sets removes that: on the five flights the
model reproduces, the worst error below 0.6 m falls from 3.1 to 2.5 cm
and the error on arriving at the hold from 2.0 to 1.7 cm. About half a
centimetre, which is what the model predicted before the flights, with no
oscillation and no change in land detect time.

The wind was light on all eight, the hold released at its 2 s minimum
every time because the error was already below 3 cm on arrival, and the
errors sit near the model's 1-2 cm floor. The case that shows the
mechanism is still log 48 at 1.07 of load: 9.1 cm at 0.35 m, 1.0 cm at
contact.

Two of the eight rested about 10 deg tilted (162 log 52, 193 log 70)
after contact errors of 1.4 and 2.9 cm, so that is mechanical, not
approach accuracy. 162 has now done it four times and 193 twice.

### Gusty air, and what the hold does vertically

Log 57 (drone 162, 0.96 of hover load, gusty: the east push swung between
0.45 and 1.23 m/s^2 during the descent) is the first flight where the
hold did more than its minimum. It stopped at 24.6 cm, held 4.6 s while
the error rode the gusts (2.5, 5.8, 3.4, 6.5, 1.2 cm) and released at
1 cm; the final descent stayed under 2.7 cm, so the retry never fired. It
contacted 1.8 cm from its target and 5.1 cm from the seat, rested 1.2 cm
off and flat. That is 0.4 s short of SHOW_HOLD_TMAX, so in gustier air
the hold will end on the timeout rather than on settling.

Across the fourteen holds flown so far the drone rises 1-9 cm (median 4)
while it waits, and then descends at up to 41 cm/s against a LAND_SPEED
of 30. Descent tracking outside the hold is good, within 2 cm/s of the
target. The vertical position target is frozen when the descent stops
while ground effect lifts the drone above it - in log 57 it sat 5.8 cm
above target - and the position loop wins that height back on release at
about 1 cm/s per cm of error. The hold now snaps the target to the
current altitude when it releases, so the descent starts clean. SITL
does not show the effect: there the drone tracks its vertical target
closely, the excess descent is 4 cm/s either way, and the change only has
to be shown not to break anything.

### Compass and the tray

The trays suppress the field by 8-12% on all four drones (162 reads 502
against 559 mGauss in flight), which is what trips "ground mag anomaly,
yaw re-aligned" at takeoff and, on 162, "EKF compass variance" and "GPS
Glitch or Compass error" while it sits there after landing.

It is not a yaw problem. The in-flight yaw realignment moves yaw by less
than a degree on all four drones (+0.3, -0.3, -0.1, +0.5 deg) and the
yaw at landing is within 0.6 deg of the yaw on the tray, so the EKF is
not landing on a rotated heading. The 3.1 cm of EKF against RTK on log 57
is not explained by yaw.

What is worth fixing is the calibration spread. In flight the four
drones measure 456, 559, 613 and 623 mGauss at the same site on the same
day, against an EKF earth-field state of 492-517 mGauss, so their scales
sit between -7% and +20%. That error plus the tray's 10% is what pushes
the field checks over their thresholds. Recalibrate away from the trays
and the fleet should read within a few per cent of about 510 mGauss.
Note the shows fly at nearly constant yaw (1-6 deg of yaw span), so a
magfit from a show log is poorly conditioned; it needs a proper
calibration.

### EKF drag coefficients and the wind estimate

Drones 162 and 193 flew with EK3_DRAG_BCOEF_X/Y 58.77/51.02 and
EK3_DRAG_MCOEF 0. Drag fusion then ran and the wind states were learned,
but the estimates came out at 8-10 m/s where the airframe's own drag says
2.6 and 5.6 m/s. BCOEF models bluff-body drag, which grows with v^2, so
at BCOEF 55 it takes 6.6 m/s to explain 0.49 m/s^2 of drag. A multirotor
at show speeds is dominated by rotor momentum drag, which is linear in
speed and is what EK3_DRAG_MCOEF models.

Fitting body-frame XY specific force against GPS velocity (thrust is
along body Z, so that force is aerodynamic) gives 0.17-0.19 1/s on these
three airframes and 0.20-0.23 on the day-1 show logs. Replay of log 63
confirms it: with MCOEF 0.2 and BCOEF 0 the wind estimate settles at
2.5 m/s from 80 deg, which is what the fit says independently, and the
drag innovation bias halves. Adding BCOEF 15 over-predicts drag at show
speeds and flips the innovation positive.

With GPS this affects nothing - log 63 tracked to 1.2-1.9 cm all the way
down - but the wind estimate is what dead reckoning would fly on.

Flown with EK3_DRAG_MCOEF 0.2 on all four drones (BCOEF left at its old
values, which with MCOEF set adds about 15% of drag at 3 m/s), the
estimates came out at 2.3-3.6 m/s from 89-105 deg against 2.2-4.0 m/s
from 93-111 deg fitted per flight, and the drag innovation bias fell from
-0.230 m/s^2 to between -0.04 and +0.06. The wind states are no longer
absorbing a modelling error.

### Parameter names after the skybrush merge

Skybrush added its own SHOW_LAND_ALT at index 41, the altitude at which a
show hands control back to ArduPilot at its end, which is a different
thing from the hold. The hold parameters therefore moved to SHOW_HOLD_ALT,
SHOW_HOLD_ERR, SHOW_HOLD_TMIN, SHOW_HOLD_TMAX, SHOW_HOLD_AERR and
SHOW_HOLD_TOUT at indexes 42 to 47. Renaming ours rather than theirs keeps
the upstream name free, so the next merge does not fight over it and their
GCS cannot write a handover altitude into the hold height.

A drone flashed with this firmware reads its old SHOW_LAND_ALT value as
the upstream handover altitude and comes up with the hold off, so the hold
settings have to be entered again.

## 8. Six-drone sweep in moderate wind (2026-09-16)

Drones 161, 193 and 166 flew as one group and 162, 165 and 167 as the
other, on build 069d6dbf. There were 17 slots between 14:22 and 15:38
UTC. In every slot all six launched within a minute, and the two groups
flew opposite settings, so the difference between the group means in one
slot is a comparison in the same wind. The flight record page was not
used; each flight's settings were read from the parameters in effect when
it armed, and the slots were lined up by GPS time.

The procedure was reboot, change parameters, fly. Parameters written by a
normal MAVLink parameter set appear in that flight's log, but a bulk
upload over MAVLink FTP (@PARAM/param.pck) is saved without a log entry,
so it only shows up in the next boot's log. The last slot was set that
way (below), and nothing in its logs showed the change.

Starting settings were the recommended set with SHOW_HOLD_TMAX 7 and
PSC_VELXY_I 3. EK3_DRAG_BCOEF_X/Y were still 58.77/51.02. The wind load
was 0.6-1.6 m/s^2, median 1.0, about 5 m/s: moderate, not gusty.

### Results

82 usable landings in slots 0-15, leaving out flights without RTK fixed
at contact or for less than 90% of the descent (most of 193's):

| | Median | Range |
|---|---|---|
| Vehicle against target at contact | 2.9 cm | 0.3-11.6 |
| Contact against the seat | 3.8 cm | 0.7-11.9 |
| Resting against the seat | 0.9 cm | 0.1-7.6 |
| Land detect after contact | 1.6 s | 1.3-4.1 |
| Hold length (74 holds) | 2.2 s | 90th percentile 3.7, max 6.4 |

No hold timed out and no retry fired. Differences between the groups in
the same slot, first setting minus second:

| Comparison | Slots | Against target | Against seat | Resting |
|---|---|---|---|---|
| hold off vs on | 3 | +1.2 cm (+2.0, +2.2, -0.4) | 0.0 | +0.1 |
| AERR 0.05 vs 0.08 | 4 | -0.5 (-3.3, +1.6, +0.1, -0.3) | -0.3 | -0.5 |
| hold 0.15 vs 0.25 m | 3 | -0.1 (-0.6, +1.1, -0.6) | -0.6 | -0.7 |
| hold 0.15 m, LAND_SPEED 10 | 1 | -2.3 | -1.5 | +0.9 |
| TMAX 8 vs 7 | 4 | +0.6 (+0.9, +0.1, +0.7, +0.8) | +0.9 | +0.5 |

- The hold brought the drone 1.2 cm closer to its target at contact, but
  where it touched down and came to rest on the seat did not change: the
  funnel evens out errors of that size.
- The retry threshold, hold height and hold cap made no measurable
  difference. No hold approached 7 s and nothing fired a retry, so in
  this wind those settings were not doing anything. The TMAX row cannot
  be the cap and is group-to-group scatter.
- LAND_SPEED 10 below a 0.15 m hold landed closer in its one slot (3
  against 2 flights) but took 3.2 s to detect the landing against 1.8 s.
  Not enough to adopt, and slow land detection is what tipped a drone off
  the tray on day 1 (section 3).

### The last slot: retry and time limit in flight

The last slot flew SHOW_HOLD_ALT 0.15 and SHOW_HOLD_AERR 0.01, set by
bulk upload. A 1 cm retry threshold is below the normal 1-3 cm error in
this wind, so each release was followed by a retry. The error while
descending between retries shows the threshold at work: the drone passed
0.3-0.9 cm without retrying and retried at about 1 cm.

| Drone | Holds | Retries | Ended by | LAND to disarm | Against target | Against seat |
|---|---|---|---|---|---|---|
| 161 | 3 | 2 | settled, then descended | 12.9 s | 0.6 cm | 1.5 cm |
| 167 | 6 | 5 | time limit, already descending | 22.9 s | 0.8 cm | 1.1 cm |
| 162 | 6 | 6 | time limit | 23.6 s | 3.1 cm | 2.5 cm |
| 165 | 6 | 6 | time limit | 23.9 s | 2.9 cm | 4.2 cm |
| 166 | 5 | 5 | time limit | 24.3 s | 3.9 cm | 5.7 cm |

Every retry went back to its hold and settled again, SHOW_HOLD_TOUT ended
four of the five landings at exactly 20 s, and all five landed normally.
The drones first held at about 14 cm, so the stopping-distance lead also
works at 15 cm. "Landing: out of time, descending" is only reported when
the limit catches the drone holding; on 167 it was already descending
and the landing ended without a message.

With 0.01 retrying on almost every landing and 0.05 and 0.08 never firing
in 74 holds, the useful range sits well above the normal descent error.

### Tilted resting

30 of 91 landings came to rest 8-26 deg tilted, mostly 10-13 deg, on
every drone: 161 4/17, 162 5/17, 165 9/17, 166 3/17, 167 7/17, 193 2/8.
They were barely less accurate than the flat ones (contact 4.2 against
3.5 cm from the seat, rest 1.6 against 0.9 cm), so this is a second way
the drone sits on the tray rather than a missed approach.

### 193's crash (log 95, 15:04:57)

193's RTK was fixed for 0-85% of most descents. On log 95 it fell back to
DGPS at about 0.6 m during the LAND stage and the EKF switched lanes. The
GPS height stepped up 0.3 m and the EKF followed, so the drone reached
the tray believing it was 0.5 m up and never triggered the hold. The GPS
east position then jumped 0.4 m. Sitting tilted on the tray, the position
loop demanded up to 26 deg of pitch, the throttle rose to 0.73, and the
crash check disarmed it at an angle error of 44 deg. The hold played no
part. 193 was flying the tight EKF GPS noise (0.2 / 0.15 / 0.2), which
this note recommends only for airframes with healthy RTK; defaults.parm
now sets it for every drone.

### Drag coefficient from all 96 flights

Fitting body-frame IMU acceleration against GPS velocity, as in section
7, but pooling 95 minutes of flight with one wind per 20 s window:

| Fit | Result |
|---|---|
| Fleet, momentum drag | MCOEF 0.176 1/s, 95% 0.166-0.184 |
| Body X only / Y only | 0.176 / 0.179 |
| Bluff-body term | zero: BCOEF effectively infinite, no change in residual |
| 161 / 162 / 165 / 166 / 167 / 193 | 0.183 / 0.183 / 0.157 / 0.191 / 0.177 / 0.154 |

Airspeed covered a median 3.4 and 95th percentile 7.4 m/s, so this
holds across show speeds in wind. The fit is sensitive to the delay
between GPS velocity and the IMU: its residual is almost flat in the
delay, and scanning for the best value picked 0.30 s and MCOEF 0.198.
Measured directly against the EKF's delay-compensated velocity the GPS
velocity delay is 0.10-0.12 s (median 0.12 over 68 logs), which gives
0.173-0.180. The per-drone intervals overlap, so one fleet value is
enough.

## 9. Landing timing in emergencies (2026-09-17)

The skybrush team's concern was that the hold makes a landing late by a
variable amount, which eats into the separation in time that a collective
return to home (CRTH) plan relies on.

The hold only runs in the show mode landing stage, which is entered at
the end of the show and at the end of a CRTH trajectory. Failsafes that
change mode (RTL, LAND, battery, GCS, geofence) never reach it.

What it cost in the sweep at LAND_SPEED 30, leaving out LAND_SPEED 10 and
the landings that retried:

| | Landings | LAND stage to contact | LAND stage to land detect |
|---|---|---|---|
| hold off | 9 | median 3.2 s, max 3.4 | median 4.9 s, max 5.3 |
| hold on | 76 | median 5.6 s, 90th pct 8.2, max 11.0 | median 7.2 s, 90th pct 9.8, max 12.5 |

Four of the five forced-retry landings in the last slot took 20-21 s to
contact, which is what SHOW_HOLD_TOUT 20 allowed.

Two changes follow:

- Landings after a CRTH skip the hold. The plan times each drone's
  landing, and a CRTH is an emergency where getting down on time matters
  more than the last 1-2 cm. It uses skybrush's own
  is_collective_rth_triggered(), which is true once a CRTH command has
  added its scene to the screenplay.
- SHOW_HOLD_TOUT drops to 10 s. Sweep landings reached the hold 2-3 s
  after handover, so SHOW_HOLD_TMAX 7 still fits. Of the 76 held landings
  above only one (165 log 115, contact at 11.0 s) ran long enough that
  the limit might have shortened it. A landing that keeps retrying now
  releases 10 s after handover instead of 20, and touches down about 1 s
  later.

SITL, hover_3m_with_rth_plan.skyb, 3 m/s wind, CRTH sent through the time
axis configuration packet:

| Build | Landing | Held | LAND stage to landed |
|---|---|---|---|
| skip | end of show, handover at 2.6 m | 2.0 s at 24 cm | 11.5 s |
| skip | CRTH during the show, handover at 1.2 m | no | 4.4 s |
| skip | CRTH during takeoff, landed from 0.7 m | no | 5.0 s |
| skip disabled | CRTH during the show, handover at 1.2 m | 2.0 s | 7.8 s |
| skip disabled | CRTH during takeoff, landed from 1.3 m | 2.0 s | 12.8 s |

SHOW_LAND_ALT stays at the skybrush default of -1 (half the takeoff
altitude, at most 1 m). With SHOW_OPTIONS 2, as in defaults.parm, the end
of the show trajectory is rewritten to descend to the handover at landing
speed, so a lower handover moves where the descent changes from the show
to LAND without removing the time the hold adds. Without that option
SHOW_LAND_ALT only affects the CRTH handover, and the SITL show above,
flown without it, handed over at 2.6 m. A handover more than 3 cm below
SHOW_HOLD_ALT would make the drone climb back up to the hold height
before settling.

## 10. Descent speed after the hold (2026-09-17)

The one LAND_SPEED 10 slot in the sweep landed closer than LAND_SPEED 30,
but LAND_SPEED slows the whole descent from the handover and it took
3.2 s against 1.8 s to detect the landing. SHOW_HOLD_SPD sets the descent
speed from the hold to the ground on its own, and zero keeps LAND_SPEED.
It applies from the first hold onwards, including after a retry or the
time limit. Landings that never held, including those after a collective
RTH, stay at LAND_SPEED. A climb back after a gust and the hold itself
take priority over it.

SITL, hover_3m.skyb, 5 m/s wind decaying below 2 m, hold at 0.25 m,
LAND_SPEED 30:

| SHOW_HOLD_SPD | Final descent | Touchdown | Contact to land detect |
|---|---|---|---|
| 0 | 32 cm/s | 0.34 m/s | 2.0 s |
| 0.10 | 10 cm/s | 0.10 m/s | 3.6 s |
| 0.50 | 49 cm/s | 0.53 m/s | 1.5 s |
| 0.10, hold off | 29 cm/s | 0.30 m/s | 2.1 s |

The descent above the hold stayed at 29-30 cm/s in every run. With
SHOW_HOLD_AERR at 1.5 cm and turbulence, retries at 24 and 15 cm climbed
back to the hold at 15-16 cm/s and each final descent returned to
10 cm/s. A slower final descent leaves more time for the error to pass
SHOW_HOLD_AERR, so expect more retries at low thresholds.

Whether 10-15 cm/s seats more reliably than 30 cm/s, and whether the
slower land detect tips drones as on day 1 (section 3), is for a flight
test.

## Recommended settings

| Parameter | Value | Why |
|-----------|-------|-----|
| PSC_POSXY_P | 2.0 | step 2 |
| PSC_VELXY_P | 4.0 | step 2 |
| PSC_VELXY_I | 3.0 | step 2 was 2.0; 3 is worth about 0.5 cm near the ground |
| PSC_VELXY_D | 0.5 | margin; the Copter firmware default is 0.25 |
| PSC_VELXY_FF | 0.2 | as flown |
| ATC_INPUT_TC | 0.10 | halves attitude lag, restores margin for step 2 |
| PSC_JERK_XY | 40 | show tracking; LAND handover without an end hold |
| WPNAV_ACCEL | 500 | show tracking; LAND handover without an end hold |
| LAND_SPEED | 30 (up to 50) | less time in the near-ground push, fast land detect |
| SHOW_VEL_FF_GAIN | 1.0 | removes the command-rate ripple, halves show tracking error |
| SHOW_CTRL_RATE | 25 | smaller target steps, ripple moved above the attitude loop |
| EK3_POSNE_M_NSE | 0.2 | only on airframes with healthy RTK; default elsewhere (193) |
| EK3_VELNE_M_NSE | 0.15 | only on airframes with healthy RTK |
| EK3_VELD_M_NSE | 0.2 | only on airframes with healthy RTK |
| SHOW_HOLD_ALT | 0.25 | hold height, above the wind fall-off and clear of the funnel |
| SHOW_HOLD_ERR | 0.03 | above the hover error floor, which is 2-3 cm here |
| SHOW_HOLD_TMIN | 2 | the error dips below the threshold before the integrator settles |
| SHOW_HOLD_TMAX | 7 | never reached in the sweep; log 57 needed 4.6 s in gusty air |
| SHOW_HOLD_AERR | 0.08 | a gust this far off sends the landing back up to settle |
| SHOW_HOLD_TOUT | 10 | the landing always finishes, about 11 s from handover to contact at worst (section 9) |
| SHOW_HOLD_SPD | 0 | LAND_SPEED until a slower final descent is flown (section 10) |
| EK3_DRAG_MCOEF | 0.18 | fleet fit over 96 flights (section 8); 0 disables wind learning |
| EK3_DRAG_BCOEF_X/Y | 0 | the fit finds no bluff-body drag up to 7 m/s |
| SHOW_LAND_ALT | -1 | skybrush handover at half the takeoff altitude, at most 1 m |

Show trajectories that end at height should end with at least 2 s of
hover over the landing point. Without it, SHOW_VEL_FF_GAIN 1.0 with this
shaper leaves up to 4.6 cm of LAND target error, and much more if the
shaper is softened back to PSC_JERK_XY 20 / WPNAV_ACCEL 800.

## Open items

- Drone 193: repeated RTK fix loss with corrections flowing, and a crash on
  the tray after an RTK drop during landing (section 8). Check the
  antenna, ground plane and cable, and fly it on default EKF GPS noise
  until then.
- mode_drone_show.cpp landing_start(): steer to the show end point instead
  of a velocity-only handover. The descent hold is now implemented
  (section 7).
- The landing hold drifts up a few cm while it waits; worth finding out
  whether that is the vertical shaper or near-ground height error.
- A third of landings rest 10-13 deg tilted, on every drone (section 8).
  Check how the legs meet the funnel and charging contacts; it is not an
  approach problem.
- The hold has been flown in light and moderate wind only. The retry has
  fired in flight only with a deliberately tiny threshold, so its value
  in real gusts is still untested.
- defaults.parm sets the tight EKF GPS noise for every airframe, including
  ones with poor RTK such as 193.
- "Landing: out of time, descending" is not reported when the time limit
  catches the drone already descending.
- Parameters written by MAVLink FTP bulk upload are not logged, so a
  flight's log can miss the settings it flew. Record them separately, or
  log those writes in AP_Filesystem_Param.
- The drone still drifts up a few cm while holding; only the catch-up
  descent is fixed. Whether that matters is a question for a windier day.
- Ground phase: relax XY control and clear the integrator once RTK height
  says the drone is within a few cm of the landing surface, so a drone
  resting off-centre is not pushed until it tips.
- Wind near the ground: bleeding the XY integrator with height in the
  last ~0.5 m would match the show's wind collapse, but it depends on the
  site and tray walls. Needs more windy data before trying.
- SHOW_HOLD_SPD 0.10-0.15 against 0 in same-slot pairs: contact and
  resting error, land detect time, and tilted resting (section 10).
- For testing, fly with LOG_DISARMED=1 so logs start at boot (faithful
  Replay) and capture post-landing events. Trim back for shows.
- Show manager: send trajectory acceleration with the guided commands so
  the target propagates consistently between updates. Untested.
- Drone 165: the single-record GPS dropouts and the low magnetic field at
  launch in log 72.

## Corrections made during the session

These were stated and then withdrawn when the data disagreed.

- Log 8's slow-descent drift was first put down to wind shielding or tray
  airflow without a way to tell them apart. Calm-air log 55 favoured
  propwash off the tray. The windy show landings were then shown to be
  wind collapsing near the ground, so both mechanisms occur.
- An EKF-RTK comparison used the home position instead of the EKF origin,
  showing a spurious 3 cm height offset on log 8.
- Drone 196 was said to have a slower attitude response (0.15 s against
  0.10 s) and ATC_INPUT_TC 0.10 was said not to reduce drone 193's lag.
  Both were artefacts of a coarse identification grid; the fine grid
  showed the same 0.14 s lag on both tunes at 0.15 and 0.08 s at 0.10.
  Step 1 vs step 2 comparisons were recomputed with the correct lags.
- Tighter EKF GPS noise was recommended from the log 8 Replay, withdrawn
  after log 43's RTK dropouts, then flight-tested: better with healthy
  RTK, untested in flight during a dropout.
- The fleet check script flagged spectral "resonances" that came from
  short windows and a peak detector fooled by a falling spectrum, "slow
  land detect" from a contact-time estimate that depended on resting
  height, and "not seated" on show landings measured against the takeoff
  spot. All three were fixed.
- PSC_JERK_XY 40 / WPNAV_ACCEL 500 and ATC_INPUT_TC 0.10 were recommended
  without checking how they respond to the 10 Hz show commands; the tray
  logs were logged at 10 Hz and could not show it. Together they made the
  command-rate ripple about 6x larger (section 6).
- rate_response.py reported under-damped rate loops at 8-9 Hz on log 29.
  The steady-flight spectra show no resonance there, so that was noise at
  the edge of its band.
- Log 48 was read as flying the older firmware because its version string
  shows the commit the build was made from, and the hold fixes were not
  committed yet. The parameters in the log settle it: SHOW_HOLD_TMIN is
  present, so it is the newer build.
- The repeated retries in the sweep's last slot were first reported as an
  unexplained misfire, with a suggestion to disable the retry. That slot
  had SHOW_HOLD_AERR 0.01 and SHOW_HOLD_ALT 0.15, set by a bulk upload
  that does not appear in the log; the retry did what it was set to do.
- Zeroing the drag innovation bias on log 63 suggested MCOEF 0.25-0.27.
  The pooled fit at the measured GPS delay gives 0.176; the innovation
  mean is a weak measure because the wind states absorb a steady bias.
- A bluff-body term of BCOEF 15 was suggested from the AFS drift at
  4 m/s. Replay of log 63 shows it over-predicts drag at show speeds and
  flips the drag innovation positive, so momentum drag alone is the
  better fit below about 3 m/s.
