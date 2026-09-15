# Tray landing accuracy on LightDynamix show drones

Analysis sessions, 2026-09-14 (tray landings, firmware 73e18022 on
lightdynamix-motherboard, APJ 5281) and 2026-09-15 (fast oscillation,
firmware 8e72a9cb on lightdynamix-pixelmb, APJ 5282). ArduCopter
V4.6.3-lightdynamix, drone show mode (127), u-blox ZED-F9P RTK at 5 Hz,
GPS as the height source, no rangefinder.

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

## Recommended settings

| Parameter | Value | Why |
|-----------|-------|-----|
| PSC_POSXY_P | 2.0 | step 2 |
| PSC_VELXY_P | 4.0 | step 2 |
| PSC_VELXY_I | 2.0 | step 2 |
| PSC_VELXY_D | 0.5 | margin; the Copter firmware default is 0.25 |
| PSC_VELXY_FF | 0.2 | as flown |
| ATC_INPUT_TC | 0.10 | halves attitude lag, restores margin for step 2 |
| PSC_JERK_XY | 40 | show tracking; LAND handover without an end hold |
| WPNAV_ACCEL | 500 | show tracking; LAND handover without an end hold |
| LAND_SPEED | 30 (up to 50) | less time in the near-ground push, fast land detect |
| SHOW_VEL_FF_GAIN | 1.0 | removes the command-rate ripple, halves show tracking error |
| SHOW_CTRL_RATE | 25 | smaller target steps, ripple moved above the attitude loop |
| EK3_POSNE_M_NSE | 0.2 | only on airframes with healthy RTK |
| EK3_VELNE_M_NSE | 0.15 | only on airframes with healthy RTK |
| EK3_VELD_M_NSE | 0.2 | only on airframes with healthy RTK |

Show trajectories that end at height should end with at least 2 s of
hover over the landing point. Without it, SHOW_VEL_FF_GAIN 1.0 with this
shaper leaves up to 4.6 cm of LAND target error, and much more if the
shaper is softened back to PSC_JERK_XY 20 / WPNAV_ACCEL 800.

## Open items

- Drone 193: repeated RTK fix loss with corrections flowing. Check the
  antenna, ground plane and cable before judging it on either EKF
  setting, and find the cause of the post-landing glitch in log 44.
- mode_drone_show.cpp landing_start(): steer to the show end point instead
  of a velocity-only handover, and optionally hold the descent until the
  XY error is a few cm.
- Ground phase: relax XY control and clear the integrator once RTK height
  says the drone is within a few cm of the landing surface, so a drone
  resting off-centre is not pushed until it tips.
- Wind near the ground: bleeding the XY integrator with height in the
  last ~0.5 m would match the show's wind collapse, but it depends on the
  site and tray walls. Needs more windy data before trying.
- For testing, fly with LOG_DISARMED=1 so logs start at boot (faithful
  Replay) and capture post-landing events. Trim back for shows.
- lightdynamix-pixelmb defaults.parm still sets SHOW_VEL_FF_GAIN 0.6 and
  does not set SHOW_CTRL_RATE or PSC_VELXY_D; log 72's values were set on
  the drone.
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
