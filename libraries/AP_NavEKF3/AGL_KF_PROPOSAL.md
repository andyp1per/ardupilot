# AGL KF as the height above ground

Notes for reviewing this branch, 2026-11-19. The branch is a prototype on
master to show the structure; it is for review, not for merging.

Proposal: make EKF3's AGL KF, the IMU-aided filter of the downward range
finder, the one height above ground. It would run wherever there is a
downward range finder, feed getHAGL() and Copter's surface tracking, and
leave terrainState holding only the ground position the filter still
needs.

## Today: three estimates, and Copter flies on the one with baro in it

Three filters estimate height above ground from the downward range
finder. Copter's controllers read the one that leans on the EKF height
between samples, so on master a range finder dropout leaves surface
tracking holding a baro height.

| Estimate | Where | What it fuses | Who reads it (master) |
|---|---|---|---|
| AGL KF | EKF3, one per core | Range finder plus that core's vertical acceleration: height, climb rate (and accel-Z bias in #33507) | Flow scaling and getHAGL(), only with EK3_OPTIONS bit 3 |
| terrainState | EKF3, one per core | Range finder, and flow with EK3_FLOW_USE 2, against the main filter's height | getHAGL() by default, the range finder height datum, the flow ceiling AC_Avoid uses |
| AP_SurfaceDistance | Copter | Raw range with a glitch check and a low-pass filter; the EKF height fills in between samples | Surface tracking, terrain following, land detector, landing speed, precision landing, low-altitude avoidance |

The only vehicle-side readers of get_hagl() are AP_GroundEffect and the
OPTICAL_FLOW message. FlowHold has its own flow-derived height but uses
no range finder; Plane's landing correction and Sub's surface tracking
keep their own copies.

## Measured

In flight logs the AGL KF follows the range finder with 0.02 s of lag
once it is reported at the current time, and through a dropout it keeps
the height far better than the EKF height surface tracking falls back on.
In SITL that takes the error out of surface tracking through a dropout
and changes nothing else.

Flight logs, 141 from the SmallFastDrone test fleet, 6,188 s armed with
the range finder in range:

| Estimate | Lag behind the range finder, median | Error when the range returns after a mid-range dropout, median of 52 |
|---|---|---|
| AGL KF as getHAGL() returns it | 0.09 s | 0.45 m |
| AGL KF moved to the current time | 0.02 s | 0.51 m |
| terrainState (getHAGL()'s default) | 0.00-0.07 s, spread over +/-0.3 s | 0.54 m |
| AP_SurfaceDistance's filtered height | 0.28 s | - |
| Last range plus the EKF height change, what surface tracking holds | - | 2.48 m |

The EKF's fusion delay itself measured 0.12 s, from its delayed range
sample.

SITL, ALT_HOLD over flat ground with surface tracking, master against the
prototype. Each cell is the most the true height above ground moved /
what it kept 10-15 s later:

| Event | master | prototype |
|---|---|---|
| Range dropout, 1.2-3.7 s | 0.05-0.10 m / all of it | 0.006-0.013 m / under 0.005 m |
| The same with a 0.15 m/s/s accel offset appearing after takeoff | 0.13-0.17 m / all of it | 0.07-0.09 m / under 0.07 m |
| 1 m baro step inside a 4.4 s dropout | 1.09 m / 0.97 m | 0.42 m / 0.13 m |
| 1 m baro step with the range good; 1 m ground step; climbs | unchanged | unchanged |

43 related autotests pass on the branch, Replay and the 14 the four PRs
add among them. The new SurfaceTrackingRangeDropout keeps 0.76 m of the
baro step on master and 0.00-0.02 m on the branch.

## The prototype

This branch, on master 5cdd6165d2, structure first and behaviour last:

1. #33359, #33478, #34457 and #33507 as they stand. #33478's XKFA.VHold
   is dropped, as its labels overflow the logger's 64 characters beside
   #33507's bias fields, and its test is adapted.
2. The AGL KF gets its own file and EK3_FEATURE_AGL_KF, which follows the
   range finder rather than optical flow. All 381 replayable fleet logs
   replay identically through it.
3. It runs wherever there is a downward range finder. EK3_OPTIONS bit 3
   is retired; bit 4, its velocity as velD, stays opt-in.
4. getHAGL() reports it at the output time, with getRangeFinderHAGL() for
   callers that want only the measured ground.
5. terrainState is renamed gndPosD, the ground's down position (rename
   only). It is then measured as the filter height plus the AGL KF height
   on every step the AGL KF takes a range sample, held while the range
   finder is the height source and kept below the vehicle otherwise.
   When the switch takes the range finder it comes from the range sample
   instead, #33359's reset made unconditional. The range half of the
   terrain estimator goes; EK3_FLOW_USE 2 keeps a flow-only part. Across
   EK3_RngHgtSwitchStepUp's 3.5 m step the EKF height moved 0.00-0.01 m,
   against -0.01 m on the four PRs alone.
6. AP_AHRS gets get_hagl_rangefinder(), only ever the estimator's range
   finder filter.
7. AP_SurfaceDistance carries a dropout on it. While the reading is
   healthy it stays the measurement, so SURFTRAK_GLDST and SURFTRAK_GLSAM
   work as before.
8. SurfaceTrackingRangeDropout, which fails on master.

Fleet Replay, 381 logs. Making the AGL KF mandatory moves 12 of them; the
restructure moves 148, by a median of 0.25 m. Against GPS, p95 velocity
error is within 0.05 m/s on 165 of 171 cores, better on 2 and worse on 4.
The four are throw logs already 4.6-14 m/s off GPS; one core of one goes
from 14 to 35 m/s.

Flash: BETAFPV-F405, which compiles out optical flow and so had no AGL
KF, grows 3,816 bytes; MatekH743 shrinks 616 bytes, as the terrain
estimator's range fusion goes.

## Decisions for review

1. Make the AGL KF mandatory wherever there is a downward range finder?
   It changes the height above ground, the flow scaling and the
   EK3_RNG_USE_HGT switch on every such vehicle that has bit 3 clear
   today, which is the default.
2. Surface tracking on the AGL KF only through a dropout, as the
   prototype does, or all the time? All the time breaks SURFTRAK_GLDST
   and SURFTRAK_GLSAM: the AGL KF turns a step into a ramp, so the glitch
   check never fires and SurfaceTracking2 climbs onto a 3 m step it
   should hold over. Default on, or behind an option (ARCH-19)?
3. Keep EK3_FLOW_USE 2's flow-only ground estimate? It is the one part of
   the terrain estimator the AGL KF cannot do.
4. Landing order: #33359, #33478, #34457 and #33507 first, as they are,
   then the restructure as one PR on top?
5. Compile the AGL KF out of the minimize_fpv_osd boards, which drop
   optical flow? It costs 3.8 KB there.

## Not covered yet

- Nothing on the branch has flown. The flight numbers above are logged
  estimates, not the loop.
- On about one flight in ten the AGL KF lags the range finder by
  0.3-0.8 s on its own, after the fusion delay is removed; all those
  flights ran EK3_AGL_ABIAS_P 0.05. Not yet explained; a Replay of those
  flights with a faster bias is the next check.
- The other PRs that lean on the AGL KF (#33484, #35147, #33585, #34851,
  #34380) are not on the branch; #33585's above-range fallback would keep
  its stored ground in gndPosD.
- The four throw logs that Replay worsens are not explained yet. Plane's
  landing range finder, Sub's surface tracking and EKF2 are untouched.

## Designs tried and dropped

- The EKF taking its height above ground from AP_SurfaceDistance: the
  EKF would read a library its vehicle owns, and Replay could not rebuild
  it from the EKF's own logged inputs.
- The ground position from every raw range sample: on a flight whose
  range finder alternated between the ground and 4.5 m, the height
  dropped 3.5 m at the switch.
- The ground position from the AGL KF alone, with no reset at the switch:
  across the 3.5 m step-up the EKF height moved 0.14 m (worst 0.33 m), as
  the AGL KF still lags a step in the ground by up to the 0.15 m the
  switch allows.
- Dropping the rule that keeps the ground below the vehicle: on a flow
  vehicle with no range finder the flow was then scaled by 5 cm.
