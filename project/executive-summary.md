# ArduPilot RP2350B Port — Executive Summary

*Audience: project sponsor / non-engineering stakeholders. Updated as the work progresses.*

**Last updated:** 2026-10-31

---

## Update: 31 October 2026 (~31 weeks elapsed)

This update sits above the 21 October update, which is kept below it as it was.

### Headline

**Anyone can now download ArduPilot for Raspberry Pi's flight controller.** A public release repository went live on 25 October, and the first prebuilt release followed on 27 October, so flying the board no longer needs a build environment. Eleven release-test flights over 29-31 October found no fault in the port itself. The one weak spot they showed is the board's current sensor, whose reading ends up in the compass.

### Since the 21 October update

- **25-27 October: a release repository for users.** Until the port is accepted upstream, ArduPilot's own download server cannot build RP2350 boards, so the new repository ([fossuav/ardupilot-rp2350](https://github.com/fossuav/ardupilot-rp2350)) fills the gap. In it:
  - ArduPilot with the RP2350 port, plus 13 fixes found during testing. Every fix is also submitted to the ArduPilot project, so nothing in it is private to the fork.
  - Ready-to-flash firmware for the Raspberry Pi flight controller, and for a stock Pico 2, which runs the flight code against a simulated aircraft with nothing attached but USB.
  - Instructions for installing, building and recovering a board, and the known limitations. Installing takes two steps over USB, and a board can always be recovered through the chip's built-in USB loader.
  - Every change is built automatically, and rebuilding the release from scratch gives byte-for-byte the same files. Users report problems there rather than to the ArduPilot project; none have been reported yet.

- **23-28 October: upstream review.** The submission was brought up to date with the latest ArduPilot and answered several more rounds of automated review. It now stands at 55 commits, and the first 51 leave the firmware for existing flight controllers exactly as it was. The human reviewers' comments have all been answered apart from two threads. Two fixes came out of this round:
  - The bootloader now erases flash more carefully during an update. It could leave behind a page written by the chip's own USB loader, which made a check after an update fail on a Pico 2.
  - The serial-port mode used to control video transmitters could lose the transmitter's replies. This is fixed and bench-tested, but not yet flown.

- **29 October: the boards are not yet the final revision.** Raspberry Pi confirmed that the boards being flown are R3s modified towards R4, with one current-sensor part still the original. True R4 boards are due shortly. The same day, a GPS that would not lock was chased as a possible firmware fault, and turned out to be the receiver itself.

- **29-31 October: release-testing flights.** Eleven flights over three days, on the build planned for the next release, in stabilised flight, altitude hold, GPS position hold and acrobatic mode with flips. The port was clean in every one: the flight-control loop held its full rate of 1600 times a second on the second core, and no log data was lost. The pre-flight check problem from September has not come back.
  - On 30 October, motor current swamped the compass during hard acrobatic flying, and the navigation failsafe tripped four times. After the current sensor was rescaled and the compass recalibrated from a dedicated calibration flight, acrobatic flight was clean.
  - On 31 October nothing tripped, and the navigation system's heading stayed within 6 degrees of true. The compass's correction had stopped matching the aircraft overnight, for a reason on the airframe that is still being traced, and the navigation system rode through it.

- **30 October: the current sensor measured.** The board's current reading depends heavily on the motor controller (ESC) it is paired with. With the ESC in the test aircraft it reads six to eight times lower than the board's default setting expects. Its zero point also drifts with board temperature, by more than half an amp per degree with this ESC. The flight controller uses that reading to cancel the motors' magnetic interference at the compass, so the error ends up in the compass. The findings went to Raspberry Pi's designers in a separate note, and the release's instructions tell users how to work around them.

### Where we are against the original plan

- ✅ **Users can get it.** Prebuilt firmware is published with instructions, so the board can be flown without building anything.
- ✅ **The port holds up in release testing.** Eleven flights, including acrobatics, found no fault in the port. The September pre-flight check problem is confirmed fixed in flight.
- 🔵 **Upstream review is in progress.** This is unchanged. The ChibiOS change has been waiting for review since September and is now the main gate.
- ⚠️ **Some safety modes have not yet been flown.** Return-to-home, autonomous missions and the GPS-loss failsafe still need deliberate testing, and automatic tuning has not yet been run.
- ⚠️ **The current sensor limits compass accuracy.** The release documents a compass setting that does not use the sensor. R4 boards should show how much of the problem their changed parts remove.
- ⚠️ **There are still open hardware questions for Raspberry Pi.** They cover running the chip above its datasheet speed, the current sensor (now measured), and the 9 V power switch.

### What's left

- **Cut the second release** from the build flown on 29-31 October, with the bootloader fix.
- **Flight-test the remaining modes and failsafes,** and run automatic tuning.
- **Test on R4 boards when they arrive:** measure the current sensor again, then recalibrate the compass.
- **Get the upstream submission accepted.** The ChibiOS change comes first.
- **Hardware answers from Raspberry Pi** on the clock speed, current sensing and power switching. For current sensing, the question is what ESC output the input is designed for, and whether R4 suits ESCs like the one in the test aircraft.
- **Later:** other vehicle types (fixed-wing and others) and CAN-bus peripherals.

### Posture

At about 31 weeks, against the plan's estimate of about 32 weeks to a production-quality port, the work has moved from engineering to release and testing. The firmware is now in users' hands through the release repository, and release testing has found no fault in the port. What remains is flight-testing the safety modes, a current-sensor question that the R4 boards should help answer, and upstream review, whose pace is not in our hands.

---

## Update: 21 October 2026 (~30 weeks elapsed)

This update sits above the June summary, which is kept below it as it was.

### Headline

**ArduPilot flies on Raspberry Pi's own flight controller.** The first flights came in early August, about 20 weeks in, close to the plan's 18.5-week estimate for this milestone. Since then the board has flown every basic flying mode, including GPS position hold and full-rate acrobatic flight upside down. The port has also been submitted to the ArduPilot project, where it is under review.

### Since the June update

- **Early July: running cooler.** The chip had been pushed to 375 MHz to keep up. Moving the time-critical flight code into fast on-chip memory let it come back down to 225 MHz at a lower voltage, with no loss of performance and more safety margin.

- **Early August: first flight, on Raspberry Pi's board.** Flight testing moved to the Raspberry Pi UAV Flight Controller, the final R2 Rev C hardware. The first flights came by 3 August, and within days the board had:
  - switched to the modern digital motor protocol, which reports each motor's speed back to the flight controller and uses it to filter out motor vibration;
  - held position on GPS;
  - flown aggressive acrobatics, including inverted flight at full roll rates.

  The flight-control timing held throughout.

- **August to September: features and hardening.** In this period:
  - Logging to the SD card became about 2.3 times faster. By late September, flights were logging without losing any data in any mode flown.
  - The on-screen display (OSD) now works with both digital and analog video.
  - LED strips are supported, and the video transmitter can be controlled from the flight controller (bench-tested so far).
  - The newer motion sensor shown on the production schematic got its driver. On 9 September it flew on a second, different aircraft, and the port needed no changes for either.

- **Mid-September: field testing with analog video.** This found the second processor core nearly full once armed, glitches in the on-screen display, and a pre-flight safety check failing. The core's load was cut from 82% to 57%, and the check was fixed on the bench but has not yet been confirmed in flight.

- **May to October: upstream submission.** The port went to the ArduPilot project in May and has been through several review rounds. Several hundred development commits were reorganised into 53 reviewable ones, and 45 of them leave the firmware for existing flight controllers exactly as it was. Twelve fixes found along the way that help ArduPilot generally were submitted separately; two have been accepted so far.

- **Early October: a setback, caught quickly.** A test build on 2 October rebooted repeatedly and kept losing GPS. Both faults had been introduced by review changes. Both were found and fixed the same day.

### Where we are against the original plan

- ✅ **Sponsor milestone reached: it flies.** It flies stabilised, with altitude hold, with GPS position hold and in acrobatic mode.
- ✅ **Most of the "extra features" stage is done.** That covers the digital motor protocol, LEDs, on-screen display and the dual-core flight architecture.
- ✅ **Less overclocking.** The chip runs at 225 MHz instead of 375 MHz.
- 🔵 **Upstream review is in progress.** This is now the gating item for ArduPilot users getting the board. The project's maintainers have asked for changes. A companion change to ChibiOS, the real-time operating system ArduPilot runs on, has to be accepted first.
- ⚠️ **Some safety modes have not yet been flown.** Return-to-home, autonomous missions and the GPS-loss failsafe all need deliberate testing before anyone relies on them.
- ✅ **Flight logging fixed.** A 9 September flight lost log data in position hold because the first core was too busy. After the September work, flights lose no log data in any mode flown so far, position hold included.
- ⚠️ **There are open hardware questions for Raspberry Pi.** They cover running the chip above its datasheet speed, a current sensor whose reading drifts with temperature, and a 9 V power switch that does not appear to switch.

### What's left

- **Get the upstream submission accepted.** This means the ArduPilot and ChibiOS changes, plus answering the reviewers' outstanding requests.
- **Flight-test the remaining modes and failsafes,** and finish automatic tuning.
- **Hardware answers from Raspberry Pi** on the clock speed, current sensing and power switching. These feed into the next board revision.
- **Later:** other vehicle types (fixed-wing and others) and CAN-bus peripherals.

### Posture

The flight-controller work is largely done, roughly on the plan's schedule: the board flies, the timing holds, and the hard platform risks the plan identified have been retired. The plan estimated about 32 weeks to a production-quality port, and at about 30 weeks the remaining work is mostly testing and upstream review rather than new engineering. The main unknown is now how long upstream review takes, which is not fully in our hands. The main hardware question is the clock speed above the datasheet rating, which only Raspberry Pi can answer.

---

## Timeline (~12 weeks elapsed)

- **Week 1 (mid-March)** — *First boot.* Brought the Raspberry Pi RP2350B chip
  up from a cold start: build system working, bootloader operational, board
  enumerates over USB, flash storage and basic peripherals (motor outputs,
  RC input, I²C, SPI) wired up. Confirmed on real hardware at the end of the
  week.

- **Week 2 (late March)** — *Telemetry online.* MAVLink telemetry over USB
  confirmed streaming reliably. Bootloader-to-firmware handoff verified
  end-to-end (the standard ArduPilot upload tooling works against the new
  board). Attempted dual-core support, encountered deadlocks, and switched
  to a simpler model — a pragmatic backtrack, not lost time. All four
  software-emulated serial ports verified working at the wire level.

- **Week 3 (early April)** — *Second board, real flight-controller features.*
  Added a second target — **Laurel**, a production-grade RP2350 flight
  controller — alongside the Pico2 dev board. Got MAVLink file transfer
  working (ground station can now read/write files on the board). Got the
  EKF (the navigation/attitude estimator) running on the second CPU core —
  proving the dual-core architecture pays off. Motor PWM outputs and
  battery-monitoring ADC inputs verified to correct values.

- **Week 4 (mid-April)** — *Integration polish.* Bug fixes, stabilization,
  and feature-gap documentation across both boards. Laurel boot path
  verified clean.

- **Week 5 (late April)** — *Architectural cleanup for upstream.* Refactored
  the RP2350 code out of the shared ArduPilot/ChibiOS codebase into a clean
  sibling library (`AP_HAL_Pico`), so the work is in a shape that can be
  reviewed and merged upstream rather than living forever in a personal
  fork. Both boards still build clean and the Laurel board still boots
  after the refactor. 15 commits, two days.

- **Weeks 6-12 (May to mid-June)** — *Dual-core in earnest, and going fast.*
  This stretch turned the "second core works" proof point into a real
  dual-core flight architecture, then started squeezing performance:

  - **Full dual-core (SMP) switched on.** The high-rate attitude/PID
    control loop now runs on the second CPU core, fed by sensor reads on
    the first core. This is the arrangement a flying board needs, not just
    the earlier "estimator on core 2" demo.
  - **Chip runs much faster.** Execution from external flash was pushed in
    stages (75 MHz, then 93.75 MHz, then 125 MHz on real hardware) right up
    to the chip's practical ceiling. More clock means more headroom for the
    flight loops.
  - **We can now measure what the board is doing.** Added in-flight
    reporting of per-core loop timing, flash-cache efficiency, and the
    sensor-to-control latency that ultimately limits how tight the control
    can be. Plus a standalone profiler that watches the running chip over
    the debug port without disturbing it, to pinpoint the hottest code.
  - **A plan, with evidence, for the last performance push.** The code is
    too big to fit entirely in the chip's fast cache, so the hottest pieces
    get hand-placed in fast on-chip memory. The measurement work above
    decides exactly which pieces, and a documented decision settles how the
    final automated optimization pass will be done.

---

## Where we are vs the original plan

The implementation plan estimated **~4½ months (18 weeks)** to reach the
"sponsor milestone" of a flying flight controller. About twelve weeks in,
we have covered the large majority of that critical path.

- ✅ **Board bring-up complete.** Both target boards boot, run firmware,
  talk to a ground station, and respond to commands.
- ✅ **Core flight-control plumbing in place.** Sensors read correctly,
  the navigation estimator runs, motor outputs respond at the correct
  timing.
- ✅ **Dual-core flight architecture in place.** The high-rate control
  loop runs on the second CPU core — the structure a flying board needs,
  not just an estimator demo.
- 🔵 **Performance tuning underway, measurement-driven.** The chip is
  overclocked to its practical limit, the firmware now reports its own
  timing and cache behaviour, and the remaining optimization is gated on
  real hardware numbers rather than guesswork. Those hardware baseline
  numbers are still being collected.
- ⚠️ **Closed-loop flight not yet demonstrated.** The board does
  everything a flight controller does individually; the end-to-end
  "lift off and hover" test has not happened yet.
- ✅ **Architectural debt managed.** The earlier refactor leaves the code
  structured the way upstream ArduPilot maintainers will expect —
  isolated to its own backend rather than mixed into the shared STM32
  code.

---

## What's left

- **First actual flight test on Laurel** — the gating event for the
  sponsor milestone.
- **Collect hardware performance numbers and finish the tuning pass** —
  the instrumentation and profiler are built; the remaining step is to run
  them on the bench, relocate the measured hot code into fast memory, and
  decide whether the final automated optimization pass is worth it. The
  decision criteria for that are already written down. Estimated 1-2 weeks.
- **High-performance motor protocol (DShot)** — currently only basic PWM
  is wired; DShot is the modern standard and is the next major driver to
  write. Estimated 1-2 weeks.
- **Two further refactor pieces** — clean separation of the UART driver
  and the motor-output driver into the new `AP_HAL_Pico` library;
  structural, not blocking flight, but nice to land before the upstream
  PR series.

---

## Posture

The port is **ahead of the original 18-week schedule**. Most of the hard,
hardware-dependent debugging work that the plan called out as risky (build
system, low-level memory model, external flash execution, software-emulated
peripherals, second-CPU coordination) is already done, and the dual-core
control architecture the plan treated as the central risk is now running.
The remaining work is well-scoped and incremental, with the one genuine
unknown being how much the performance-tuning pass buys once measured on
real hardware. The headline milestone — a flight — is still ahead of us.
