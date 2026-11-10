# ArduPilot on RP2350 - Follow-on Plan (draft)

*Draft advice for Eben, 2026-11-10. The release has moved to after Christmas. The goal is to hold the whole market, which means H7 performance at a sub-F4 price.*

## Recommendation

In priority order:

1. **Keep the upstream merge moving, and plan it as a standing job.** Merging is what puts RP2350 boards on ArduPilot's firmware server and in every ground station's firmware list, and it ends the question of whether ArduPilot supports the chip. But it will not finish in a set number of weeks: it is a cycle of review, changes, rebase and retest that has run for six months already. It needs a share of every week through both halves of this plan, with no promised date. The release does not wait for it.
2. **Validate Plane.** Copter alone leaves out fixed-wing, which is a large part of the low-cost flight controller market. It is also where ArduPilot is strongest against the alternatives: autonomous missions, automatic takeoff and landing, and QuadPlane. Plane builds for these boards today but has never run on one. Validating it is mostly bench and flight time, not new code, so it can be in the release.
3. **Make the Copter release solid.** Fly the modes and failsafes that have not flown yet, and test on the R4 boards.
4. **Measure before building.** Several of the ideas on the list (Lua in flash, gcc17, -O2, SRAM placement, more UARTs, CAN) turn on numbers nobody has taken yet. Take them before Christmas, so the work after Christmas goes where the numbers point.
5. **Then close the H7 gap:** more UARTs, CAN, multiple IMUs, the gyro FFT, and Lua or Rust scripting. Several of these need a board revision, so the hardware wish list should reach Raspberry Pi early.

## The merge: a long haul

### Where it stands (10 November)

- **The PR:** ArduPilot#32995 has been open since 6 May. It is 71 commits, touching 158 files, adding about 24,000 lines and deleting under 300.
- **Its history so far:** 76 force-pushes, and more than 30 rounds of automated review.
- **Change requests:** three maintainers have requested changes (tridge, Peter Barker and tpwrules). Nearly all their comments have been answered, but the change requests still stand.
- **The ChibiOS port:** ArduPilot/ChibiOS#113, which the PR depends on, has been open since 11 September with no review at all.
- **Right now:** the PR conflicts with master, so CI has not run on it since 7 November.

### What one cycle involves

ArduPilot master moves every week, and the port touches shared HAL code, so each rebase brings conflicts and API changes. The 10 November review found three, from master's changes to the UART driver and the board name. One cycle:

1. Rebase onto master and fix what broke.
2. Answer the new review findings.
3. Re-verify:
   - Every commit builds.
   - The commits meant to leave existing boards alone still produce byte-identical firmware on a reference STM32 board.
   - All three RP2350 boards build with no warnings.
   - Any change to behaviour is flown before it is folded into commits that have already flown.
4. Push, and wait for the next round of review.

Each cycle repeats the whole verification, so it costs no less as the PR matures. The release branch also carries 13 follow-up PRs, each with its own review on the same pattern.

### The gates that are not ours

- **The ChibiOS port** has to be reviewed and merged into ArduPilot's ChibiOS. Then the PR is re-pinned to it and the bootloaders rebuilt.
- **tridge's condition:** Renode simulation of the RP2350 at "full fidelity, including performance". Andy owns this, finishing the demonstrator Buzz put together.
- **The three change requests** have to be cleared.
- **No written list:** no maintainer has said what merging requires. At the developer conference tridge hedged when Buzz asked.

### What that means for the plan

- Give the merge a fixed share of every week, before and after Christmas, rather than treating it as a task that finishes.
- Promise no merge date. The release repository exists so that users are not waiting on it.
- Ask the maintainers for a written list of what merging requires, and work to that list, so the review rounds converge rather than continue. The Renode scope is the first item on it.
- Take stalled items, ChibiOS#113 first, to the ArduPilot developer call.

### Before Christmas, for the merge

- Rebase #32995 and apply the 10 November review's three fixes, so CI runs again.
- Find a reviewer for ChibiOS#113.
- Agree the Renode scope with tridge in writing: what it must simulate, including performance, and which test has to pass. Then start finishing the demonstrator.
- Ask for the written merge list.
- Follow-up PRs: get the SBUS fix (#33057) merged, and take the current version of #34474 onto the release branch.

### After Christmas, for the merge

- Keep the cycle going until it merges.
- Renode in ArduPilot's CI at the agreed fidelity, running ArduPilot's Copter and Plane tests on a simulated RP2350. This also gives Raspberry Pi a hardware-free way to check every ArduPilot change.
- Once it merges, RP2350 boards build on ArduPilot's firmware server, ground stations offer them for download, and they get a board page on ardupilot.org. The support commitment (the retainer) starts to matter from then on.

## Before Christmas (to about 19 December)

The merge work above runs alongside everything here.

### Must: Plane and the release

1. **Validate Plane on RPI_UAVFC.** Each step below is cheaper than the next, and catches different faults:
   - **Build:** Plane for all three boards with no warnings. The build check that keeps hot code in SRAM runs for Copter only today; extend it to Plane.
   - **Simulate on the chip:** run Plane on a Pico 2 against the simulator's fixed-wing model. The code already picks the Plane model for a Plane build, but it has not been tried. Fly a mission with an automatic takeoff and landing, and trip the RC and GPS failsafes. Until Renode is ready, this is the closest thing to ArduPilot's automated tests on the real chip.
   - **Profile:** the SRAM placement was tuned on Copter. Plane has no equivalent of Copter's second-core rate thread, so its whole control loop runs on core 0, at 50 Hz by default against Copter's 400. That should make it light, but measure it rather than assume it.
   - **Bench:** servos at 50 Hz, an airspeed sensor on the GPS connector's I2C bus, the failsafes, CPU load and free RAM.
   - **Fly:** a flying wing or a four-channel plane. First MANUAL, FBWA and CRUISE, then AUTOTUNE, then AUTO with an automatic takeoff and landing, RTL, and the RC and GPS failsafes. Check the logs as for Copter.
   - **Ship:** Plane binaries go in the release once that passes.

   Outputs limit what flies. RPI_UAVFC has four servo/motor outputs, in two pairs that share an update rate (1-2 and 3-4), plus an LED output that the board README says not to use for servos. That is enough for a flying wing (two elevons and a throttle) or a basic four-channel plane. It is not enough for separate aileron servos with flaps, or for a QuadPlane, which needs eight or more.
2. **Confirm the flash part at 100 MHz** (James Adams). The 200 MHz build depends on it, and it has flown three times.
3. **Fly what has not flown on Copter:**
   - The GPS failsafe.
   - RTL's landing, re-flown with corrected settings.
   - An AutoTune run to completion.
   - The analog OSD and the half-duplex video-transmitter control (SmartAudio), both on a quad with analog video.
4. **R4 boards:** re-measure the current sensor and recalibrate the compass. Then correct the documentation, which still describes the R3 boards.
5. **Installing from a ground station:** test firmware uploads from Mission Planner and QGroundControl. Also try a single file that installs the bootloader and firmware together.
6. **Cut the alpha2 release** from the flown 200 MHz build.

### Should: measurements that decide the post-Christmas work

7. **SRAM/flash placement:** decide which functions run from SRAM using a profile taken in flight, for Copter and Plane, and make that a script anyone can re-run. Then a refactor upstream means re-running the script rather than hand-editing a list, which answers Peter's concern about refactoring.
8. **-O2 vs -Os A/B:** the build is -Os, with 13 hot files (the EKF among them) at -O2, and the trade-off has never been measured on the board. Measure CPU load and flash size.
9. **gcc17 A/B:** measure the same two, and check it builds cleanly. ArduPilot's firmware server builds with gcc 10.2.1 and would keep doing so after a merge, so gcc17 would only change the binaries we publish ourselves.
10. **Free more RAM** with tridge's stack-measurement tool, as agreed at the developer conference. The OSD thread's stack is a known item: it has only 104 bytes to spare.
11. **Lua baseline:** Lua is compiled in but off by default. What limits it is RAM, not flash. ArduPilot already counts the RP2350 in its 500 KB memory class, whose default Lua heap is 100 KB. That would take most of the 150 KB free when armed, a figure measured with the full log buffer and the analog OSD on. Fly RPI_UAVFC with scripting on and a representative set of scripts. Measure free RAM and core 0 load while armed. That says whether Lua can be on by default, at what heap size, and how much the flash-caching idea needs to save.
12. **Connector audit for outputs, UARTs and CAN.** This is a desk exercise against the schematic. The serial-port engines (PIO blocks) are already allocated:
    - PIO0 runs the two software UARTs.
    - PIO1 runs one of: the analog OSD, the LED strip, or two more UARTs.
    - PIO2 runs DShot.

    So a third UART, or CAN, competes with the analog OSD unless it fits elsewhere. Servo outputs need no PIO, since the chip's PWM hardware can drive any pin, but they compete with UARTs and CAN for the same few spare pads. Candidates are the external I2C pair on the GPS connector (a natural fit for CAN, since a DroneCAN GPS replaces the I2C compass), GPIO41, the spare analog input and the LED pad. The output is a list of what the current board can do and what needs a new revision.
13. **Hardware wish list for the next board revision,** drawn from testing:
    - More outputs: eight or more for QuadPlane.
    - A second IMU.
    - A CAN transceiver footprint.
    - More serial pads.
    - The current-sensor input.
14. **Support:**
    - Agree the retainer.
    - Start Matthew Selby's onboarding: building, flashing, reading a log, triaging an issue from the release repository.
    - Decide who answers RP2350 questions on the Raspberry Pi forums.

## After Christmas

The merge cycle continues alongside everything here.

### Release

1. **Public release** of Copter and Plane, from the flight-tested build. It does not wait for the merge.

### Plane beyond the basics

2. **QuadPlane,** on a board with eight or more outputs. Its multicopter half is Copter's attitude control at a much higher loop rate than plain Plane, so measure core 0 load before flying it.
3. **Rover,** which builds but has never run. The same ladder as Plane, at lower priority.

### Closing the H7 gap

4. **More UARTs** through PIO, with clear rules for which feature gets which PIO block on each board.
5. **CAN:** Eben's software CAN on PIO plus an external transceiver, starting with a DroneCAN GPS and compass, and a DroneCAN airspeed sensor for Plane. Check that it coexists with DShot, the OSD and the UARTs, and measure its CPU cost.
6. **Multiple IMUs:** needs a board revision. Measure what a second IMU costs core 1 and what a second EKF lane costs core 0 and in RAM.
7. **Gyro FFT and DSP:** off on RP2350 today, standard on H7. Measure whether it fits.
8. **A fixed benchmark against an H7 board:** loop rates, filter count, logging rate and CPU load. That tracks "closing the gap" as numbers rather than claims.

### Scripting

9. **Lua with bytecode and constant strings in flash,** to cut the RAM it needs. Measure the cost on core 0, since reading from flash is slower than reading from SRAM.
10. **Rust modules:** follow tridge's work and make RP2350 an early target. Compiled modules avoid the interpreter's heap altogether.

### Reach (beyond Eben's list)

11. **AP_Periph on RP2350:** cheap DroneCAN peripherals (GPS, airspeed, rangefinder nodes) are a market of their own. They follow from CAN.
12. **A kit for third-party board makers:** a porting guide, a reference hardware definition, the board ID process, and a validation checklist (CPU load, a flight-test list per vehicle). This keeps third-party boards from becoming ArduPilot's support burden, which answers the "too many boards" concern.
13. **Light-show drones:** ask a show operator what they need (RTK GPS, LEDs, show sequencing) before building anything for that segment.
14. **The next microcontroller:**
    - Get early silicon.
    - Plan its HAL changes: Helium for the filters and FFT, double-precision floating point for the EKF (as H7 boards run it), and the doubled RAM for Lua and logging.
    - Keep the port structured so that a second chip is a new MCU definition, not a second port.

## Owners and open points

- **Andy:** the PR, Renode, release, flight testing, measurements and retainer support.
- **Raspberry Pi:** James Adams (the flash part), Matthew Selby (onboarding, then owning the relationship), chip detail for Renode where the model needs it, and the next board revision.
- **ArduPilot:** tridge (ChibiOS#113 review, the Renode scope and the merge list, the stack tool); Peter Barker and tpwrules (their change requests).
- **Open:**
  - The release date.
  - The retainer terms.
  - A Plane airframe and a pilot for its flight tests.
  - The "500 KB of Lua" figure: which quad it came from, and whether it is the script files, the heap setting or the heap in use (Andy).
  - Which items Raspberry Pi wants to fund beyond the release.
