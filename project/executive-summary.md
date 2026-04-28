# ArduPilot RP2350B Port — Executive Summary

*Audience: project sponsor / non-engineering stakeholders. Updated as the work progresses.*

**Last updated:** 2026-04-28

---

## Timeline (5 weeks elapsed)

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

- **Week 5 (this week)** — *Architectural cleanup for upstream.* Refactored
  the RP2350 code out of the shared ArduPilot/ChibiOS codebase into a clean
  sibling library (`AP_HAL_Pico`), so the work is in a shape that can be
  reviewed and merged upstream rather than living forever in a personal
  fork. Both boards still build clean and the Laurel board still boots
  after the refactor. 15 commits, two days.

---

## Where we are vs the original plan

The implementation plan estimated **~4½ months (18 weeks)** to reach the
"sponsor milestone" of a flying flight controller. Five weeks in, we have
covered roughly **80% of that critical path**.

- ✅ **Board bring-up complete.** Both target boards boot, run firmware,
  talk to a ground station, and respond to commands.
- ✅ **Core flight-control plumbing in place.** Sensors read correctly,
  the navigation estimator runs (and runs on the second CPU core), motor
  outputs respond at the correct timing.
- ⚠️ **Closed-loop flight not yet demonstrated.** The board does
  everything a flight controller does individually; the end-to-end
  "lift off and hover" test has not happened yet.
- ✅ **Architectural debt managed.** The refactor this week means the
  code is structured the way upstream ArduPilot maintainers will expect —
  isolated to its own backend rather than mixed into the shared STM32
  code.

---

## What's left

- **First actual flight test on Laurel** — the gating event for the
  sponsor milestone.
- **High-performance motor protocol (DShot)** — currently only basic PWM
  is wired; DShot is the modern standard and is the next major driver to
  write. Estimated 1–2 weeks.
- **Performance optimization** — chip overclock and full dual-core
  flight-loop wiring to hit the target 400 Hz update rate.
  Estimated 1–2 weeks.
- **Two further refactor pieces** — clean separation of the UART driver
  and the motor-output driver into the new `AP_HAL_Pico` library;
  structural, not blocking flight, but nice to land before the upstream
  PR series.

---

## Posture

The port is **substantially ahead of the original 18-week schedule**.
Most of the hard, hardware-dependent debugging work that the plan called
out as risky (build system, low-level memory model, external flash
execution, software-emulated peripherals, second-CPU coordination) is
already done. The remaining work is well-scoped and incremental.
