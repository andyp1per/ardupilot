# ArduPilot STM32H5 Port — Deliverables

Staged deliverables for the STM32H5 port engagement, targeting the NucleoH573 (STM32H573) development board. Nucleo-level support: there is no shipping H5 product board today, so the objective is a documented, community-grade port on the Nucleo.

## How this engagement is structured

The work is split into three deliverables of roughly one week of effort each. Each deliverable is a milestone: it is self-contained, independently useful, and its acceptance is what releases the corresponding payment tranche.

The three deliverables are ordered by dependency and risk so that the engagement can conclude cleanly at any milestone boundary. Each one leaves ST with a coherent, usable result rather than half-finished work, and no deliverable depends on a later one having been started.

The full remaining scope of a complete Nucleo port is larger than three weeks of effort. This is deliberate, not a mismatch: Deliverable 3 in particular carries the two hardest items in the port (DMA-on-silicon and its downstream peripherals), and its full extent cannot be fixed in advance. The staging exists precisely so that this uncertainty is bounded. Deliverables 1 and 2 retire the near-term risk and produce standalone value; Deliverable 3 commits to a firm minimum (DMA proven) and then covers as much peripheral bring-up as the week allows, in a fixed priority order. If the later work proves harder than hoped, ST has already banked two complete, useful milestones.

## Deliverable 1 — Verified baseline and USB de-risking

Objective: package the port as it stands into a documented, reproducible baseline, and spend the engineering week de-risking the USB and bootloader path so that Deliverable 2 can be committed to (or re-scoped) with an informed view of the risk.

Scope:
- Reproducible build of the NucleoH573 bootloader and main firmware from a named commit/tag, with documented build steps.
- Baseline documentation: current state of the clock tree, DMA driver, flash driver, and hwdef, plus a verified-vs-unverified feature matrix so the true maturity of each subsystem is explicit.
- Demonstration that the firmware boots on the Nucleo to a known state without faulting, evidenced on a debugger.
- A USB/bootloader investigation report: root-cause analysis of the USB enumeration failure across the likely causes (clock configuration, PHY power, USB descriptors, and the ChibiOS USB device driver path on H5), with a go/no-go recommendation for Deliverable 2 and, where the fix is identified, the path to it.

Acceptance criteria (payment trigger):
- The named commit builds cleanly for the NucleoH573 (bootloader and firmware) by following the documented steps.
- The firmware boots to a known state on the Nucleo, demonstrated on a debugger.
- Baseline documentation and the feature matrix are delivered.
- The USB de-risking report is delivered with a go/no-go recommendation for Deliverable 2.

Value if the engagement stops here: ST holds a clean, documented, reproducible H5 baseline that compiles and boots, together with a technical assessment of the USB path that any engineer can pick up and continue from.

Risk: low. This is primarily consolidation and investigation of work that already exists.

## Deliverable 2 — USB device and ArduPilot bootloader

Objective: a working USB device on the Nucleo, with the ArduPilot bootloader able to accept a firmware upload over USB. In practice this is the milestone that removes the debugger from the development loop and gives ST the standard ArduPilot flash-over-USB workflow.

Scope:
- USB device enumerates on a host operating system (Linux and/or Windows) as an ArduPilot CDC serial device.
- The ArduPilot bootloader enumerates over USB and accepts a firmware image via the standard ArduPilot upload tooling, and the uploaded firmware runs.
- Target, if reached within the week: the main firmware presents its USB console and emits a MAVLink heartbeat over USB.

Acceptance criteria (payment trigger):
- The board enumerates as a USB device on a host, evidenced by host-side logs.
- A firmware binary is successfully uploaded to the board over USB using the standard ArduPilot toolchain, and the uploaded firmware runs.

Value if the engagement stops here: ST can build and flash the H5 over USB with the standard ArduPilot workflow, without a debugger. Even before any sensor or DMA work, this is the normal bring-up baseline for any new ArduPilot board and makes the Nucleo a usable development platform.

Not in scope: peripheral drivers, DMA validation on silicon, sensors, or arming.

Risk: high. USB is one of the two hardest items in the port, and the enumeration failure may prove to be an upstream ChibiOS issue on the H5 USB device stack rather than a local misconfiguration. Deliverable 1's de-risking report is what determines whether this deliverable is committed as written or re-scoped before work starts. Fallback if enumeration proves to be an upstream blocker: Deliverable 2 is re-scoped to a UART-based bootloader upload path plus upstream engagement, delivered alongside the enumeration findings so the path forward is documented rather than lost.

## Deliverable 3 — DMA validation and peripheral bring-up

Objective: retire the GPDMA risk on real silicon, then bring up peripherals as far as the week allows, in a fixed priority order, working toward an arming-capable MAVLink bench test.

This is the largest and highest-variance deliverable. The full peripheral bring-up is more than one week of work, and the cost of each peripheral depends on the shape the DMA driver settles into once it is exercised on silicon. It is therefore committed as a firm minimum plus a prioritized best-effort list, rather than as a fixed full scope.

Committed minimum (payment trigger): GPDMA verified end-to-end on silicon — one UART transmit and receive under sustained traffic, plus an SPI loopback — demonstrating that the DMA driver functions on real hardware. This is the pivotal risk for the whole port; everything downstream inherits its outcome, which is why it is the committed floor for this deliverable.

Target, addressed in priority order as the remaining week allows:
1. SPI IMU and baro via GPDMA.
2. Internal-flash parameter storage (removing the empty-storage placeholder).
3. I2C and one compass.
4. RCIN and RCOut (timer capture and PWM).
5. FDCAN with a DroneCAN GPS attach test.
6. TRNG wired into the HAL random source.
7. Arming-capable bench test with MAVLink over USB.

Acceptance criteria (payment trigger): the committed minimum (GPDMA verified end-to-end on silicon) is demonstrated. Any target items completed within the week are delivered on top of that minimum, in the order above, and documented as verified.

Value if the engagement stops here: DMA — the single risk that governs the difficulty of the entire remainder of the port — is either proven working or fully characterized, plus whatever peripheral set was reached. Combined with Deliverables 1 and 2, ST has a documented, USB-flashable H5 with proven DMA and a working subset of sensors.

Risk: high and highest-variance. GPDMA is code-complete but unverified on silicon. If it needs driver rework, or forces an upstream ChibiOS change, the target list contracts accordingly. The committed minimum is deliberately set at DMA retirement rather than at any particular sensor, because the sensors cannot be estimated with confidence until DMA behaviour on silicon is known.

## Summary

| # | Deliverable | Committed acceptance (payment trigger) | Standalone value if work stops here |
|---|-------------|----------------------------------------|-------------------------------------|
| 1 | Verified baseline + USB de-risking | Reproducible build boots on Nucleo; baseline docs + feature matrix; USB de-risking report with go/no-go | Documented, reproducible, boot-verified H5 baseline plus a USB path assessment |
| 2 | USB device + ArduPilot bootloader | Board enumerates over USB; firmware uploads over USB via standard tooling and runs | Standard flash-over-USB dev workflow, no debugger required |
| 3 | DMA validation + peripheral bring-up | GPDMA verified end-to-end on silicon; target peripherals delivered in priority order as time allows | DMA risk retired/characterized plus a working sensor subset |

## Scope boundaries

- Target is the NucleoH573 only. No product board, and no product-board-specific work.
- The MCU runs fully non-secure; TrustZone/SAU configuration beyond that is out of scope until a product board requires it.
- Release hygiene for an upstream ArduPilot pull request (rebase, submodule squash, CI matrix, bootloader binary commit) is not part of these three deliverables and can be scoped separately if ST wants the work upstreamed.

## Risk summary

| Risk | Affects | Handling |
|------|---------|----------|
| USB enumeration is an upstream ChibiOS issue on H5 | Deliverable 2 | De-risked in Deliverable 1 before commitment; UART-upload fallback keeps the milestone deliverable |
| GPDMA needs rework once exercised on silicon | Deliverable 3 target list | Committed floor set at DMA retirement; target peripherals are best-effort and prioritized |
| Peripheral bring-up exceeds one week | Deliverable 3 target list | Fixed priority order so the most valuable peripherals land first; engagement can close at the milestone boundary with value banked |
