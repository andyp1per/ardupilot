# ArduPilot STM32H5 Port — Status

Working notes for the STM32H5 port. Target is the STM32H573 on the NucleoH573 development board; there is no shipping H5 product board today, so Nucleo-level support is the scope.

Companion documents in this directory:

- `H5_PORT_PLAN.md` — technical gap analysis, milestones and risk register.
- `H5_ST_DELIVERABLES.md` — the phased delivery structure and acceptance criteria.

These three files are working notes. They are not part of the upstream contribution and should be dropped before the port is proposed to ArduPilot.

## Where the work stands

Branch: `pr-h5-armv8-rebase`.

Code work has been paused since April 2026 while the delivery arrangement was put in place. The technical position is unchanged from `H5_PORT_PLAN.md` and should be read there rather than restated here. In summary:

- ChibiOS has been updated to the 21.11.5 kernel line, with ARMv8-M / Cortex-M33 startup, context switching and interrupt handling for the H573.
- Device configuration, clock tree, GPDMA, internal flash for the 8 KB sector layout, IWDG, ADC, PAL lines and the system timer are all in the tree.
- The bootloader builds and runs far enough to be verified on a debugger without crashing.
- **USB does not enumerate. This is the current blocker** and everything interactive is gated behind it.
- Beyond "it boots without crashing", essentially nothing is verified on silicon. Treat the tree as compiles-and-boots, not functional.

## How the remaining work is structured

Delivery is organised into three phases, each a self-contained milestone that is demonstrated and accepted on its own, followed by a consolidated summary. The technical content and acceptance criteria for each are in `H5_ST_DELIVERABLES.md`:

1. ChibiOS port and firmware foundation — a documented, repeatable build producing bootloader and firmware that boot on the Nucleo.
2. USB connectivity and a validated firmware kernel — USB CDC enumeration and firmware upload over USB using the standard ArduPilot tooling, through a working ArduPilot bootloader.
3. Sensor and peripheral bring-up — the DMA data path, SPI IMU and baro, internal-flash parameter storage, I2C compass, RC in and PWM out, FDCAN with a DroneCAN peripheral, TRNG, up to an arming-capable bench configuration with MAVLink over USB.

Each phase is budgeted at roughly a month, so the bench-capable milestone lands in early 2027. Phase 2 carries the most schedule risk: USB enumeration and GPDMA-on-silicon are the two items `H5_PORT_PLAN.md` flags as hardest, and a GPDMA problem cascades into most of phase 3.

## Scope boundaries

- The device runs in a non-secure configuration. TrustZone security partitioning is out of scope until a product board drives the need.
- The upstream commitment is a pull request against ArduPilot in a state fit for review. Carrying it through the project's review cycles to merge depends on the maintainers and on the release schedule, so it is treated as a separate activity rather than part of this work.
- The ICACHE/UID workaround (`d6f6d19668`) is still an open question: either root-cause it or document it as a permanent mitigation before calling the port done.

## Sponsorship and commercial terms

The remaining work is sponsored by STMicroelectronics under a services agreement. The agreement, its annexes and the governing terms are held by FOSS UAV Ltd outside this repository and are not reproduced here; nothing in these notes should be read as stating them.

The only points that bear on the technical record are:

- The software is and remains open source. ArduPilot work is GPL-3.0-or-later and ChibiOS work is under the licence of the component concerned. Copyright in contributions stays with the author, and the code is intended for public release in the upstream projects.
- The NucleoH573 hardware and the supporting STM32H5 documentation and errata are provided by ST.
