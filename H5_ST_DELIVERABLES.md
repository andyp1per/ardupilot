# ArduPilot STM32H5 Port — Deliverables

Phased delivery plan for ArduPilot support on the STM32H5, targeting the NucleoH573 (STM32H573) development board.

## Approach

The work is delivered in three phases of approximately one week each. Each phase is a self-contained milestone that delivers a usable, demonstrable capability and is accepted on its own merits. Acceptance of a phase releases the corresponding payment.

The phases follow the order in which a board comes to life: a firmware foundation that boots, host connectivity to load and talk to it, then the sensor and peripheral set that makes it fly. Each milestone leaves ST with working functionality that stands on its own, so value is delivered at every stage rather than only at the end.

## Phase 1 — Firmware foundation and boot

Objective: a buildable ArduPilot firmware and bootloader for the NucleoH573 that boots reliably on the board.

Delivered:
- ArduPilot brought up on the STM32H573 / ARMv8-M (Cortex-M33) architecture: kernel, startup, and core system configuration for the new core.
- Clock tree, internal flash, and core system peripherals configured for the H573.
- NucleoH573 board definition and a documented, repeatable build for both the bootloader and the main firmware.
- Firmware boots to a known-good state on the Nucleo, demonstrated on the board.

Acceptance:
- The documented build produces bootloader and firmware for the NucleoH573.
- The firmware boots to a known-good state on the board, demonstrated.

Value: ArduPilot running on the STM32H5. ST has a firmware foundation and a repeatable build that every subsequent capability is built on.

## Phase 2 — USB connectivity and firmware upload

Objective: full USB device support, with firmware upload over USB using the standard ArduPilot workflow.

Delivered:
- USB device enumerates on a host as an ArduPilot serial (CDC) device.
- The ArduPilot bootloader accepts firmware upload over USB via the standard ArduPilot tooling, and the uploaded firmware runs.
- USB serial link available to the main firmware for host communication.

Acceptance:
- The board enumerates as a USB device on a host.
- Firmware is uploaded over USB with the standard ArduPilot tooling and runs.

Value: the board is programmed and communicated with over a single USB cable using the standard ArduPilot workflow, with no debugger or special tooling. This is the everyday development and update path for the platform.

## Phase 3 — Sensor and peripheral bring-up

Objective: bring up the DMA-driven peripheral set and sensors, up to an arming-capable firmware on the bench.

Delivered, in priority order:
- High-throughput DMA data path established on the H5, underpinning the sensor and communications peripherals.
- SPI IMU and barometer.
- Internal-flash parameter storage.
- I2C compass.
- RC input and PWM output.
- FDCAN with a DroneCAN peripheral (for example, a GPS).
- Hardware random number generator wired into the firmware.
- Arming-capable firmware demonstrated on the bench with MAVLink over USB.

Acceptance:
- The DMA data path is demonstrated on the H5.
- Sensors and peripherals are brought up in the order above, each demonstrated as it lands, up to an arming-capable bench configuration.

Value: the H5 reaches a flight-ready bench state with sensors, storage, RC, CAN, and a full MAVLink link, completing the core platform.

## Summary

| Phase | Delivers | Value to ST |
|-------|----------|-------------|
| 1 | Firmware foundation and boot on the NucleoH573 | ArduPilot running on the STM32H5, with a repeatable build |
| 2 | USB connectivity and firmware upload | Program and talk to the board over one USB cable, standard workflow |
| 3 | DMA-driven sensors and peripherals | Flight-ready bench platform with the full sensor and comms set |

## Scope

- Target board is the NucleoH573 development board.
- The device runs in a non-secure configuration; TrustZone security partitioning is out of scope.
- Upstream contribution of the port to the ArduPilot project (rebasing, project CI integration, release packaging) is a separate activity and can be scoped on request.
