# Implementation Plan

## Strategy: Incremental Mergeable PRs

Each step produces a PR that can be reviewed and merged independently. This
avoids an unwieldy branch and shows concrete progress to the community and
sponsor. The steps build on each other but each one compiles and is testable
in isolation.

**Target hardware:** Laurel board (Betaflight: HELLBENDER_0001, manufacturer
RASP). This is an RP2350B flight controller with ICM-42688P IMU, DPS310 baro,
SD card, 4 motor outputs, PIO UARTs, CRSF RC input, and WS2812 LEDs. Betaflight
already flies on this board.

---

## Step 1: ChibiOS Update

**Goal:** Update the ChibiOS submodule to include RP2350 support.

**PR scope:** Submodule pointer update only. No ArduPilot code changes.

### Tasks

1. Update `modules/ChibiOS` to a version that includes `os/hal/ports/RP/RP2350/`
   and the SMP dual-core port
2. Verify all existing ChibiOS board builds still compile (CI will test this)
3. If ChibiOS trunk is too far ahead for a clean merge, work with ChibiOS
   maintainers to get RP2350 support into a stable branch that ArduPilot can
   track

### Verification
```bash
# Existing boards still build
./waf configure --board CubeOrange && ./waf copter
./waf configure --board Pixhawk6X && ./waf copter
# RP2350 files are present
ls modules/ChibiOS/os/hal/ports/RP/RP2350/
```

### Deliverable
- ChibiOS submodule updated, all CI tests pass
- RP2350 LLD drivers available in the tree

### Notes
- This is the lowest-risk PR — it changes nothing about ArduPilot itself
- May require coordination with the ChibiOS submodule maintainer (bugbot)
- ChibiOS RP2350 demo confirmed building with ArduPilot's GCC 10.2.1

---

## Step 2: Bootloader

**Goal:** ArduPilot bootloader runs on Pico 2, accepts firmware via USB.
This is the standard first milestone for any new board bringup — nothing else
can be tested until firmware can be flashed.

**PR scope:** New files (bootloader hwdef, board config). Board ID registration
in `board_types.txt`.

### Tasks

1. **Register board type**
   - Add `HAL_BOARD_PICO = 14` to `libraries/AP_HAL/AP_HAL_Boards.h`
   - Register board ID in `Tools/AP_Bootloader/board_types.txt`

2. **Bootloader hwdef**
   - Create `libraries/AP_HAL_ChibiOS/hwdef/Pico2-bl/hwdef-bl.dat`
     (bootloader can use ChibiOS hwdef since it's a minimal build)
   - Or create a minimal standalone bootloader using ChibiOS + USB
   - USB CDC for MAVLink bootloader protocol
   - LED blink pattern to indicate bootloader mode

3. **Build and flash**
   - Build: `./waf configure --board Pico2-bl && ./waf bootloader`
   - Initial flash via RP2350's built-in BOOTSEL mode (hold button, drag UF2)
   - Subsequent flashes via MAVLink bootloader protocol or UF2

4. **Bootloader binary**
   - Commit bootloader binary to `Tools/bootloaders/`
   - This is standard ArduPilot practice for all boards

### Verification
```bash
./waf configure --board Pico2-bl
./waf bootloader
# Hold BOOTSEL, drag UF2 to RPI-RP2 drive
# LED blinks in bootloader pattern
# MAVProxy: upload firmware via USB
```

### Deliverable
- Bootloader runs on Pico 2
- Firmware can be uploaded via USB (MAVLink or UF2)
- Board ID registered

### Notes
- RP2350 has a ROM bootloader (BOOTSEL) that can always be used as fallback,
  so a bricked bootloader is recoverable by holding the BOOTSEL button
- The bootloader may initially be very simple — just enough to accept firmware
  via USB and write to flash. It can be enhanced later.
- Consider whether the ArduPilot bootloader should produce a UF2 file that
  chains to the application firmware, or use the standard MAVLink protocol

---

## Step 3: AP_HAL_Pico Skeleton + Pico 2 Demo Board

**Goal:** `./waf configure --board pico2 && ./waf AP_Periph` compiles (even if
firmware does nothing useful). Board boots via bootloader, LED blinks, USB
console prints.

**PR scope:** New files only (AP_HAL_Pico directory, waf tool). Board ID
already registered in Step 2.

### Tasks

1. **Board header**
   - Create `libraries/AP_HAL/board/pico.h` with initial defines
   - Add to `#if` chain in AP_HAL_Boards.h (board ID from Step 2)

2. **Create AP_HAL_Pico skeleton**
   - `libraries/AP_HAL_Pico/` directory structure
   - Stub implementations for all HAL interfaces (compile but no-op)
   - `HAL_Pico_Class.cpp` — HAL object, `halInit()` + `chSysInit()`
   - System startup: clock init, LED blink, USB console via ChibiOS

3. **Scheduler (minimal)**
   - ChibiOS thread creation for timer thread
   - `delay()`, `delay_microseconds()`, `millis()`, `micros()`
   - `register_timer_process()` callback list

4. **Semaphores**
   - Wrap ChibiOS `mutex_t` → `HAL_Semaphore`
   - Wrap ChibiOS `binary_semaphore_t` → `HAL_BinarySemaphore`

5. **USB Console (SERIAL0)**
   - ChibiOS USB driver + SerialUSB for `hal.console->printf()`

6. **GPIO basics**
   - `palSetLineMode()`, `palReadLine()`, `palSetLine()`, `palClearLine()`
   - Status LED blink (GP25 on Pico 2)

7. **Build system**
   - Add `pico` board class to `Tools/ardupilotwaf/boards.py`
   - Create `Tools/ardupilotwaf/pico.py` waf tool
   - ChibiOS config files: `mcuconf.h`, `halconf.h`, `chconf.h`
   - Linker script, UF2 generation via picotool
   - Initial `hwdef.dat` for Pico 2 dev board

8. **hwdef processor**
   - Create `pico_hwdef.py` — simpler than STM32 hwdef
   - Generate `hwdef.h` from `hwdef.dat`

### Verification
```bash
./waf configure --board pico2
./waf AP_Periph
# Flash pico2, see LED blink, USB console shows boot messages
```

### Deliverable
- AP_Periph compiles for pico2
- Board boots, LED blinks, `hal.console->printf()` works over USB
- Scheduler timer ticks at correct rate

### GCC Toolchain Note
The full ChibiOS RP2350 demo (88 source files, all LLD drivers, dual-core SMP)
compiles with zero errors using ArduPilot's existing GCC 10.2.1
(`gcc-arm-none-eabi-10-2020-q4-major`). No toolchain change is required.
See [developer-concerns.md](developer-concerns.md) section 6 for the full
build log.

---

## Step 4: Peripherals (one PR per peripheral)

Each peripheral is a separate PR that adds one capability. Order chosen to
build toward the Laurel board's sensor set, with the most fundamental
peripherals first.

### Step 4a: SPI Bus

**Goal:** SPI bus works, can detect devices on the bus.

- ChibiOS `spiStart()` / `spiExchange()` with DMA
- Chip select management via ChibiOS PAL
- Bus locking (ChibiOS `chMtxLock()` per bus)
- Clock frequency calculation from `RP_PERI_CLK`

**Verification:** SPI bus scan detects IMU chip ID.

### Step 4b: IMU (ICM-42688P over SPI)

**Goal:** IMU produces valid accel/gyro readings.

- AP_InertialSensor backend works with SPI driver
- Data-ready interrupt via `palSetLineCallback()`
- Target: 1kHz sample rate

**Verification:** `hal.console` prints accel/gyro data. Values respond to
board movement.

### Step 4c: I2C Bus

**Goal:** I2C bus works, can probe for devices.

- ChibiOS `i2cStart()` / `i2cMasterTransmitTimeout()`
- Bus scanning/probing
- Bus recovery via GPIO bit-bang

**Verification:** I2C scan finds baro at expected address.

### Step 4d: Barometer (DPS310 over I2C)

**Goal:** Barometer produces valid pressure/temperature readings.

- DPS310 driver works over I2C
- Correct altitude estimation from pressure

**Verification:** Baro readings on console, altitude changes with hand over sensor.

### Step 4e: Flash and Storage (Parameters)

**Goal:** Parameters persist across reboots. ArduPilot will not boot without
a working Storage backend, and Storage depends on flash page operations.

- **`AP_HAL::Flash` implementation:** `getpageaddr()`, `getpagesize()`,
  `getnumpages()`, `erasepage()`, `write()`, `keep_unlocked()`,
  `ispageerased()` — maps to RP2350 QSPI flash via ChibiOS EFL driver.
  This is the low-level interface that `AP_FlashStorage` calls.
- **Storage backend:** `AP_HAL::Storage` using `AP_FlashStorage` on top
  of the Flash interface above
- 16KB storage area in flash beyond program code
- Wear-leveling via ArduPilot's existing `AP_FlashStorage` layer
- Flash sector layout: define reserved sectors for storage in linker script

**Verification:** Set parameter, reboot, parameter persists. Verify wear-leveling
by writing parameters repeatedly and confirming flash sector rotation.

### Step 4f: Hardware UARTs

**Goal:** Hardware UART0 and UART1 work at configurable baud rates.

- ChibiOS SIO driver (`sioStart()` on `SIOD0`, `SIOD1`)
- Ring buffer management in UARTDriver
- Configurable baud rate

**Verification:** Loopback test at 115200 and 921600 baud.

### Step 4g: PIO UART

**Goal:** Additional serial ports via PIO state machines.

- **PIO runtime driver** (`pio_manager.cpp`, ~300 LOC): program loading,
  SM configuration, clock divider, FIFO access, DMA setup. Uses ChibiOS
  register definitions from `rp2350.h` (which has the full `PIO_TypeDef`
  struct, `PIO0`/`PIO1`/`PIO2` macros, and DMA DREQ defines).
- **PIO assembler** (`pioasm`): build-time tool from Pico SDK that compiles
  `.pio` files into C headers with instruction arrays. BSD-3-Clause licensed.
  Either integrate as waf build step or pre-assemble and check in headers.
- **PIO instruction encoding** (`pio_instructions.h`): header-only helpers
  from Pico SDK for encoding PIO instructions. Optional — can hand-write.
- PIO programs for UART TX and RX (adapted from Pico SDK examples,
  BSD-3-Clause, or written clean-room)
- GPIO function-select helper for PIO pins (trivial, ~5 lines)
- DMA from PIO FIFOs to ring buffers via ChibiOS DMA driver
- Integration with UARTDriver class

See [architecture.md](architecture.md) "PIO Support: Pico SDK Integration"
for the full analysis of options.

**Verification:** PIO UART loopback test. GPS connected to PIO UART gets fix.

### Step 4h: PWM Output

**Goal:** PWM outputs drive servos.

- ChibiOS PWM driver (`pwmStart()` / `pwmEnableChannelI()`)
- Configurable frequency (50Hz servo / 400Hz ESC)
- Map servo channels to PWM slice/channel via hwdef

**Verification:** Servo responds to RC input (manual test with servo and
oscilloscope).

### Step 4i: ADC (Battery Monitoring)

**Goal:** Battery voltage and current sensing.

- ChibiOS ADC driver with DMA
- Voltage divider scaling from hwdef
- Software filtering/averaging

**Verification:** Battery voltage displayed, matches multimeter.

### Step 4j: RC Input (CRSF via PIO UART)

**Goal:** RC receiver input decoded.

- CRSF/ELRS via PIO UART at 416kbaud (Laurel board default)
- Integration with AP_RCProtocol

**Verification:** RC channel values appear on console from CRSF receiver.

### Step 4k: SD Card and Filesystem

**Goal:** SD card mounts, files can be created and read. Required for logging.

- **SD card SPI driver:** SD card on SPI1 in SPI mode (Laurel: SCK=26, MOSI=27,
  MISO=24, CS=25). Uses ChibiOS SPI driver with appropriate clock speed
  (400kHz for init, up to 25MHz for data).
- **FatFS integration:** ChibiOS FatFS with SPI-mode MMC/SD via `hal_mmc_spi`
  or manual SPI+FatFS wiring. `sdcard_init()` / `sdcard_stop()` / `sdcard_retry()`.
- **Filesystem mount:** Mount on boot, handle card insertion/removal
- **Directory support:** `get_custom_log_directory()`, `get_custom_terrain_directory()`
- **stdio redirection:** `printf()` → USB console (~50 lines)

**Verification:** Mount SD card, create file, write data, read back. List directory.

---

## Step 5: Laurel Board Support

**Goal:** ArduPilot flies on the Laurel (HELLBENDER_0001) board.

**PR scope:** New hwdef, board header adjustments, feature tuning.

### Laurel Board Pin Mapping

From Betaflight HELLBENDER_0001 config:

| Function | GPIO | Betaflight Name | AP_HAL_Pico Use |
|---|---|---|---|
| SPI0 SCK | GP2 | SPI0_SCK | IMU SPI clock |
| SPI0 MOSI | GP3 | SPI0_SDO | IMU SPI data out |
| SPI0 MISO | GP4 | SPI0_SDI | IMU SPI data in |
| IMU CS | GP1 | GYRO_1_CS | ICM-42688P chip select |
| IMU INT | GP22 | GYRO_1_EXTI | Data-ready interrupt |
| SPI1 SCK | GP26 | SPI1_SCK | OSD/SD SPI clock |
| SPI1 MISO | GP24 | SPI1_SDI | SD card data in |
| SPI1 MOSI | GP27 | SPI1_SDO | SD card data out |
| SD CS | GP25 | SDCARD_SPI_CS | SD card chip select |
| I2C0 SDA | GP44 | I2C0_SDA | DPS310 baro |
| I2C0 SCL | GP45 | I2C0_SCL | DPS310 baro |
| UART0 TX | GP12 | UART0_TX | VTX / MSP |
| UART0 RX | GP13 | UART0_RX | VTX / MSP |
| UART1 TX | GP8 | UART1_TX | GPS |
| UART1 RX | GP9 | UART1_RX | GPS |
| PIO UART0 TX | GP20 | PIOUART0_TX | CRSF RC input |
| PIO UART0 RX | GP21 | PIOUART0_RX | CRSF RC input |
| PIO UART1 TX | GP34 | PIOUART1_TX | Aux serial |
| PIO UART1 RX | GP35 | PIOUART1_RX | Aux serial |
| Motor 1 | GP28 | MOTOR1 | PWM output |
| Motor 2 | GP29 | MOTOR2 | PWM output |
| Motor 3 | GP30 | MOTOR3 | PWM output |
| Motor 4 | GP31 | MOTOR4 | PWM output |
| LED0 | GP6 | LED0 | Status LED |
| LED1 | GP7 | LED1 | Status LED |
| LED Strip | GP38 | LED_STRIP | WS2812 (PIO) |
| Beeper | GP5 | BEEPER | PWM buzzer |
| VBAT ADC | GP40 | ADC_VBAT | Battery voltage |
| Current ADC | GP41 | ADC_CURR | Battery current |
| RSSI ADC | GP42 | ADC_RSSI | RSSI |
| Flash CS | GP0 | FLASH_CS | QSPI flash |
| 5V BEC Enable | GP14 | BEC_5V_ENABLE | Power control |
| 9V BEC Enable | GP15 | BEC_9V_ENABLE | Power control |

### Tasks

1. **Create Laurel hwdef.dat**
   - Map all pins from the table above
   - Define IMU, baro, serial ports, PWM outputs, ADC channels
   - Set feature flags appropriate for 520KB RAM

2. **Feature minimization for vehicle build**
   - Use `HAL_MEM_CLASS HAL_MEM_CLASS_300` (520KB SRAM)
   - Disable features per [developer-concerns.md](developer-concerns.md) section 9
   - Profile memory with `arm-none-eabi-size`
   - Target: Copter (the Laurel is a quad FC)

3. **Vehicle integration**
   - Build ArduCopter (or start with Rover if Copter RAM is too tight)
   - Verify sensor data flows through to EKF
   - Verify RC → motor output control loop
   - MAVLink telemetry over USB
   - DataFlash logging to SD card (filesystem from Step 4k)

4. **Flight test**
   - Bench test: motors spin, RC input works, telemetry flows
   - First hover (with safety pilot, conservative PIDs)

### Verification
```bash
./waf configure --board laurel
./waf copter
# Flash, connect GCS, verify sensors, fly
```

### Deliverable
- ArduPilot flies on Laurel board
- Sponsor milestone: working flight controller

---

## Step 6: Additional Peripherals

**Goal:** Full-featured flight controller with DShot, WS2812, and other
peripherals that aren't strictly needed for basic flight but make the board
competitive with Betaflight's feature set.

### Step 6a: DShot Output (PIO)

- PIO-based DShot150/300/600 using PIO runtime from Step 4g
- Write PIO program for DShot bit timing. Betaflight's `dshot.pio` is a
  useful timing reference (GPL-licensed — must write clean-room for ArduPilot).
  The DShot protocol is well-documented; the PIO program is ~13 instructions
  for unidirectional, ~29 for bidirectional.
- DMA from buffer to PIO FIFO via ChibiOS DMA driver
- Bidirectional DShot for ESC telemetry (stretch) — Betaflight has working
  bidir with edge-detection sampling at 75MHz (clkdiv=2)

### Step 6b: WS2812 LEDs (PIO)

- PIO-based WS2812 driver
- NeoPixel notification patterns
- Integration with AP_Notify

### Step 6c: SD Card Logging Optimization

- DMA optimization for SD SPI transfers
- Larger log buffers if RAM permits
- Logging rate optimization

### Step 6d: Compass Support

- If Laurel has a compass (or external I2C compass)
- I2C compass driver verification

### Step 6e: OSD (MAX7456 over SPI1)

- MAX7456 OSD chip on SPI1 (shared with SD card — mutually exclusive
  per Betaflight config)
- ArduPilot OSD integration

### Step 6f: Dual-Core Optimization

- Move I/O threads to Core 1 via ChibiOS SMP
- Profile and optimize loop rate
- Target 400Hz for Copter

### Step 6g: Additional Vehicle Types

- Plane (if RAM allows with minimal feature set)
- Rover
- AP_Periph (CAN peripheral — needs PIO CAN or MCP2515)

---

## Step 7: Full HAL Fidelity

**Goal:** Feature parity with AP_HAL_ChibiOS for capabilities relevant to RP2350.
These items are optional for basic flight but needed for a production-quality port.
Ordered roughly by priority.

### Step 7a: Shared DMA Arbitration

- DMA channel allocation/deallocation with mutex-based locking
- Contention tracking and reporting (for `@SYS/dma.txt`)
- RP2350 has 16 flat DMA channels (simpler than STM32's stream/channel model)
- Used by SPI, ADC, PIO FIFO DMA — needed when multiple peripherals share channels

### Step 7b: Util — System Infrastructure

Core Util methods that most ArduPilot features depend on:

- **Tone alarm / buzzer:** PWM-based buzzer on GP5 (Laurel). ChibiOS PWM driver
  with frequency/volume/duration control. Required by AP_Notify.
- **RTC:** `set_hw_rtc()` / `get_hw_rtc()` — RP2350 has no battery-backed RTC.
  Use ChibiOS RTC driver with epoch counter, or software RTC from GPS time.
- **System ID:** `get_system_id()` — read RP2350's unique 64-bit chip ID from
  OTP (one-time-programmable memory). Used for MAVLink AUTOPILOT_VERSION.
- **Random number generation:** `get_random_vals()` / `get_true_random_vals()` —
  RP2350 has a hardware TRNG (True Random Number Generator). Use for MAVLink
  signing, nonce generation.
- **Memory allocation:** `malloc_type()` with `MEM_DMA_SAFE` / `MEM_FAST` —
  RP2350 has no separate DMA-safe vs cacheable regions (all SRAM is DMA-safe),
  so this simplifies to standard `calloc`.
- **Available memory:** `available_memory()` — report free heap via ChibiOS
  `chCoreGetStatusX()`.
- **Safety switch:** `safety_switch_state()` — GPIO-based if board has one,
  otherwise return `SAFETY_NONE`.

### Step 7c: Watchdog and Fault Handling

- **Watchdog:** RP2350 has a hardware watchdog (ChibiOS WDG driver). Configure
  timeout, feed from scheduler, detect watchdog reset on boot.
- **Persistent data:** Save `PersistentData` struct across watchdog resets.
  RP2350 has no RTC backup registers like STM32 — use watchdog scratch registers
  (4x 32-bit) or reserved SRAM region that survives soft reset.
- **Fault handlers:** HardFault, MemManage, BusFault, UsageFault handlers that
  save fault info to persistent data before watchdog reset.
- **`was_watchdog_reset()`:** Check reset reason register on boot.
- **Stack overflow detection:** ChibiOS `CH_DBG_ENABLE_STACK_CHECK` with
  `AP_stack_overflow()` callback.

### Step 7d: Crash Dump

- Save crash state (registers, stack trace, persistent data) to flash on fault
- `last_crash_dump_size()` / `last_crash_dump_ptr()` for retrieval via MAVLink
- Dedicated flash sector for crash dump storage
- Lower priority than watchdog persistent data (Step 7c)

### Step 7e: Util — Diagnostics and Monitoring

Reporting methods for `@SYS/` virtual filesystem and GCS status:

- **Thread info:** `thread_info()` — enumerate ChibiOS threads with stack usage,
  priority, CPU time. Uses `CH_DBG_ENABLE_STACK_CHECK`.
- **DMA info:** `dma_info()` — report DMA contention statistics from shared_dma.
- **Memory info:** `mem_info()` — report heap fragmentation and pool usage via
  ChibiOS heap APIs.
- **System load:** `get_system_load()` — measure idle thread percentage for
  average and peak CPU load.
- **UART stats:** `uart_info()` / `uart_log()` — per-port TX/RX byte counts,
  error counts, buffer usage.
- **Timer info:** `timer_info()` — report PWM timer frequencies and assignments.
- **Stack logging:** `log_stack_info()` — log per-thread stack high-water marks.

### Step 7f: Flash Bootloader Update

- `flash_bootloader()` in Util — overwrite bootloader from ROMFS image
- Uses `AP_HAL::Flash` page operations (from Step 4e)
- Required for in-field bootloader updates via MAVLink

### Step 7g: DFU Boot

- `boot_to_dfu()` — reboot into USB DFU mode for firmware recovery
- RP2350 has a built-in USB bootloader in ROM (BOOTSEL mode)
- Set flag in persistent data, reset, ROM bootloader checks flag

### Step 7h: DSP (FFT for GyroFFT)

- `AP_HAL::DSP` implementation using CMSIS-DSP `arm_math.h`
- Cortex-M33 supports CMSIS-DSP (same ARM math library as STM32)
- FFT window state, `fft_init()`, `fft_start()`, `fft_analyse()`
- Vector operations: `arm_max_f32`, `arm_scale_f32`, `arm_mean_f32`, `arm_add_f32`
- Required by AP_GyroFFT harmonic notch filter
- May be RAM-constrained on RP2350 (FFT buffers are significant)

### Step 7i: Bidirectional DShot (PIO)

- Extension of Step 6a DShot output
- PIO program switches between TX and RX mode for ESC telemetry
- Edge-detection sampling at high clock rate (Betaflight uses 75MHz, clkdiv=2)
- Decode eRPM, voltage, current, temperature from ESC responses
- Integration with `AP_ESC_Telem_Backend`
- ~29 PIO instructions for bidir (vs ~13 for unidirectional)

### Step 7j: Serial ESC Passthrough (BLHeli)

- `setup_serial_output()` — bit-bang serial protocol on motor output pins
- Used by BLHeli configurator passthrough (BLHeli32/BLHeliS)
- On STM32 this uses timer input capture; on RP2350 can use PIO for
  more flexible serial emulation
- Lower priority — BLHeli configuration can be done via separate USB adapter

### Step 7k: Serial LED via RCOutput Channels

- `set_serial_led_num_LEDs()`, `set_serial_led_rgb_data()`, `serial_led_send()`
- NeoPixel/ProfiLED output on PWM channels (not standalone GPIO)
- On STM32 uses timer-DMA; on RP2350 uses PIO (same mechanism as Step 6b
  but routed through RCOutput channel API)
- Allows LED strips on motor output pins when not used for motors

### Step 7l: CAN Bus (PIO or MCP2515)

- RP2350 has no hardware CAN — two options:
  1. **PIO CAN:** Implement CAN 2.0B via PIO state machines (~2 SMs per bus).
     Challenging but proven possible on RP2040. Bit timing at 1Mbps requires
     careful PIO clock management.
  2. **MCP2515 via SPI:** External CAN controller on SPI bus. Well-supported
     in ArduPilot (`AP_HAL_Linux` has MCP2515 driver). Simpler but requires
     extra hardware.
- `AP_HAL::CANIface` implementation for DroneCAN / UAVCAN
- Important for GPS modules, airspeed sensors, and other CAN peripherals
- PIO CAN would use 2 of 3 PIO blocks — conflicts with DShot + UART allocation

### Step 7m: Persistent Parameters

- `load_persistent_params()` / `get_persistent_param_by_name()` — save key
  calibration data (temperature cal, etc.) in dedicated flash sector
- On STM32 F7/H7 this uses the bootloader sector; on RP2350 use a dedicated
  flash region beyond program and storage areas
- Lower priority — most parameters already persist via normal Storage

### Step 7n: WSPI / QSPI Device Manager

- Wide SPI (quad/octal SPI) device manager for external flash
- RP2350 QMI (QSPI Memory Interface) already handles the boot flash
- Could be used for additional QSPI peripherals if any RP2350 boards have them
- **Very low priority** — no known RP2350 FC boards use separate QSPI devices
  beyond the boot flash

### Step 7o: IMU Heater

- `set_imu_temp()` / `set_imu_target_temp()` — PWM-controlled heater pad
- Requires dedicated GPIO + PWM output for heater element
- Board-specific — only if hardware includes a heater pad
- **Very low priority** for initial RP2350 boards

### Not Applicable to RP2350

These AP_HAL_ChibiOS features are **not needed** for RP2350:

- **IOMCU firmware** (`RCOutput_iofirmware.cpp`): Separate IO co-processor
  pattern used by Pixhawk boards. RP2350 boards don't have a separate IOMCU.
- **SoftSigReader / SoftSigReaderInt**: STM32 timer ICU + DMA based pulse width
  capture for RC protocols. On RP2350, PIO replaces this entirely — PIO UARTs
  handle SBUS/CRSF directly, and PIO can do pulse capture if needed.
- **CANFDIface**: CAN-FD requires hardware FDCAN peripheral (STM32H7/G4 only).
  Even if PIO CAN is implemented, it would be CAN 2.0B only.

---

## PR Sequencing Summary

| # | PR | Touches Existing Code? | Risk | Dependency |
|---|---|---|---|---|
| 1 | ChibiOS submodule update | Submodule pointer only | Low | None |
| 2 | Bootloader | Board ID + board_types.txt | Low | Step 1 |
| 3 | AP_HAL_Pico skeleton + pico2 | New files + board header | Low | Step 2 |
| 4a | SPI bus driver | New files only | None | Step 3 |
| 4b | IMU (ICM-42688P) | New files only | None | Step 4a |
| 4c | I2C bus driver | New files only | None | Step 3 |
| 4d | Barometer (DPS310) | New files only | None | Step 4c |
| 4e | Flash + storage | New files only | None | Step 3 |
| 4f | Hardware UARTs | New files only | None | Step 3 |
| 4g | PIO UART | New files only | None | Step 4f |
| 4h | PWM output | New files only | None | Step 3 |
| 4i | ADC | New files only | None | Step 3 |
| 4j | RC input (CRSF) | New files only | None | Step 4g |
| 4k | SD card + filesystem | New files only | None | Step 4a |
| 5 | Laurel board support | New hwdef + config | Low | Steps 4a-4k |
| 6a | DShot (PIO) | New files only | None | Step 5 |
| 6b | WS2812 (PIO) | New files only | None | Step 5 |
| 6c-g | Additional features | New files only | None | Step 5 |
| 7a | Shared DMA | New files only | None | Step 4a |
| 7b | Util — system infrastructure | New files only | None | Step 3 |
| 7c | Watchdog + fault handling | New files only | None | Step 3 |
| 7d | Crash dump | New files only | None | Step 7c |
| 7e | Util — diagnostics | New files only | None | Step 7a |
| 7f | Flash bootloader update | New files only | None | Step 2, 4e |
| 7g | DFU boot | New files only | None | Step 7c |
| 7h | DSP (GyroFFT) | New files only | None | Step 5 |
| 7i | Bidirectional DShot | New files only | None | Step 6a |
| 7j | Serial ESC passthrough | New files only | None | Step 6a |
| 7k | Serial LED via RCOutput | New files only | None | Step 6b |
| 7l | CAN bus (PIO/MCP2515) | New files only | None | Step 4a |
| 7m | Persistent parameters | New files only | None | Step 4e |
| 7n | WSPI/QSPI device manager | New files only | None | Step 4a |
| 7o | IMU heater | New files only | None | Step 4h |

**Key insight:** After Step 3, virtually all PRs are new files in
`libraries/AP_HAL_Pico/`. They don't touch AP_HAL_ChibiOS or any vehicle code.
This makes them low-risk, easy to review, and safe to merge incrementally.

Steps 4a-4k could also be batched into fewer PRs if the reviewers prefer
(e.g., "SPI + IMU" as one PR, "I2C + Baro" as another). The important thing
is each PR is self-contained and testable.

### Priority Tiers for Step 7

| Tier | Items | Rationale |
|---|---|---|
| **High** | 7a (shared DMA), 7b (Util core), 7c (watchdog), 7f (flash bootloader) | Required for production reliability |
| **Medium** | 7d (crash dump), 7e (diagnostics), 7g (DFU), 7h (DSP), 7i (bidir DShot) | Important for debugging and advanced features |
| **Low** | 7j (BLHeli), 7k (serial LED), 7l (CAN), 7m (persistent params), 7n (WSPI), 7o (heater) | Nice-to-have or hardware-dependent |

**Note:** Bootloader is Step 2 (first bringup milestone). Flash page operations
(`AP_HAL::Flash`) and SD card filesystem are in Steps 4e and 4k respectively —
ArduPilot cannot boot without Storage, and a working board needs SD logging.

---

## Betaflight Reference

Betaflight already has a working RP2350B port (`src/platform/PICO/`) with
the HELLBENDER_0001 board config. Key observations:

- Uses **Pico SDK** (not ChibiOS) — different approach from ours, but the
  board pinout and PIO programs are directly useful as reference
- Has working **DShot PIO programs** (`dshot.pio`) including bidirectional
  DShot with edge detection — useful for understanding timing requirements
  (must be clean-room reimplemented due to GPL vs ArduPilot license)
- **PIO allocation:** PIO0 → DShot motors, PIO1 → PIO UARTs, PIO2 → LED strip
- **Runs from RAM** by default (`RUN_FROM_RAM=1`) to avoid XIP latency —
  we should evaluate this approach vs SRAM code placement for hot paths
- Uses `-mcmse` flag (TrustZone) — we don't need this
- Has multicore support via Pico SDK multicore API
- SD card on SPI1, mutually exclusive with MAX7456 OSD

The Betaflight config gives us a verified, working pin mapping for the
Laurel board. We don't need to reverse-engineer the schematic.
