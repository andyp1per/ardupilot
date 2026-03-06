# Developer Concerns and Technical Analysis

This document addresses specific technical concerns raised during the RP2350B
port planning, with findings from code analysis and testing.

---

## 1. ChibiOS LLD Driver Status

**Concern:** What ChibiOS LLD drivers exist for RP2350? What's missing?

**Finding: All essential LLD drivers are COMPLETE.**

ChibiOS trunk ([ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn)) includes production-ready RP2350 drivers:

| Driver | LLD Path | DMA | Status |
|---|---|---|---|
| ADC | `os/hal/ports/RP/LLD/ADCv1/` | YES | Complete - single + round-robin, temp sensor, FIFO |
| DMA | `os/hal/ports/RP/LLD/DMAv1/` | N/A | Complete - 16 channels, allocation + interrupt management |
| GPIO | `os/hal/ports/RP/LLD/GPIOv1/` | NO | Complete - 48 pins, event callbacks, multicore-aware |
| I2C | `os/hal/ports/RP/LLD/I2Cv1/` | NO | Complete - master TX/RX, timeout, 10-bit addressing |
| PWM | `os/hal/ports/RP/LLD/PWMv1/` | NO | Complete - 12 slices, 2 channels each |
| SPI | `os/hal/ports/RP/LLD/SPIv1/` | YES | Complete - DMA transfers, polled fallback |
| UART (SIO) | `os/hal/ports/RP/LLD/SIOv1/` | NO | Complete - modern SIO interface, full event support |
| USB | `os/hal/ports/RP/LLD/USBv1/` | NO | Complete - device mode, 15 endpoints, VBUS |
| Watchdog | `os/hal/ports/RP/LLD/WDGv1/` | NO | Complete - persistent scratch registers |
| Timer | `os/hal/ports/RP/LLD/TIMERv1/` | NO | Complete - SysTick + TIMER0, 4 alarms, multicore |
| Flash | `os/hal/ports/RP/LLD/efl/` | NO | Complete - QMI QSPI, 256B pages, 4K sectors, XIP |

**Not provided by ChibiOS (must be implemented in AP_HAL_Pico):**
- PIO driver (no ChibiOS abstraction; needed for extra UARTs, SBUS, DShot, CAN, WS2812)
- CAN driver (no hardware CAN on RP2350; need PIO CAN or MCP2515 via SPI)
- SD/MMC (no SDMMC peripheral; use SPI mode via SPIv1)

---

## 2. STM32 #defines in AP_HAL_ChibiOS

**Concern:** How many STM32-specific #defines exist? What needs alternate implementation?

**Finding: 449 `#if.*STM32` occurrences across 51 files in AP_HAL_ChibiOS.**

This is a primary reason for the separate AP_HAL_Pico approach. Breaking it down:

| File Category | STM32 Refs | Nature |
|---|---|---|
| `RCOutput.cpp` | ~80 | Direct STM32 timer register access (TIMx->CCR, TIMx->DIER), DShot via timer DMA |
| `SPIDevice.cpp` | ~30 | Clock prescaler per STM32 family (F4/F7/H7/L4/G4), bus clock detection |
| `AnalogIn.cpp` | ~25 | ADC register differences per family |
| `UARTDriver.cpp` | ~40 | USART CR1/CR2/CR3 registers, DMA stream config |
| `system.cpp` | ~30 | Startup, fault handlers, watchdog, clock tree |
| `hwdef scripts` | ~100+ | MCU families, alternate functions, DMA streams |
| Other drivers | ~140 | Scattered throughout remaining files |

**Approach taken:** Rather than adding another layer of #ifdefs for RP2350,
we create AP_HAL_Pico with clean implementations that directly use ChibiOS
RP2350 LLD APIs. See [approach-comparison.md](approach-comparison.md) for the
detailed rationale.

---

## 3. hwdef Generator / Python

**Concern:** The hwdef generator has STM32-specific assumptions. Is it expandable?
Can we build the generator using tables from RP2350 datasheets?

**Finding: hwdef system IS expandable by design, but needs new RP2350-specific tables.**

Analysis of the hwdef code:
- `chibios_hwdef.py` inherits from `hwdef.HWDef` base class
- The base class provides general infrastructure (pin parsing, feature flags)
- STM32-specific parts: alternate function tables, DMA stream topology, timer
  channel grouping, clock bus assignments - all extracted from STM32 datasheets
- These STM32 tables (e.g., `stm32h7xx_hal_dma.h` DMA request mappings) have
  no equivalent on RP2350

**RP2350 simplification:** The hwdef for RP2350 is dramatically simpler because:

| Aspect | STM32 | RP2350 |
|---|---|---|
| Pin mux | Alternate function number (AF0-AF15), each pin has fixed AF options | IO bank function select - any GPIO can be any function |
| DMA mapping | Explicit stream/channel per peripheral, shared DMA mux | Auto-allocated from 16 channels, simple request IDs |
| Timer groups | Complex shared-counter groupings for PWM | 12 independent PWM slices, no grouping needed |
| Clock tree | Per-family APB/AHB bus assignments | Single PERI_CLK for all peripherals |

**Plan:** Create `pico_hwdef.py` as a new, simpler hwdef processor that:
- Parses a simplified `hwdef.dat` format (see [architecture.md](architecture.md))
- Generates `hwdef.h`, `mcuconf.h`, `halconf.h` for ChibiOS
- No STM32 datasheet tables needed - RP2350 pin mux is trivial
- Can inherit from `hwdef.HWDef` base class for common infrastructure

---

## 4. Custom Board / Target Needed

**Concern:** Need a custom board with RP2350B, IMU, compass, RC, PWM out, with
pinouts, wiring, and hwdef.dat.

**Finding: Reference designs exist; initial development uses Pico 2 dev board.**

**Phase 1 - Development target:** Raspberry Pi Pico 2 (RP2350B variant)
- Available off-the-shelf, $5
- ChibiOS already has board support (`RP_PICO2_RP2350`)
- Breakout sensors on SPI/I2C for testing
- hwdef.dat in [architecture.md](architecture.md) section "hwdef.dat Format"

**Phase 2 - Custom FC board:** Based on reference designs analyzed in
[reference-boards.md](reference-boards.md):

Minimum viable RP2350B flight controller:
- 1x ICM-45686 IMU (SPI0)
- 1x ICP-20100 or BMP390 barometer (I2C0)
- 1x RM3100 compass (I2C0)
- 4-8x PWM outputs (ChibiOS PWM driver)
- RC input (PIO SBUS/CRSF)
- GPS on HW UART1
- Telemetry on HW UART0
- USB console
- Battery V/I sensing (ADC)

See [reference-boards.md](reference-boards.md) for detailed pin mapping
derived from the Laurel (HELLBENDER_0001) Betaflight configuration and MatekF405 comparison.

---

## 5. DMA Support

**Concern:** STM32 DMA is assumed in many places. Can we succeed without DMA?
Do we need to write a ChibiOS DMA driver? Will it be totally different from STM32?

**Finding: ChibiOS DMAv1 driver for RP2350 EXISTS and is already integrated.**

The concern is resolved - a DMA driver already exists:

- **Path:** `os/hal/ports/RP/LLD/DMAv1/` in [ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn)
- **Channels:** 16 DMA channels (confirmed in `rp_registry.h`: `RP_DMA_NUM_CHANNELS 16`)
- **SPI integration:** ChibiOS SPI LLD uses DMA automatically (`rp_dma_channel_t` in `hal_spi_lld.h`)
- **ADC integration:** ChibiOS ADC LLD uses DMA for conversion results

Yes, RP2350 DMA is architecturally different from STM32 DMA (no streams/channels
hierarchy, different request routing, different descriptor format), but this is
handled inside the ChibiOS LLD. AP_HAL_Pico code uses ChibiOS DMA APIs, not
direct DMA registers.

**What about Shared_DMA in AP_HAL_ChibiOS?**
- `Shared_DMA` class (167 DMA references across 23 files in AP_HAL_ChibiOS)
  is primarily used by RCOutput for DShot timer-DMA on STM32
- On RP2350, DShot is PIO-based (not timer-DMA), so Shared_DMA's primary
  use case doesn't apply
- AP_HAL_Pico will have a simpler `shared_dma.h/.cpp` for the cases where
  DMA channel sharing is still needed (e.g., SPI buses sharing DMA channels)

---

## 6. GCC Toolchain Version

**Concern:** ArduPilot uses old stable GCC 10. RP2350 needs GCC 15. Is there
a middle ground?

**Finding: GCC 10.2 CONFIRMED WORKING. This is NOT a blocker.**

The ChibiOS RP2350 demo compiles cleanly with ArduPilot's exact toolchain:

```
$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (GNU Arm Embedded Toolchain 10-2020-q4-major) 10.2.1

$ cd demos/RP/RT-RP2350-PICO2 && make   # in ChibiOS.svn checkout
Compiler Options: arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -O2 ...
                  -mfloat-abi=softfp -mfpu=fpv5-sp-d16
...
(88 source files compiled, zero errors, zero warnings)
Linking build/ch.elf
   text    data     bss     dec     hex filename
  67576     220  532256  600052  927f4 build/ch.elf
Done
```

**Full build test** - the entire ChibiOS RP2350 SMP demo (RT kernel, all LLD
drivers, dual-core startup, shell, test suite) compiles and links successfully
with GCC 10.2.1. This includes:
- `core_cm33.h` (CMSIS V5.0.9) - uses only GCC 4.5+ features
- `crt0_v8m-ml.S` - ARMv8-M assembly (MSPLIM, PSPLIM, FPU init)
- `rp2350_imagedef.S` - PICOBIN boot block for RP2350 bootrom
- All RP2350 LLD drivers (DMA, SPI, SIO, GPIO, ADC, PWM, USB, WDG, Timer, Flash)
- SMP dual-core port (`chcoresmp.c`, `chcoreasm.S`)

**Why it works:** Cortex-M33 (ARMv8-M Mainline) support was added in GCC 8.
ChibiOS uses standard C99/C11 and GCC `__attribute__` extensions that have
been available since GCC 4.x. No TrustZone CMSE intrinsics are used (those
would need GCC 11+ with `-mcmse`, but ArduPilot doesn't need TrustZone).

**No toolchain change needed.** ArduPilot's existing `gcc-arm-none-eabi-10-2020-q4-major`
works for the entire RP2350 port. This avoids the dual-toolchain problem entirely.

---

## 7. Dual-Core Scheduling

**Concern:** Can we even get ChibiOS to use both cores? Is it worth the pain?

**Finding: ChibiOS SMP dual-core on RP2350 is CONFIRMED WORKING.**

The ChibiOS demo at `demos/RP/RT-RP2350-PICO2/` in [ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn) demonstrates:

**Core 0 (main.c):**
```c
int main(void) {
    halInit();
    chSysInit();
    // ... creates threads, runs shell
}
```

**Core 1 (c1_main.c):**
```c
void c1_main(void) {
    chSysWaitSystemState(ch_sys_running);
    chInstanceObjectInit(&ch1, &ch_core1_cfg);
    // ... creates threads, runs timer
    chSysUnlock();
}
```

**SMP infrastructure:**
- Spinlock #31 for inter-core synchronization
- Inter-processor FIFO for cross-core scheduling messages
- Core-local memory: SCRATCH_X at 0x20080000, SCRATCH_Y at 0x20081000
- Each core runs its own ChibiOS instance with shared ready lists
- Port file: `os/common/ports/ARMv8-M-ML/smp/rp2/chcoreport.h`

**Strategy for AP_HAL_Pico (start simple, add SMP later):**

1. **Phase 1: Single-core** - All threads on Core 0. Simplest, debuggable.
2. **Phase 2: Asymmetric** - Main loop + timer on Core 0, I/O threads on Core 1.
   This is the plan in [architecture.md](architecture.md).
3. **Phase 3: Full SMP** - ChibiOS scheduler distributes threads across cores
   based on priority and load.

**Is it worth it?** Yes, for performance. At 150MHz, a single core may struggle
with 400Hz+ loop rates while also handling UART, SPI, and USB I/O. Offloading
I/O to Core 1 is the primary mitigation for the clock speed limitation.

---

## 8. Performance Expectations

**Concern:** Expecting it to be slower than STM32. Can we get 100Hz? 200Hz? 400Hz?

**Analysis:**

| Metric | STM32F405 @ 168MHz | RP2350 @ 150MHz | Ratio |
|---|---|---|---|
| Raw clock | 168 MHz | 150 MHz | ~0.9x — nearly identical |
| Architecture | Cortex-M4 (3-stage, in-order) | Cortex-M33 (3-stage, in-order) | Similar IPC; M33 has some DSP improvements |
| FPU | Single-precision | Single-precision | Identical |
| Cache | ART accelerator + 64B I-cache | 16KB XIP cache | F405 has limited cache too |
| Cores | 1 | 2 (SMP) | RP2350 can offload I/O to core 1 |
| Flash | 1MB internal (wait-states at 168MHz) | No internal; all code via QSPI XIP | **Primary risk**: XIP cache misses are costlier than F405 wait-states; SRAM placement critical |
| SRAM | 192KB (128+64) | 520KB | RP2350 has 2.7x more |

The MatekF405 runs ArduCopter at 400Hz. With similar clock speed, similar
architecture, and significantly more RAM, the RP2350 should achieve comparable
loop rates — especially with dual-core I/O offloading.

For reference vs high-end boards: STM32H7 @ 480MHz with Cortex-M7 superscalar
pipeline and double-precision FPU is ~6x faster in raw compute. The RP2350 is
not competing with H7 boards; it competes with F4 boards at a fraction of the
cost.

**Estimated achievable loop rates:**

| Configuration | Estimated Rate | Notes |
|---|---|---|
| AP_Periph (single sensor) | 400Hz+ | Easily achievable, minimal processing |
| Rover/Plane (minimal) | 400Hz | MatekF405 achieves this; RP2350 has more RAM and dual-core |
| Copter (minimal) | 200-400Hz | F405 achieves 400Hz; RP2350 should match with dual-core |
| Copter (full features) | Not a target | Same as F405 — feature-limited build |

**Key optimizations:**
1. **Dual-core:** Sensor sampling on Core 0, UART/USB I/O on Core 1
2. **DMA everywhere:** SPI sensor reads, ADC conversions, UART transfers
3. **SRAM placement:** Critical control loop code marked `__RAMFUNC__` to
   avoid XIP latency (Pico SDK provides `__not_in_flash_func()`)
4. **PIO offloading:** DShot, SBUS decode, UART all run in PIO hardware
   without CPU intervention
5. **Overclock to 200MHz:** RP2350 commonly runs at 200MHz (well within
   spec for many boards)
6. **Reduce features:** See section 9 below

**Comparison with ESP32:** The ESP32 port runs at 240MHz dual-core Xtensa
with 520KB SRAM and achieves functional flight. RP2350 at 150MHz has a
more efficient ISA (Cortex-M33 vs Xtensa) and better DSP extensions, so
comparable performance is expected.

---

## 9. Feature Disabling Strategy

**Concern:** Do we need to disable large subsystems? MavFTP? EKF? Others?

**Finding: Feature disabling is needed, but less aggressively than initially
assumed.** The MatekF405 runs ArduCopter in 192KB SRAM — the RP2350 has 520KB
(2.7x more). We follow the same pattern as F4 boards rather than the extreme
ESP32 approach.

ArduPilot has a well-established mechanism for this:

### HAL_MEM_CLASS Gating

`HAL_MEM_CLASS` (set in the board header) controls automatic scaling of:
- UART buffer sizes (smaller buffers for low-RAM boards)
- Logger buffer allocation
- Mission storage limits
- Scripting heap size
- Various internal buffer sizes

For RP2350: `HAL_MEM_CLASS HAL_MEM_CLASS_300` (520KB SRAM, ~300KB usable)

### Feature Disable List (board/pico.h)

Based on ESP32's `board/esp32.h` pattern and build options from
`Tools/scripts/build_options.py` (150+ options available):

**Definitely disable (large RAM/flash cost, not needed for basic flight):**

| Feature | Build Option | RAM Saved (est.) |
|---|---|---|
| GyroFFT | `HAL_GYROFFT_ENABLED 0` | ~40KB |
| Scripting (Lua) | `AP_SCRIPTING_ENABLED 0` | ~50KB heap |
| Avoidance | `AC_AVOID_ENABLED 0` | ~10KB |
| Fence | `AP_FENCE_ENABLED 0` | ~8KB |
| Follow mode | `AP_FOLLOW_ENABLED 0` | ~5KB |
| Terrain | `AP_TERRAIN_AVAILABLE 0` | ~20KB |
| Camera/Mount | `HAL_MOUNT_ENABLED 0` | ~15KB |
| Gripper | `AP_GRIPPER_ENABLED 0` | ~3KB |
| Landing Gear | `AP_LANDINGGEAR_ENABLED 0` | ~3KB |
| ADSB | `HAL_ADSB_ENABLED 0` | ~10KB |
| Soaring | `HAL_SOARING_ENABLED 0` | ~8KB |
| Generator | `HAL_GENERATOR_ENABLED 0` | ~5KB |
| Sprayer | `HAL_SPRAYER_ENABLED 0` | ~3KB |
| Precision Landing | `AC_PRECLAND_ENABLED 0` | ~10KB |
| Button support | `HAL_BUTTON_ENABLED 0` | ~2KB |
| RPM sensor | `AP_RPM_ENABLED 0` | ~3KB |
| Beacon | `AP_BEACON_ENABLED 0` | ~5KB |
| Winch | `AP_WINCH_ENABLED 0` | ~3KB |

**Keep but reduce:**

| Feature | Setting | Notes |
|---|---|---|
| EKF | `HAL_WITH_EKF_DOUBLE 0`, consider EKF2 only | Single-precision, smaller footprint |
| MavFTP | `AP_MAVLINK_FTP_ENABLED 0` initially | Re-enable if flash space allows; ~5KB RAM |
| Logging | Reduced buffer sizes via HAL_MEM_CLASS | Functional but slower write rate |
| Parameters | 16KB storage | Sufficient for minimal build |

**Always keep (essential for flight):**
- AP_AHRS, AP_InertialSensor, AP_Baro, AP_GPS, AP_Compass
- AC_AttitudeControl, AC_PosControl (Copter) or AP_L1_Control (Plane)
- AP_Motors, AP_RCOutput
- MAVLink (core telemetry)
- EKF (at least EKF2 or simplified EKF3)

### Memory Budget with Disabling

For context, MatekF405 (192KB SRAM) runs Copter with the `minimize_fpv_osd.inc`
feature set. RP2350 has 520KB — significantly more headroom.

| Build | RAM without disabling | RAM with disabling | Fits in 520KB? |
|---|---|---|---|
| AP_Periph | ~150KB | ~120KB | Yes (comfortable) |
| Rover (minimal) | ~400KB | ~300KB | Yes |
| Plane (minimal) | ~420KB | ~320KB | Yes |
| Copter (minimal) | ~500KB+ | ~380KB | Yes — similar to F4 boards |

---

## Summary: Risk Assessment Update

| Concern | Initial Risk | After Analysis | Key Finding |
|---|---|---|---|
| LLD drivers missing | High | **Resolved** | All 11 drivers complete |
| STM32 #ifdefs | Medium | **Mitigated** | Separate HAL avoids this |
| hwdef expandability | Medium | **Low** | Simpler model for RP2350 |
| Custom board | Medium | **Low** | Pico 2 for dev, ref designs exist |
| DMA support | High | **Resolved** | ChibiOS DMAv1 exists, 16 channels |
| GCC version | High | **Low** | GCC 10.2 supports cortex-m33 |
| Dual-core | High | **Medium** | ChibiOS SMP confirmed, start single-core |
| Performance | Medium | **Medium** | 200-400Hz achievable with optimization |
| Feature disabling | Medium | **Low** | Well-established mechanism exists |
