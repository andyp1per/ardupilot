# Approach Comparison: Separate AP_HAL_Pico vs Extend AP_HAL_ChibiOS

## Context

ChibiOS trunk (in [ChibiOS.svn](https://github.com/ArduPilot/ChibiOS.svn)) now includes RP2350 support with:
- **Complete LLD drivers:** ADC, DMA, GPIO, I2C, PWM, SPI, UART (SIO), USB, WDG, Timer, Flash
- **Full SMP/dual-core support** with inter-core FIFO, spinlocks, core-local memory
- **Linker scripts** for both XIP (flash) and RAM execution
- **Board support** for Pico 2 (RP_PICO2_RP2350)
- **No PIO driver** in ChibiOS (the key gap)
- **No CAN driver** (RP2350 has no hardware CAN)

AP_HAL_ChibiOS analysis shows:
- **Portable via ChibiOS APIs:** Scheduler, GPIO, RCInput, Storage, shared_dma
- **Partially portable:** UART, SPI, I2C (ChibiOS HAL + STM32 clock/timing specifics)
- **Heavily STM32-specific:** RCOutput/DShot (direct timer registers), system.cpp, AnalogIn (ADC family conditionals)
- **hwdef system:** Deeply coupled to STM32 pin mux, DMA topology, timer grouping

Both approaches use ChibiOS RT kernel and the RP2350 LLD drivers underneath.
The question is whether the ArduPilot HAL wrapper layer lives in AP_HAL_ChibiOS
or in a new AP_HAL_Pico.

---

## Approach A: Extend AP_HAL_ChibiOS

Add RP2350 support directly into the existing AP_HAL_ChibiOS, alongside STM32.

### Pros

1. **No code duplication** for portable components (Scheduler, GPIO, Storage,
   shared_dma, Semaphores) - these use ChibiOS APIs and work on both platforms
2. **Bug fixes propagate automatically** - a fix in Scheduler.cpp benefits
   both STM32 and RP2350
3. **Single HAL to maintain** - reduces long-term maintenance burden
4. **Community familiarity** - developers already know AP_HAL_ChibiOS structure
5. **hwdef infrastructure** - can extend existing hwdef.py rather than rewriting
6. **Waf integration** - chibios.py already handles ChibiOS builds, just needs
   RP2350 toolchain support added
7. **Consistent patterns** - both platforms follow same coding conventions,
   same file organization, same review process

### Cons

1. **#ifdef pollution** - RCOutput.cpp already has STM32H7/F4/F7/L4/G4
   conditionals; adding RP2350 makes it worse. The file is 1900+ lines with
   deep STM32 timer register access throughout
2. **RCOutput must be heavily conditionally compiled** - STM32 timers vs
   RP2350 PWM slices are fundamentally different architectures. DShot via
   timer DMA (STM32) vs PIO state machines (RP2350) share essentially no code
3. **hwdef.py is deeply STM32-coupled** - RP2350 has a completely different
   pin mux model (any GPIO can be any function), no DMA stream topology to
   encode, no timer channel grouping. The existing hwdef scripts embed STM32
   alternate-function tables, DMA mappings, and clock tree knowledge
4. **Risk of breaking STM32 builds** - every change during development must
   be tested against existing boards. A typo in a shared file breaks everyone
5. **Slow iteration** - changes require careful consideration of both platforms;
   can't move fast on RP2350 without reviewing STM32 impact
6. **Code complexity** - files like SPIDevice.cpp already have clock calculation
   blocks for each STM32 family; RP2350 would add another block. Each file
   becomes harder to read and maintain
7. **PIO has no analog in STM32** - PIO-based UART, CAN, DShot, SBUS all need
   new code that has zero overlap with STM32 implementations. Forcing this
   into AP_HAL_ChibiOS files creates awkward structure
8. **AnalogIn.cpp** has MCU-family detection with different ADC register access
   per family; RP2350's simpler ADC doesn't benefit from this complexity
9. **system.cpp** startup, fault handling, and watchdog are all STM32-specific
10. **UARTDriver.cpp** stores STM32 USART CR1/CR2/CR3 register options in
    struct members - these registers don't exist on RP2350

---

## Approach B: Separate AP_HAL_Pico (Using ChibiOS Underneath)

Create `libraries/AP_HAL_Pico/` as a new HAL backend that still uses ChibiOS
RT kernel and RP LLD drivers, but has its own ArduPilot HAL wrapper files.

### Pros

1. **Clean codebase** - no #ifdef tangles mixing STM32 and RP2350 register
   access. Each file is readable and self-contained
2. **Rapid development** - can iterate freely on RP2350 without fear of
   breaking any existing STM32 board. No cross-platform testing needed for
   each change
3. **PIO is a first-class citizen** - PIO UART, PIO SBUS, PIO DShot, PIO CAN
   can be designed cleanly from the start rather than retrofitted into
   STM32-shaped code
4. **Simpler hwdef** - RP2350's "any pin, any function" model doesn't need
   the complex alternate-function tables that STM32 requires. A simpler
   hwdef parser means less code and fewer bugs
5. **Easier code review** - PRs only touch Pico-specific files, reviewers don't
   need to verify STM32 side effects
6. **Natural for the hardware** - RCOutput can use PWM slices + PIO natively
   rather than being forced into STM32 timer abstractions
7. **Smaller, simpler files** - without the multi-family conditional compilation,
   each driver file is significantly shorter and more understandable
8. **Independent release cadence** - can stabilize RP2350 support on its own
   timeline without being gated on STM32 release cycles
9. **Precedent:** ESP32 port took this approach successfully. The ESP32 HAL
   is clean, maintainable, and doesn't pollute other HALs
10. **Easier onboarding** - a developer wanting to contribute to RP2350 support
    only needs to understand AP_HAL_Pico, not all of AP_HAL_ChibiOS

### Cons

1. **Code duplication** - Scheduler, Semaphores, and some utility code will be
   duplicated from AP_HAL_ChibiOS. Both wrap the same ChibiOS thread/mutex APIs
2. **Divergence risk** - over time, AP_HAL_ChibiOS may get improvements (e.g.,
   better scheduler timing, new Storage backends) that don't automatically
   propagate to AP_HAL_Pico
3. **Two ChibiOS HALs to maintain** - community must track both. Bug in ChibiOS
   thread handling might need fixing in two places
4. **More files in the repo** - roughly 20-30 new files in libraries/
5. **waf build system** - needs a new `pico.py` waf tool (though it can follow
   the `chibios.py` pattern closely)
6. **Board ID registration** - needs a new HAL_BOARD type (HAL_BOARD_PICO = 14)
   and associated board header. More entries in the #if chain in AP_HAL_Boards.h
7. **build_options.py** - may need RP2350-specific feature flags alongside
   existing ChibiOS ones
8. **Potential for inconsistency** - if ChibiOS updates its RP LLD drivers,
   both HALs need updating, but AP_HAL_Pico might lag behind

---

## Code Sharing Analysis

What would be duplicated vs unique in Approach B:

| Component | Lines (est.) | Shared? | Notes |
|---|---|---|---|
| Scheduler | ~800 | ~70% similar | Same ChibiOS thread APIs, but RP2350 has different watchdog, dual-core model |
| Semaphores | ~200 | ~95% similar | Thin ChibiOS wrappers, nearly identical |
| UARTDriver | ~1700 | ~30% similar | Ring buffer logic shared; HW setup, DMA, PIO UART completely different |
| SPIDevice | ~600 | ~40% similar | Protocol logic shared; clock calcs, DMA setup different |
| I2CDevice | ~300 | ~50% similar | ChibiOS i2c API shared; timing config different |
| GPIO | ~400 | ~80% similar | ChibiOS PAL API; interrupt setup differs slightly |
| RCOutput | ~2500 | ~5% similar | Fundamentally different (STM32 timers vs RP2350 PWM+PIO) |
| RCInput | ~300 | ~60% similar | Protocol decoding shared; pulse capture mechanism differs |
| AnalogIn | ~600 | ~20% similar | ADC architecture completely different |
| Storage | ~500 | ~70% similar | FRAM/SD backends identical; flash backend different |
| shared_dma | ~400 | ~60% similar | Allocation logic similar; RP2350 DMA topology simpler |
| system.cpp | ~400 | ~10% similar | Startup, faults, clocks all different |
| **Total** | **~8700** | | |

Estimated unique code in AP_HAL_Pico: ~5500 lines
Estimated duplicated code: ~3200 lines (from ~8700 total in AP_HAL_ChibiOS)

However, the "duplicated" code is mostly simple ChibiOS API wrappers that are
stable and rarely change. The unique code is where the real complexity and
active development happens.

---

## Mitigation Strategies for Approach B (Separate HAL)

The main downsides of a separate HAL can be mitigated:

### 1. Code Duplication → Extract shared utilities

Common ChibiOS wrapper code could potentially live in a shared location:
```
libraries/AP_HAL_ChibiOS_Common/   (or inline headers)
    ChibiOS_Semaphores.h           # Thin ChibiOS semaphore wrappers
    ChibiOS_Scheduler_Common.h     # Timer/thread registration patterns
```
However, this adds complexity and may not be worth it for ~3000 lines of
stable wrapper code. The ESP32 port duplicates its FreeRTOS wrappers without
issues.

### 2. Divergence → Periodic sync reviews

Establish a process to review AP_HAL_ChibiOS changes and assess applicability
to AP_HAL_Pico. Since the ChibiOS wrapper layer changes infrequently (a few
times per release), this is low overhead.

### 3. ChibiOS updates → Single submodule

Both HALs can share the same ChibiOS submodule. ChibiOS LLD updates benefit
both. The ArduPilot wrapper layer is what differs, not the ChibiOS code itself.

---

## How Developer Concerns Reinforce Separation

Detailed technical analysis (see [developer-concerns.md](developer-concerns.md))
of the specific concerns raised during review further strengthens the case for a
separate HAL:

### DMA architecture is fundamentally different

AP_HAL_ChibiOS has 167 DMA references across 23 files, built around STM32's
stream/channel/FIFO/DMAMUX model. RP2350 DMA is architecturally simpler (flat
16-channel pool, no FIFO, no streams hierarchy). The key coupling point is
`Shared_DMA`, which is primarily used by RCOutput for STM32 timer-DMA DShot —
a pattern that doesn't exist on RP2350 where DShot is PIO-based. In a separate
HAL, `shared_dma` can be simpler and RP2350-native rather than trying to map
RP2350's model onto STM32 abstractions.

### Dual-core scheduling is RP2350-specific

ChibiOS SMP on RP2350 uses inter-processor FIFO, spinlock #31, and per-core
ChibiOS instances (`ch0`, `ch1`). The Scheduler in AP_HAL_ChibiOS has no
concept of multi-core — adding core affinity, cross-core thread placement, and
SMP initialization to a Scheduler that currently assumes single-core STM32
would require significant #ifdef blocks or runtime branching. A separate
Scheduler can be designed for dual-core from the start.

### hwdef is more different than just "expandable"

While `chibios_hwdef.py` inherits from `hwdef.HWDef` (expandable by design),
the STM32-specific content isn't just tables — it's the entire processing model.
STM32 hwdef resolves alternate-function conflicts, validates DMA stream
sharing, groups timer channels for PWM, and computes bus clock prescalers.
None of these concepts exist on RP2350. Bolting RP2350 onto `chibios_hwdef.py`
would mean most of the code does nothing for Pico boards while a large new
RP2350-specific section handles PIO allocation, simple pin function select,
and PWM slice assignment. A separate `pico_hwdef.py` is naturally ~1/3 the
size and complexity.

### Performance tuning needs are RP2350-specific

At 150MHz with external flash, the optimization strategies are unlike STM32:
SRAM code placement (`__not_in_flash_func()`), XIP cache management, PIO
offloading, and overclock configuration. These need first-class integration
in system.cpp, Scheduler, and RCOutput — not afterthought #ifdefs in files
designed around STM32's internal flash and fast single-core model.

### PIO requires Pico SDK build tooling

PIO programs need `pioasm` (the PIO assembler from Pico SDK) as a build-time
tool, and benefit from `pio_instructions.h` (header-only instruction encoding
helpers). The PIO runtime driver (~300 LOC) uses register definitions that
ChibiOS provides but needs GPIO function-select for PIO pins that ChibiOS
PAL doesn't expose. None of this fits naturally into AP_HAL_ChibiOS's build
system, which has no concept of PIO programs, `.pio` source files, or a
host-tool compilation step for `pioasm`.

In a separate HAL, the waf build tool (`pico.py`) can cleanly integrate
`pioasm` as a build step, manage `.pio` → `.pio.h` generation, and include
the PIO runtime alongside the ChibiOS drivers without any interaction with
the STM32 build path. In AP_HAL_ChibiOS, this would mean adding PIO build
steps to `chibios.py` that only apply to RP2350 boards — more complexity
in an already complex build system for zero benefit to STM32 users.

### Feature disabling is more aggressive

While 520KB SRAM is actually more than the STM32F405's 192KB (MatekF405 runs
Copter in this), the RP2350 still needs feature disabling similar to F4 boards.
The feature set will be comparable to existing `minimize_fpv_osd.inc` boards.
This isn't a problem for separation per se, but it means RP2350 builds exercise
a similar subset of ArduPilot code to resource-constrained F4 boards rather than
the full H7 feature set, reducing the "shared code benefits" argument for
keeping them in one HAL.

---

## Recommendation

**Approach B (Separate AP_HAL_Pico) is the better choice** for this project, for these reasons:

1. **Development velocity** is the highest priority for a new port. Being able
   to iterate freely without cross-platform testing saves enormous time.

2. **The code that would be duplicated is boring and stable** (ChibiOS API
   wrappers). The code that must be unique is where all the complexity lives
   (RCOutput, PIO, hwdef, system startup). Duplicating ~3000 lines of stable
   wrappers is a small price for clean, readable RP2350-specific code.

3. **RCOutput alone justifies separation.** At 2500+ lines, it's the largest
   and most complex driver. STM32 timer-DMA DShot and RP2350 PIO DShot share
   essentially zero implementation code. Putting both in one file with #ifdefs
   would be unmaintainable.

4. **PIO is fundamental to RP2350** and has no STM32 equivalent. PIO UARTs,
   PIO CAN, PIO SBUS, PIO DShot all need clean integration points that don't
   fit the STM32 model.

5. **DMA, scheduling, and hwdef are all architecturally different** enough
   that "extending" AP_HAL_ChibiOS means maintaining two parallel
   implementations side-by-side in the same files — the worst of both worlds.
   See "How Developer Concerns Reinforce Separation" above.

6. **Precedent works.** The ESP32 port proved that a separate HAL using a
   different RTOS/SDK is viable and maintainable long-term.

7. **hwdef simplicity.** RP2350's pin mux model is radically simpler than
   STM32's. A dedicated hwdef parser can be clean and small instead of
   bolting RP2350 support onto the STM32-oriented chibios_hwdef.py.
