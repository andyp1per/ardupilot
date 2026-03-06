# Code Execution Performance: STM32F405 vs RP2350B

**Sources:** STM32F405/407 Datasheet (DS8626 Rev 9, Aug 2020);
RP2350 Datasheet (RP-008373-DS-2, build-date 2025-07-29).

## The Viability Threshold

The STM32F405RG (MatekF405, OmnibusF4, etc.) is the lowest-end MCU that
runs full ArduPilot vehicles — Copter at 400Hz, Plane, Rover. The RP2350B
**must be at least as fast as the F405 to be a viable flight controller**.
If it can't match the F405, it's limited to AP_Periph only.

## Hardware Specs from Datasheets

| Feature | STM32F405RG | RP2350B |
|---|---|---|
| **Core** | Cortex-M4F @ 168MHz | 2× Cortex-M33 @ 150MHz |
| **Pipeline** | 3-stage, single-issue | 3-stage, single-issue |
| **DMIPS** | 210 (1.25 DMIPS/MHz) per DS8626 | ~225 est. (1.5 DMIPS/MHz) |
| **FPU** | Single-precision | Single-precision |
| **Flash** | 1MB internal, 128-bit bus | None internal; 16MB external QSPI |
| **Flash accelerator** | ART (prefetch + branch cache) | 16KB XIP cache (2-way, 1-cycle hit) |
| **Flash wait states** | 5 WS @ 168MHz (hidden by ART) | N/A — XIP cache or SRAM |
| **SRAM total** | 192KB (112+16+64 CCM) + 4KB backup | 520KB (8×64KB + 2×4KB) |
| **Code-executable SRAM** | 128KB (SRAM1+SRAM2 only) | 520KB (all banks) |
| **Data-only SRAM** | 64KB CCM (D-bus only, no I-bus) | None — all SRAM is uniform |
| **Cores** | 1 | 2 (SMP) |
| **DMA** | 16 streams (2 controllers) | 16 channels |
| **CAN** | 2× bxCAN | None (PIO or external) |
| **UART** | 4 USART + 2 UART | 2 HW + PIO UARTs |

### Key Architectural Difference

The F405 executes code from **1MB internal flash** through a 128-bit bus with
the ART (Adaptive Real-Time) accelerator. The RP2350B executes code from
**external QSPI flash** through a 16KB XIP cache. This is the fundamental
difference that determines relative performance.

## DMIPS: RP2350 Wins on Paper

This is surprising but confirmed by the datasheets:

| Metric | STM32F405 | RP2350B | Advantage |
|---|---|---|---|
| Clock | 168 MHz | 150 MHz | F405 +12% |
| DMIPS/MHz | 1.25 (per DS8626) | ~1.5 (ARM M33 spec) | **RP2350 +20%** |
| Total DMIPS | 210 | ~225 | **RP2350 +7%** |
| Cores | 1 | 2 | **RP2350 2×** |

The Cortex-M33 achieves higher DMIPS/MHz than the M4 despite both being
3-stage, single-issue pipelines. The M33 benefits from:
- Improved branch prediction
- More efficient instruction timing (fewer multi-cycle instructions)
- DSP and SIMD improvements over M4

**On raw compute alone, the RP2350 at 150MHz matches or slightly exceeds
the F405 at 168MHz.** The question is whether the flash access model
preserves this advantage.

## Flash Execution Models

### STM32F405: ART Accelerator + Internal Flash

From DS8626 §2.2.2:
> "The ART Accelerator implements an instruction prefetch queue and branch
> cache, which increases program execution speed from the 128-bit Flash
> memory. Based on CoreMark benchmark, the performance achieved thanks to
> the ART accelerator is equivalent to 0 wait state program execution from
> Flash memory at a CPU frequency up to 168 MHz."

The ART works as follows:
- **128-bit flash bus**: Each fetch reads 128 bits = 16 bytes = ~4 Thumb-2
  instructions. Only 1 fetch needed per 4 sequential instructions.
- **Prefetch queue**: While the CPU executes instructions from the current
  128-bit line, the next sequential line is prefetched. For sequential code,
  this completely hides the 5 wait-state latency.
- **Branch cache**: Remembers recent branch targets to avoid refetch penalty
  on loops and repeated branches.

**When ART works well (sequential code, tight loops):** Effectively 0 wait-
state, full 210 DMIPS performance.

**When ART struggles (branches to new code, function calls across the
codebase, virtual dispatch):** The prefetch queue is invalidated and a new
flash fetch is needed. Each miss costs **5 wait states × 1 CPU cycle =
5 extra cycles**. However, the 128-bit bus means the refetch loads 4
instructions, so the penalty is amortized for sequential code after the
branch.

**Effective ART miss cost:**
```
Branch to uncached address:
  Flash fetch: 1 + 5 wait states = 6 cycles for 128 bits (4 instructions)
  Cost per instruction: 6/4 = 1.5 cycles (sequential after branch)
  First instruction at branch target: 6 cycles
```

### RP2350B: XIP Cache + External QSPI

From RP2350 datasheet §4.4.1:
> "The cache is 16 kB, two-way set-associative, 1 cycle hit."

The XIP cache works as follows:
- **16KB cache, 8-byte lines, 2048 entries**: This is a true cache, not a
  prefetch buffer. Both sequential and non-sequential code benefit equally
  from cache hits.
- **1-cycle hit**: Cached code runs at near-full CPU speed. The M33 pipeline
  can sustain ~1 IPC for cached sequential code.
- **Miss penalty**: ~26 SCK cycles (continuous read) at 75MHz = ~347ns =
  ~52 CPU cycles at 150MHz. Each miss fetches 8 bytes (2 instructions).

**When XIP cache works well (hot code that fits in 16KB):** 1-cycle hit,
pipeline hides the latency, near-full ~225 DMIPS performance.

**When XIP cache struggles (code path > 16KB, cold code):** Each miss costs
~52 CPU cycles for 2 instructions = ~26 cycles per instruction. This is
severe, but the 16KB cache covers much more code than the ART.

## Head-to-Head: Code Access Efficiency

### Cache/Accelerator Size Comparison

| Feature | F405 ART | RP2350 XIP Cache |
|---|---|---|
| **Type** | Prefetch buffer + branch cache | True set-associative cache |
| **Effective capacity** | Small (prefetch queue + branch cache) | **16KB** (2048 × 8-byte lines) |
| **Fetch granularity** | 128 bits = 16 bytes per fetch | 8 bytes per cache line |
| **Hit behavior** | Sequential: 0WS. Branch: depends on cache | **All access patterns: 1-cycle** |
| **Miss cost** | 6 cycles (5 WS), serves 4 instructions | ~52 cycles, serves 2 instructions |
| **Miss cost per instruction** | ~1.5 cycles (sequential) to 6 cycles (branch target) | ~26 cycles |

### Why the RP2350's Larger Cache Matters

ArduPilot's main control loop at 400Hz touches roughly 37KB of code per
iteration (measured from MatekF405 ArduCopter build — see below).

The F405 ART can only prefetch one 128-bit line ahead. For a function call
to a new address, the ART must fetch from flash (6 cycles). The branch cache
helps with loops, but ArduPilot's complex call graph (virtual functions,
library calls across modules) means frequent ART misses.

The RP2350's 16KB XIP cache holds the most recently accessed 16KB of code.
For ArduPilot's hot path, this covers a large fraction of the frequently-
called code. Once the cache is warm, function calls to previously-visited
code hit the cache regardless of access pattern.

**Estimated effective hit rates for ArduPilot control loop:**

| Code section | F405 ART effective | RP2350 XIP cache |
|---|---|---|
| Tight inner loop (PID, filter) | ~100% (prefetch hides WS) | ~100% (fits in cache) |
| Attitude controller | ~90-95% (mostly sequential) | ~99%+ (fits entirely in cache) |
| EKF update (~15KB code) | ~70-80% (many branches, virtual calls) | ~90-95% (mostly fits in cache) |
| Full control loop (~25KB) | ~75-85% overall | ~85-95% (most fits, some evictions) |

### Effective Throughput Comparison

Using the estimated hit rates:

**F405 at 85% effective hit rate (control loop average):**
```
Avg cycles/insn = 0.85 × 1 + 0.15 × 3 = 1.30 cycles
  (miss cost ~3 cycles avg, accounting for 128-bit line amortization)
Effective MIPS = 168 / 1.30 ≈ 129 MIPS
```

**RP2350 at 93% XIP cache hit rate (control loop average):**
```
Avg cycles/insn = 0.93 × 1 + 0.07 × 26 = 2.75 cycles
Effective MIPS = 150 / 2.75 ≈ 55 MIPS  ← worse than F405!
```

This shows that **pure XIP without SRAM placement is risky** — the high miss
penalty (26 cycles/insn) means even a 7% miss rate cuts throughput
drastically. The F405's 128-bit bus with low miss penalty (3 cycles avg) is
much more forgiving.

**RP2350 at 98% XIP cache hit rate (tight hot path, ~10KB):**
```
Avg cycles/insn = 0.98 × 1 + 0.02 × 26 = 1.52 cycles
Effective MIPS = 150 / 1.52 ≈ 99 MIPS  ← approaching F405
```

**RP2350 with SRAM placement (hot path in SRAM, 0 wait-state):**
```
Avg cycles/insn = 1.0 (guaranteed)
Effective MIPS = 150 MIPS  ← exceeds F405's effective throughput
DMIPS ≈ 225 (1.5 DMIPS/MHz × 150MHz)  ← exceeds F405's 210 DMIPS
```

### What H750 Actually Puts in RAM (Reference)

The STM32H750 (`common_extf_h750.ld`) places **334KB** of code+rodata
in RAM — far more than just "hot functions." This is the proven approach
for XIP-based ArduPilot, measured from the RADIX2HD ArduCopter build:

| RAM Region | Size | Contents |
|---|---|---|
| **ITCM** (`.fastramfunc`) | **45.7KB** | ChibiOS kernel, Semaphores, AP_Math, vector3, matrix3, **ALL of libm**, libgcc, memcpy |
| **AXI** (`.ramfunc`) | **281.5KB** | Filters, SPI/I2C/UART drivers, Scheduler, RingBuffer, **ALL of EKF3** (text+rodata), AHRS, InertialSensor (ALL backends), Compass_Backend, RC Protocol/CRSF, Motors, AttitudeControl, PID, vector2, quaternion, polygon, flash ops, CRC |
| **DTCM** (`.ramdata`) | **6.2KB** | ChibiOS rodata, math lookup tables, scheduler task tables, vehicle `.rodata` |
| **DTCM** (vectors) | **0.7KB** | Interrupt vector table (must be in RAM for flash safety) |
| **Total** | **334KB** | |

Critical things the linker script captures beyond "hot functions":
- **`.rodata*`** alongside `.text*` — vtables, const data, lookup tables
- **ALL of libm** — every trig/math function, not just the hot ones
- **ALL of libgcc** — soft-float division, 64-bit ops
- **ALL EKF3 object files** — not just the predict path, ALL code+rodata
- **ALL InertialSensor backends** — including init/calibration code
- **Vector table** — interrupt dispatch must not depend on XIP
- **Scheduler task tables** — const arrays read every loop iteration

### Why More Than Just ".text Hot Functions" Matters

The initial analysis measured only 37KB of hot `.text` symbols (the
functions called at 400Hz). This missed several critical categories:

**1. Vtables (C++ virtual function dispatch tables):**
Every virtual function call loads a function pointer from a vtable
(`.rodata`). If the vtable is in XIP flash and its cache line was
evicted, each virtual call costs ~52 extra cycles. ArduPilot uses
virtual dispatch pervasively — HAL drivers, sensor backends, flight
modes, scheduler dispatch. Hot-path vtables alone are ~5KB; all
vtables are ~28KB (MatekF405 build, 181 vtables).

**2. Module `.rodata` (const data co-located with code):**
The H750 linker captures `*(.text* .rodata*)` for each module — not
just code but all const data in those translation units. This includes
lookup tables, string literals, parameter metadata. Total for modules
placed in RAM: ~30KB of `.rodata`.

**3. Math libraries (libm + libgcc):**
libm functions (sinf, cosf, atan2f, sqrtf, etc.) and libgcc
(soft-float division, 64-bit arithmetic) are called from every module.
A cache miss on `sinf` in the middle of CovariancePrediction stalls
the entire computation. The H750 puts ALL of libm in ITCM: ~14KB.
libgcc adds ~10KB. AP_Math/vector3/matrix3 add ~22KB.

**4. Template instantiations (W symbols):**
C++ templates like `Vector3<float>`, `Matrix3<float>`, `QuaternionT`,
`LowPassFilter<Vector3<float>>` generate weak symbols scattered across
many object files. These total ~125KB across the build and are called
from everywhere in the hot path.

**5. Vector table:**
The ARM interrupt vector table (~0.7KB) must be in SRAM — if an
interrupt fires during a flash erase/write, the vector must be
readable. Also, vectors in SRAM mean faster interrupt entry (no XIP
cache miss on exception handler lookup).

### Measured Hot Path Code Sizes (MatekF405 ArduCopter Build)

Actual compiled `.text` sizes from `arm-none-eabi-nm` on the MatekF405
ELF (877KB text). These are the 400Hz function bodies only — the full
SRAM budget must also include the rodata, math, and vtable categories
above.

**400Hz Rate Controller Inner Loop:** ~5.4KB
**400Hz EKF Predict Path:** ~22.7KB (dominated by
`CovariancePrediction` at 17.9KB)
**IMU Sample Pipeline:** ~2.7KB
**Hot math/filter functions:** ~6KB

**Total 400Hz `.text` only: ~37KB** (but this is NOT the full SRAM need)

### The CovariancePrediction Problem

`CovariancePrediction()` is 17.9KB — a single function called every
iteration at 400Hz. The XIP cache is only 16KB. This makes it a **cache
wrecker**: each call executes sequentially through 17.9KB, evicting all
other cached code. After it returns, every subsequent function (attitude
control, motor output, IMU read) must reload from QSPI flash.

**CovariancePrediction must be in SRAM.** There is no workaround — the
function exceeds the entire XIP cache capacity.

### SRAM Placement Tiers

Based on the H750 linker script and measured sizes:

| Tier | Contents | Size | Cumulative |
|---|---|---|---|
| **1. Mandatory** | Vector table, flash write routines | ~2KB | 2KB |
| **2. Cache wrecker** | EKF predict path (CovariancePrediction + inner loop) | ~23KB | 25KB |
| **3. Math foundation** | ALL libm, libgcc, AP_Math, vector3, matrix3, memcpy | ~48KB | 73KB |
| **4. Vtables + rodata** | Hot-path vtables (~5KB), scheduler tables (~3KB), module rodata (~30KB) | ~38KB | 111KB |
| **5. Complete hot path** | AttitudeControl+PID, Motors, IMU (all backends), Filters, Scheduler | ~113KB | 224KB |
| **6. Full H750 equivalent** | Remaining EKF3, AHRS, HAL drivers, RC protocol | ~168KB | 392KB |

### SRAM Budget Scenarios

| Scenario | Code+rodata in SRAM | Remaining for data | vs F405 data |
|---|---|---|---|
| Tier 1-3 (mandatory + math) | ~73KB | ~447KB | 3.5× |
| Tier 1-4 (+ vtables + rodata) | ~111KB | ~409KB | 3.2× |
| **Tier 1-5 (complete hot path)** | **~224KB** | **~296KB** | **2.4×** |
| Tier 1-6 (H750 equivalent) | ~392KB | ~128KB | 1.0× |

**Recommended: Tier 1-5 (~224KB).** All 400Hz code, math, vtables, and
associated rodata in SRAM. Leaves 296KB for data — 2.4× the F405's
122KB. EKF fusion functions (10-50Hz) and cold code remain in XIP.

### The SRAM Placement Argument

With ~224KB of code+rodata in SRAM (Tier 1-5):

| Configuration | Effective MIPS | vs F405 |
|---|---|---|
| F405 (ART, internal flash) | ~129 MIPS (mixed) | Baseline |
| RP2350 pure XIP (93% hit) | ~55 MIPS | **0.4× — not viable** |
| RP2350 pure XIP (98% hit) | ~99 MIPS | 0.77× — marginal |
| RP2350 SRAM hot path | ~150 MIPS | **1.16× — exceeds F405** |
| RP2350 SRAM + 200MHz | ~200 MIPS | **1.55× — comfortably exceeds** |
| RP2350 SRAM + 200MHz + Core 1 I/O | ~200 MIPS (Core 0 only) | **1.55× + I/O offloaded** |

## The Dual-Core Advantage

The F405 is single-core. **Every** task shares the 168MHz CPU:
- Control loop (EKF, attitude, PID)
- Sensor I/O (SPI reads, UART parsing)
- Logging (SD card writes via SPI/SDIO)
- MAVLink (telemetry formatting, GCS responses)
- GPS parsing
- RC input processing

On the MatekF405, these tasks are time-sliced by the ChibiOS scheduler.
The control loop gets priority, but I/O tasks steal cycles, reducing the
effective CPU time available for the control loop.

The RP2350's dual-core changes this completely:
- **Core 0**: Control loop (EKF, attitude, PID, motor output) — dedicated
- **Core 1**: All I/O (logging, MAVLink, GPS, UART, USB)

This means Core 0 on RP2350 has nearly 100% of its cycles available for the
control loop, while the F405 must share with I/O. In practice, the F405's
control loop gets perhaps 60-70% of CPU time at 400Hz. This means:

| Metric | F405 (1 core, shared) | RP2350 (Core 0, dedicated) |
|---|---|---|
| Total CPU MIPS | ~129 MIPS | ~150-200 MIPS |
| Control loop share | ~60-70% | ~95%+ |
| **Effective control MIPS** | **~77-90 MIPS** | **~143-190 MIPS** |

**With dual-core, the RP2350 delivers ~1.7-2.5× the effective control loop
throughput compared to F405**, even before overclocking.

## Memory Comparison

| Resource | F405 | RP2350B | Advantage |
|---|---|---|---|
| Total SRAM | 192KB + 4KB backup | 520KB | **RP2350 2.7×** |
| Code-executable SRAM | 128KB (SRAM1+2 only) | 520KB (all banks) | **RP2350 4.1×** |
| Data-only fast SRAM | 64KB CCM | None needed | F405 has dedicated fast data region |
| Flash for code | 1MB internal | 16MB external | RP2350 16× more (but external) |

The F405's 64KB CCM is data-only (connected via D-bus only, per block
diagram Figure 5 — no I-bus connection). Code cannot execute from CCM.
ArduPilot uses CCM for performance-critical data (stacks, DMA buffers).

The RP2350 has **520KB of uniform SRAM** — all banks can hold both code and
data at zero wait-states. With 224KB for code+rodata (recommended Tier 1-5),
296KB remains for data — still 2.4× the F405's total code-executable SRAM.

### Measured F405 RAM Usage (MatekF405 ArduCopter)

| Section | Size | Notes |
|---|---|---|
| .data | 3KB | Initialized globals |
| .bss | 72KB | Zero-initialized globals |
| .heap | 48KB | Dynamic allocation pool |
| **Total used** | **122KB** | Of 128KB available |
| **Free** | **8.5KB** | Extremely tight |

The MatekF405 runs ArduCopter in 128KB with only 8.5KB free. After placing
224KB of code+rodata in SRAM (Tier 1-5), the RP2350 has **296KB for data —
2.4× the F405's total code-executable SRAM.**

### Measured H750 RAM Usage (RADIX2HD ArduCopter)

For comparison, the H750 places 334KB of code+rodata across ITCM+AXI+DTCM,
and uses 262KB of AXI SRAM for data (.data 3KB + .bss 104KB + .heap 155KB).
The H750 has ~1MB of SRAM total, so this is comfortable. The RP2350's 520KB
is more constrained but the Tier 1-5 approach (224KB code, 296KB data) fits
with margin.

## Viability Assessment

### What Must Be True for RP2350 ≥ F405

1. **Hot code, math libraries, vtables, and rodata must be in SRAM** —
   not just "hot functions" but the complete execution context including
   virtual dispatch tables, const data, and math foundations
2. **SRAM placement of ~224KB** (Tier 1-5) is the recommended approach —
   this matches the H750's proven strategy, scaled for RP2350's resources
3. **Dual-core I/O offloading** provides the decisive advantage over F405

### Scenario Analysis

| Scenario | RP2350 vs F405 | Viable? |
|---|---|---|
| Pure XIP, no optimization, 150MHz | ~0.4-0.8× | **No** — miss penalty too high |
| Minimal SRAM (Tier 1-3, ~73KB), 150MHz | ~0.8-1.0× | **Marginal** — vtable/rodata misses |
| Full hot path SRAM (Tier 1-5, ~224KB), 150MHz | ~1.1-1.2× | **Yes** — matches/exceeds F405 |
| SRAM + dual-core, 150MHz | ~1.7-2.0× | **Yes** — comfortably exceeds |
| SRAM + dual-core, 200MHz | ~2.0-2.5× | **Yes** — significantly exceeds |

### The EKF: Measured Reality

`NavEKF3_core` total compiled code is **82KB** (text only) plus **4KB**
of associated `.rodata` (const data, lookup tables). On the H750, the
linker places ALL of EKF3 (text+rodata, ~133KB for the RADIX2HD build)
in AXI SRAM — not just the predict path.

For the RP2350 with Tier 1-5, the EKF predict path (~23KB) goes in SRAM
while fusion functions (`FuseMagnetometer` 10KB, `FuseVelPosNED` 3.5KB,
etc.) run at 10-50Hz from XIP cache. Each fusion function fits in the
16KB cache individually, and the lower rate provides time budget for
cache misses. This is a reasonable split — but requires empirical
validation that fusion function cache misses don't cause timing jitter.

## Conclusions

1. **Raw DMIPS: RP2350 ≥ F405.** The M33 at 150MHz achieves ~225 DMIPS vs
   the M4 at 168MHz achieving 210 DMIPS. Clock speed deficit is overcome
   by higher IPC.

2. **Flash access: F405 has a better model for unoptimized code.** The ART
   accelerator with internal flash is more forgiving of cache misses (low
   penalty, wide bus). Pure XIP on RP2350 without SRAM placement is not
   viable for the control loop.

3. **SRAM placement requires ~224KB, not 37KB.** The initial estimate of
   37KB counted only hot `.text` symbols. The H750 linker script reveals
   that effective SRAM placement must also include: vtables (virtual
   dispatch tables), module `.rodata` (const data), ALL of libm/libgcc
   (math foundations), template instantiations, vector table, and
   scheduler task tables. The recommended Tier 1-5 approach places
   ~224KB in SRAM, leaving 296KB for data (2.4× the F405's 128KB).

4. **Dual-core is the decisive advantage.** The F405 must share its single
   core between control loop and I/O. The RP2350 dedicates Core 0 to the
   control loop, providing ~1.7-2.5× the effective control throughput.

5. **SRAM placement is NOT optional for viability.** This elevates Step 3b
   (XIP Flash and Memory Model) from "optimization" to "hard requirement
   for matching F405 performance."

6. **At 200MHz + SRAM + dual-core, the RP2350 significantly exceeds F405**
   and approaches F7-class performance for the control loop, while costing
   ~$0.80 vs ~$6-10.

7. **The 520KB SRAM is adequate but not generous.** With 224KB for code
   and 296KB for data, there is 2.4× the F405's data capacity. This is
   sufficient but leaves less margin than the original 37KB estimate
   suggested. The RP2350 cannot afford the H750's luxury of placing 334KB
   of code in RAM — that would leave only 128KB for data (matching the
   F405 with zero margin). The tiered approach is essential.

### Bottom Line

**The RP2350B is viable as a flight controller — but requires ~224KB of
SRAM for code+rodata placement.** This is much more than the naive 37KB
"hot function" estimate because SRAM must also hold vtables, module
rodata, math libraries, and the vector table — all things that cause
costly XIP cache misses when accessed via virtual dispatch, const data
lookups, or interrupt entry. With 224KB for code and 296KB for data
(2.4× F405), plus dual-core I/O offloading, the RP2350 comfortably
exceeds F405 performance.
