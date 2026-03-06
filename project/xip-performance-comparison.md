# XIP Performance Comparison: STM32H750 vs RP2350B

**Sources:** RP2350 Datasheet (RP-008373-DS-2, build-date 2025-07-29);
STM32H750xB Datasheet (DS12556 Rev 6, Feb 2021); ArduPilot mcuconf.h
(STM32H7 PLL2 configuration).

## Why This Matters

The STM32H750 is the existing ArduPilot board that's closest to the RP2350B's
situation: both execute most code from external QSPI flash via XIP. The H750
is described as "borderline" for ArduPilot Copter at 400Hz. If the RP2350B is
significantly worse, it may not achieve acceptable loop rates.

## Hardware Comparison

| Feature | STM32H750 | RP2350B |
|---|---|---|
| **CPU** | Cortex-M7 @ 400MHz (ArduPilot; chip max 480MHz) | Cortex-M33 @ 150MHz |
| **Pipeline** | 6-stage, superscalar dual-issue | 3-stage, single-issue |
| **IPC** | ~2.14 DMIPS/MHz (dual-issue) | ~1.5 DMIPS/MHz (single-issue, DSP) |
| **Effective MIPS** | ~856 DMIPS @ 400MHz (per DS12556) | ~225 DMIPS @ 150MHz (est.) |
| **Internal flash** | 128KB (bootloader + vectors + critical code) | None |
| **SRAM** | 1MB total (64KB ITCM + 128KB DTCM + 512KB AXI-SRAM + 352KB SRAM1-4 + 4KB backup) | 520KB (10 banks: 8×64KB + 2×4KB) |
| **TCM** | 64KB ITCM + 128KB DTCM (zero wait-state) | None (all SRAM is uniform, single-cycle) |
| **L1 I-cache** | 16KB, 32B lines, 2-way (core-integrated) | None |
| **L1 D-cache** | 16KB, 32B lines, 2-way (core-integrated) | None |
| **XIP cache** | None (L1 cache serves this role) | 16KB, 2-way set-associative, 1-cycle hit |
| **XIP cache hit latency** | 0 cycles (L1 cache is core-integrated) | 1 system clock cycle |
| **Cores** | 1 | 2 (SMP) |

## QSPI Flash Interface

| Feature | STM32H750 | RP2350B |
|---|---|---|
| **QSPI peripheral** | QUADSPI (memory-mapped mode) | QMI (QSPI Memory Interface) |
| **QSPI kernel clock** | 200MHz (PLL2_R) | Up to clk_sys (150MHz) |
| **QSPI SCK max** | 133MHz SDR (per DS12556 Table 82) | Up to clk_sys (per RP2350 datasheet §4.4.2) |
| **QSPI SCK (ArduPilot)** | 100MHz (PLL2_R ÷2) | 75MHz (clk_sys ÷2, typical) |
| **Flash data bus** | Quad (4-bit) | Quad (4-bit) |
| **Peak throughput** | 400 Mbit/s = 50 MB/s @ 100MHz | 300 Mbit/s = 37.5 MB/s @ 75MHz |
| **Typical flash chip** | W25Q128 (133MHz max) | W25Q128FV (133MHz max) |
| **Memory-mapped base** | 0x90000000 | 0x10000000 |

### QSPI Clock Derivation

**STM32H750** (8MHz HSE crystal, typical ArduPilot config):
```
PLL2 input  = 8MHz / DIVM(1) = 8MHz
PLL2 VCO    = 8MHz × DIVN(75) = 600MHz
PLL2_R      = 600MHz / DIVR(3) = 200MHz (QSPI kernel clock)
QUADSPI SCK = 200MHz / (prescaler+1) = 100MHz (prescaler=1)
```

**RP2350B** (12MHz crystal, Pico 2 default → PLL to 150MHz):
```
System clock = 150MHz
QMI SCK max  = clk_sys (per datasheet §4.4.2: "SCK speeds as high as clk_sys")
QMI SCK typ  = clk_sys / clkdiv = 150MHz / 2 = 75MHz (clkdiv=2 is typical)
  Note: Flash chip limits apply. W25Q128 supports up to 133MHz for QPI reads.
  clkdiv=1 (150MHz SCK) may work if the flash chip and board layout support it.
```

## Cache Architecture Comparison

### STM32H750

The H750 Cortex-M7 has **core-integrated L1 caches**:
- 16KB I-cache: 32-byte lines, 2-way set-associative, **0-cycle hit** (part
  of the pipeline — instruction fetch sees cached data with no penalty)
- 16KB D-cache: 32-byte lines, 2-way set-associative, 0-cycle hit
- 512 cache lines per cache
- Line fill fetches 32 bytes (8 Thumb-2 instructions) per miss
- Plus **64KB ITCM**: zero wait-state code region, bypasses cache entirely

For hot loops, the H750 runs at full 400MHz from L1 cache or ITCM with
zero penalty. Only cache misses (cold code, branches to new pages) incur
the QSPI fetch latency.

### RP2350B

The RP2350B has a **dedicated XIP cache controller** (not a CPU L1 cache):
- 16KB, 2-way set-associative, **1-cycle hit** (per RP2350 datasheet)
- Cache lines are 8 bytes (2 Thumb-2 instructions per line)
- 2048 cache lines total (more lines than H750, but smaller)
- No ITCM — SRAM is the only zero-wait-state code region
- Cortex-M33 has a prefetch buffer that can partially hide cache latency
  for sequential code

The **1-cycle hit latency** is the key datapoint. This means for sequential
code that stays in cache, the M33 pipeline can sustain close to 1 instruction
per cycle — nearly matching SRAM execution speed for hot loops. The penalty
is only paid on cache misses.

## Cache Miss Latency Analysis

### STM32H750 Cache Miss

```
L1 I-cache miss → QUADSPI fetch from external flash

QUADSPI quad-read sequence (0xEB command):
  Command:  8 SCK cycles  (instruction byte, quad mode for addr/data)
  Address:  8 SCK cycles  (24-bit addr in quad mode = 6 cycles + 2 mode)
  Dummy:    4-6 SCK cycles (performance mode / continuous read)
  Data:     64 SCK cycles  (32 bytes cache line × 8 bits / 4 lines)
  ──────────────────────────
  Total:    ~84-86 SCK cycles

At 100MHz SCK: ~860ns per 32-byte cache line miss
Per 32-bit instruction (avg): ~860ns / 8 = ~107ns
In CPU cycles (400MHz): ~344 CPU cycles lost per cache miss
```

But H750 has **three layers of mitigation**:
1. **128KB internal flash**: Bootloader, vectors, and small critical routines
   execute from internal flash at low wait-states
2. **ITCM (64KB)**: Hot code copied here runs at zero wait-states, bypassing
   QSPI entirely. ArduPilot places scheduler, attitude controller, and EKF
   critical paths here.
3. **L1 I-cache (16KB, 32B lines, 2-way)**: Core-integrated, zero-penalty
   cache hits. With 512 cache lines and 32B per line, this covers ~16KB of
   hot code with zero-cycle hits. Sequential code benefits from line fill
   (8 instructions per line).

**Effective H750 performance:** For hot code in ITCM, execution is at full
400MHz. For cached XIP code, hits are zero-cycle. Only cold/random code paths
pay the ~860ns miss penalty.

### RP2350B Cache Miss

```
XIP cache miss → QMI fetch from external flash

QMI quad-read sequence (0xEB continuous read):
  Command:  0 cycles  (continuous read mode, no repeated command)
  Address:  6 SCK cycles  (24-bit addr in quad mode)
  Dummy:    4 SCK cycles  (mode + dummy for W25Q128)
  Data:     16 SCK cycles  (8 bytes cache line × 8 bits / 4 lines)
  ──────────────────────────
  Total:    ~26 SCK cycles (continuous read mode)
  Cold:     ~34 SCK cycles (first access, includes 8-cycle command)

At 75MHz SCK: ~347ns per 8-byte cache line miss (continuous)
              ~453ns per 8-byte cache line miss (cold first access)
Per 32-bit instruction (avg): ~347ns / 2 = ~173ns
In CPU cycles (150MHz): ~52 cycles lost per cache miss
```

RP2350B mitigation layers:
1. **No internal flash**: Zero benefit — all code is XIP
2. **No TCM**: No dedicated zero-wait-state code memory. SRAM is uniform.
3. **XIP cache (16KB, 8B lines, 1-cycle hit)**: 2048 cache lines. The
   1-cycle hit means hot loops run at near-full 150MHz throughput. The main
   cost is **miss frequency** — 8B lines mean the cache covers only 2
   instructions per line, so branches and function calls cause more misses.
4. **SRAM code placement**: Functions marked `__not_in_flash_func()` are
   copied to SRAM at boot and execute at zero wait-states. Eliminates miss
   risk entirely for critical code.

## Effective Execution Speed Comparison

### Cached XIP Code (hot path in cache, no misses)

| Metric | STM32H750 | RP2350B | Ratio |
|---|---|---|---|
| CPU clock | 400MHz | 150MHz | 2.67x |
| Cache hit penalty | 0 cycles | 1 cycle | — |
| Effective instruction rate | ~856 DMIPS (2.14/MHz × 400MHz) | ~225 DMIPS (est. ~1.5/MHz × 150MHz) | **~3.8x** |
| Per-instruction time | ~1.17ns | ~4.44ns | ~3.8x slower |

With 1-cycle hit latency, the Cortex-M33 pipeline and prefetch buffer can
sustain close to **1 instruction per cycle** for sequential code from XIP
cache. This means cached XIP code on RP2350 runs at nearly the same speed as
SRAM-resident code. The performance gap vs H750 is primarily the clock speed
and IPC difference (~3.8x), not the cache penalty.

This is a much better picture than a 2-cycle penalty would give — the XIP
cache essentially makes flash-resident code competitive with SRAM for hot
loops that fit in 16KB of cache.

### SRAM-Resident Code (hot path in SRAM)

| Metric | STM32H750 (ITCM) | RP2350B (SRAM) | Ratio |
|---|---|---|---|
| CPU clock | 400MHz | 150MHz | 2.67x |
| Wait states | 0 | 0 | Same |
| Effective instruction rate | ~856 DMIPS | ~225 DMIPS | **~3.8x** |
| Per-instruction time | ~1.17ns | ~4.44ns | ~3.8x slower |

With SRAM execution, the gap is the same **~3.8x** as cached XIP, because the
1-cycle XIP hit is mostly hidden by the pipeline. SRAM placement still
provides value for **guaranteed** timing (no cache miss risk) and for
**flash write safety** (code in SRAM keeps running during flash erase/write).

### Code with Cache Misses (cold paths, large functions)

| Metric | STM32H750 | RP2350B | Ratio |
|---|---|---|---|
| Miss penalty (time) | ~860ns | ~347ns | H750 worse per miss |
| Miss penalty (CPU cycles) | ~344 cycles | ~52 cycles | H750 loses more cycles |
| Data per miss | 32 bytes (~8 insns) | 8 bytes (~2 insns) | H750 more efficient |
| Effective miss cost per insn | ~107ns | ~173ns | RP2350 ~1.6x worse |
| Amortized miss rate | Lower (32B lines) | Higher (8B lines) | RP2350 has ~4x more misses |

For cold code paths (initialization, rare branches), RP2350B's smaller cache
lines mean ~4x more cache misses for the same code sequence. Each miss is
cheaper in absolute time (347ns vs 860ns), but there are more of them. The
net effect is that **cold code is ~1.6x slower per instruction** on RP2350
compared to H750, which is better than the ~3.8x clock speed difference would
suggest (because H750's miss penalty is proportionally more expensive).

## Loop Rate Feasibility

### ArduPilot Copter 400Hz Budget

At 400Hz, each loop iteration has **2500µs**. Typical breakdown:

| Phase | STM32F405 (168MHz, internal flash) | STM32H750 (400MHz, XIP+ITCM) |
|---|---|---|
| IMU read | ~50µs | ~20µs |
| EKF update | ~800µs | ~300µs |
| Attitude control | ~200µs | ~80µs |
| Motor output | ~50µs | ~20µs |
| Logging + MAVLink | ~400µs | ~150µs |
| **Total** | **~1500µs** | **~570µs** |
| **Headroom** | ~1000µs (40%) | ~1930µs (77%) |

### RP2350B Projection

**Scenario A: All code from XIP cache (no SRAM placement) @ 150MHz**

With 1-cycle cache hit, cached XIP code runs at ~150 MIPS for sequential
code. The RP2350 is ~1x F405 clock speed (150 vs 168MHz) but has lower IPC
for some operations. For hot loops that fit in the 16KB XIP cache, performance
should be comparable to the F405. The main cost is cache misses on code that
doesn't fit.

| Phase | RP2350B @ 150MHz XIP | Notes |
|---|---|---|
| IMU read | ~60µs | Hot loop, fits in cache |
| EKF update | ~1000-1200µs | Large code — cache misses on outer functions |
| Attitude control | ~250µs | Mostly fits in cache |
| Motor output (PIO) | ~30µs | PIO handles DShot, CPU just loads FIFO |
| Logging + MAVLink | ~500µs | Cold paths, many cache misses |
| **Total** | **~1840-2040µs** | **Fits in 2500µs at 400Hz** |
| **Headroom** | ~460-660µs (18-26%) | **Tight but possible** |

**Result: 400Hz Copter may be achievable from pure XIP at 150MHz** — but
it's tight. The EKF is the wildcard: if it causes too many cache misses
(the code is large and branches frequently), it could push over budget.

**Scenario B: Hot path in SRAM (~35KB), rest from XIP @ 150MHz**

Moving critical code (EKF inner loop, attitude controller, IMU read,
scheduler) to SRAM eliminates cache miss risk for the hot path:

| Phase | RP2350B SRAM+XIP | Notes |
|---|---|---|
| IMU read | ~50µs | SRAM (guaranteed zero wait-state) |
| EKF update | ~800-900µs | Core in SRAM, outer code from XIP cache |
| Attitude control | ~200µs | SRAM |
| Motor output (PIO) | ~30µs | PIO + SRAM FIFO load |
| Logging + MAVLink | ~500µs | XIP (not time-critical) |
| **Total** | **~1580-1680µs** | **Comfortable at 400Hz** |
| **Headroom** | ~820-920µs (33-37%) | **Good margin** |

**Result: 400Hz Copter comfortable with SRAM placement at 150MHz.**

**Scenario C: SRAM hot path + overclock to 200MHz + dual-core**

RP2350 commonly runs at 200MHz (well within safe overclocking range). With
Core 0 doing control + EKF, and Core 1 handling I/O:

| Phase | RP2350B @ 200MHz | Notes |
|---|---|---|
| IMU read | ~38µs | SRAM, 200MHz |
| EKF update | ~600-675µs | SRAM core path, 200MHz |
| Attitude control | ~150µs | SRAM, 200MHz |
| Motor output (PIO) | ~23µs | PIO + SRAM FIFO load |
| **Core 0 total** | **~811-886µs** | Control loop only |
| Logging + MAVLink | (Core 1) | Offloaded |
| **Headroom** | ~1614-1689µs (65-68%) | **Very comfortable** |

**Result: 400Hz Copter very comfortable at 200MHz with SRAM + dual-core.**

**Scenario D: Conservative — 200Hz Copter at 150MHz, no SRAM**

At 200Hz (5000µs budget), even Scenario A (pure XIP) works easily:

| Phase | RP2350B @ 150MHz XIP | Budget |
|---|---|---|
| Total (from Scenario A) | ~1840-2040µs | 5000µs available |
| **Headroom** | ~2960-3160µs (59-63%) | **Very comfortable** |

## SRAM Budget for Code Placement

SRAM code placement is **required** for flash write safety and **mandatory**
for F405-equivalent performance. Analysis of the H750 linker script
(`common_extf_h750.ld`) reveals that effective SRAM placement requires far
more than just "hot functions" — vtables, module rodata, math libraries, and
the vector table must also be in SRAM.

### What H750 Places in RAM (334KB total, RADIX2HD ArduCopter)

| Region | Size | Contents |
|---|---|---|
| ITCM `.fastramfunc` | 45.7KB | ChibiOS kernel, Semaphores, AP_Math, vector3, matrix3, ALL libm, libgcc, memcpy |
| AXI `.ramfunc` | 281.5KB | ALL EKF3 (text+rodata), AHRS, InertialSensor (all backends), Filters, AttitudeControl, PID, Motors, HAL drivers, RC Protocol, vector2, quaternion, polygon, flash ops, CRC |
| DTCM `.ramdata` | 6.2KB | Math lookup tables, scheduler task tables, ChibiOS rodata |
| DTCM vectors | 0.7KB | Interrupt vector table |

### Why More Than "Hot Functions" Must Be in SRAM

- **Vtables:** Every virtual function call loads a pointer from `.rodata`.
  A cache miss costs ~52 cycles per virtual dispatch. ArduPilot uses virtual
  dispatch pervasively (HAL drivers, sensor backends, flight modes).
  Hot-path vtables: ~5KB; all vtables: ~28KB (MatekF405).
- **Module `.rodata`:** The H750 linker captures `.text* .rodata*` together.
  Const data (lookup tables, parameter metadata, CRC tables) co-located
  with code totals ~30KB for modules placed in RAM.
- **Math libraries:** libm + libgcc + AP_Math/vector3/matrix3 total ~48KB.
  Called from every module — a cache miss on `sinf` inside
  CovariancePrediction stalls the entire computation.
- **Vector table:** ARM interrupt vectors (~0.7KB) must be in SRAM for
  flash safety and fast interrupt entry.
- **Template instantiations:** Vector3/Matrix3/Quaternion/Filter templates
  generate weak symbols scattered across object files (~125KB total).

### Tiered SRAM Placement for RP2350

| Tier | Contents | Size | Cumulative |
|---|---|---|---|
| 1. Mandatory | Vector table, flash write routines | ~2KB | 2KB |
| 2. Cache wrecker | EKF predict path (CovariancePrediction 17.9KB + inner loop) | ~23KB | 25KB |
| 3. Math foundation | ALL libm, libgcc, AP_Math, vector3, matrix3, memcpy | ~48KB | 73KB |
| 4. Vtables + rodata | Hot-path vtables, scheduler tables, module rodata | ~38KB | 111KB |
| 5. Complete hot path | AttitudeControl+PID, Motors, IMU (all), Filters, Scheduler | ~113KB | 224KB |
| 6. H750 equivalent | Remaining EKF3, AHRS, HAL drivers, RC protocol | ~168KB | 392KB |

### Recommended: Tier 1-5 (~224KB code+rodata in SRAM)

| Budget | Amount |
|---|---|
| Code+rodata in SRAM | ~224KB |
| Remaining for data | ~296KB |
| F405 data usage | 122KB (of 128KB available) |
| **RP2350 data headroom** | **2.4× the F405** |

This leaves EKF fusion functions (10-50Hz, ~35KB each fitting in 16KB cache)
and cold code (~1MB) running from XIP. The Tier 1-6 "H750-equivalent" would
use 392KB, leaving only 128KB for data — matching the F405 with zero margin.

`CovariancePrediction()` at 17.9KB exceeds the 16KB XIP cache and is the
non-negotiable minimum for SRAM placement. But stopping at just the "hot
functions" (37KB) misses vtable misses on every virtual call, rodata misses
on const data access, and math library misses inside hot computations.

## Key Differences from H750

| Aspect | H750 (works, borderline) | RP2350B | Impact |
|---|---|---|---|
| **CPU raw speed** | ~856 DMIPS | ~225 DMIPS (1 core) | ~3.8x slower |
| **Internal flash** | 128KB for boot+critical | None | All code via XIP or SRAM |
| **ITCM** | 64KB zero-wait code | None (use SRAM) | RP2350 must use SRAM for guaranteed timing |
| **L1 I-cache hit** | 0 cycles | 1 cycle (XIP cache) | Similar — M33 pipeline hides 1-cycle hit |
| **Cache line size** | 32 bytes | 8 bytes | RP2350 has ~4x more misses per code path |
| **Cache lines** | 512 | 2048 | RP2350 covers more unique addresses |
| **Dual core** | No | Yes | RP2350 can offload I/O to Core 1 |
| **FPU** | Single+Double precision | Single precision only | Same constraint as F405 |
| **Overclock** | 480MHz max per DS (20% gain) | 200MHz common (33% gain) | Overclock helps RP2350 more (%) |

### Why H750 is "borderline" despite being ~3.8x faster

The H750's 400MHz Cortex-M7 is ~3.8x faster than RP2350, yet is "borderline"
for 400Hz Copter. This is because:

1. **H750 runs ArduPilot with full features** (more processing per loop)
2. **Double-precision EKF** on H750 uses FP64 math (2-3x slower per operation
   than FP32). RP2350 uses single-precision EKF (`HAL_WITH_EKF_DOUBLE 0`),
   which is faster per operation.
3. **H750 is single-core** — all I/O (logging, MAVLink, GPS parsing) shares
   the CPU with the control loop. RP2350 can offload I/O to Core 1.
4. **H750's 32-byte cache lines** waste bandwidth fetching unused bytes at
   the end of functions. RP2350's 8-byte lines are more efficient per miss
   (less wasted data) though they miss more often.

This means the **effective performance gap for ArduPilot Copter is closer
to 2x than 3.8x** when accounting for feature set, FPU precision, and
dual-core benefits.

## Conclusions

1. **The 1-cycle XIP cache hit is the key enabler.** With 1-cycle hit, cached
   XIP code runs at near-full CPU speed (~150 MIPS), making pure XIP execution
   viable for many code paths. This is much better than a 2-cycle penalty
   would be.

2. **400Hz Copter appears feasible at 150MHz** from pure XIP cache, though
   tight (~20% headroom). With SRAM placement of ~35KB hot code, headroom
   improves to ~35%.

3. **SRAM placement is mandatory for F405-equivalent performance.** The F405
   comparison ([f405-performance-comparison.md](f405-performance-comparison.md))
   shows pure XIP with cache misses cannot match the F405's ART accelerator.
   SRAM placement is **required** for:
   - Flash write safety (code must run from SRAM during flash erase/write)
   - Guaranteed timing (no cache miss jitter in control loop)
   - Matching/exceeding F405 throughput for the control loop

4. **At 200MHz + dual-core, 400Hz Copter is very comfortable** (~65%
   headroom). This is the recommended production configuration.

5. **200Hz Copter at 150MHz requires no special optimization** — even pure
   XIP works with 60% headroom.

6. **400Hz Plane/Rover is easily achievable** at 150MHz — these vehicles
   have simpler control loops and lower EKF update rates.

7. **Step 3b (XIP memory model) is the critical path for viability:**
   - Flash write SRAM placement: mandatory (XIP safety)
   - Performance SRAM placement: mandatory (must match F405 throughput)
   - Performance baseline measurement: essential to validate these projections

## Recommendations for Implementation Plan

- **Default target: 400Hz Copter at 150MHz** with SRAM hot code (Step 5)
- **Stretch target: 400Hz Copter at 200MHz** with dual-core (Step 6f) for
  maximum headroom
- **SRAM placement macros must be in Step 3b** — mandatory for both flash
  write safety and F405-equivalent performance
- **Performance profiling infrastructure** early (GPIO toggle timing) to
  validate projections
- **Measure actual XIP cache hit rate** during flight loop to confirm the
  16KB cache covers the hot path adequately
