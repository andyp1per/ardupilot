# XIP Performance Comparison: STM32H750 vs RP2350B

## Why This Matters

The STM32H750 is the existing ArduPilot board that's closest to the RP2350B's
situation: both execute most code from external QSPI flash via XIP. The H750
is described as "borderline" for ArduPilot Copter at 400Hz. If the RP2350B is
significantly worse, it may not achieve acceptable loop rates.

## Hardware Comparison

| Feature | STM32H750 | RP2350B |
|---|---|---|
| **CPU** | Cortex-M7 @ 400MHz | Cortex-M33 @ 150MHz |
| **Pipeline** | 6-stage, superscalar dual-issue | 3-stage, single-issue |
| **IPC** | ~1.5-2.0 (dual-issue) | ~1.0 (single-issue) |
| **Effective MIPS** | ~600-800 DMIPS | ~150 DMIPS |
| **Internal flash** | 128KB (bootloader + vectors + critical code) | None |
| **SRAM** | 564KB total (DTCM 128KB + SRAM1-3 + AXI) | 520KB (10 uniform banks) |
| **TCM** | 64KB ITCM + 128KB DTCM (zero wait-state) | None (all SRAM is uniform) |
| **L1 I-cache** | 16KB, 32B lines, 4-way (core-integrated) | None |
| **L1 D-cache** | 16KB, 32B lines, 4-way (core-integrated) | None |
| **XIP cache** | None (L1 cache serves this role) | 16KB, 2-way set-associative, 1-cycle hit |
| **XIP cache hit latency** | 0 cycles (L1 cache is core-integrated) | 1 system clock cycle |
| **Cores** | 1 | 2 (SMP) |

## QSPI Flash Interface

| Feature | STM32H750 | RP2350B |
|---|---|---|
| **QSPI peripheral** | QUADSPI (memory-mapped mode) | QMI (QSPI Memory Interface) |
| **QSPI kernel clock** | 200MHz (PLL2_R) | 150MHz (system clock) |
| **QSPI SCK (typical)** | 100MHz (prescaler ÷2) | 75MHz (÷2 from sys clock) |
| **Flash data bus** | Quad (4-bit) | Quad (4-bit) |
| **Peak throughput** | 400 Mbit/s = 50 MB/s | 300 Mbit/s = 37.5 MB/s |
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
QMI clock    = sys_clk / clkdiv = 150MHz / 2 = 75MHz SCK
  (clkdiv=2 is typical; clkdiv=1 may work with fast flash chips)
```

## Cache Architecture Comparison

### STM32H750

The H750 Cortex-M7 has **core-integrated L1 caches**:
- 16KB I-cache: 32-byte lines, 4-way set-associative, **0-cycle hit** (part
  of the pipeline — instruction fetch sees cached data with no penalty)
- 16KB D-cache: 32-byte lines, 4-way set-associative, 0-cycle hit
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
In CPU cycles (400MHz): ~344 cycles lost per cache miss
```

But H750 has **three layers of mitigation**:
1. **128KB internal flash**: Bootloader, vectors, and small critical routines
   execute from internal flash at low wait-states
2. **ITCM (64KB)**: Hot code copied here runs at zero wait-states, bypassing
   QSPI entirely. ArduPilot places scheduler, attitude controller, and EKF
   critical paths here.
3. **L1 I-cache (16KB, 32B lines)**: Core-integrated, zero-penalty cache hits.
   With 512 cache lines and 32B per line, this covers ~16KB of hot code with
   zero-cycle hits. Sequential code benefits from line fill (8 instructions
   per line).

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
| Effective instruction rate | 400MHz × ~1.5 IPC = 600 MIPS | ~150 MIPS (pipeline hides 1-cycle hit) | **~4x** |
| Per-instruction time | ~1.67ns | ~6.67ns | 4x slower |

With 1-cycle hit latency, the Cortex-M33 pipeline and prefetch buffer can
sustain close to **1 instruction per cycle** for sequential code from XIP
cache. This means cached XIP code on RP2350 runs at nearly the same speed as
SRAM-resident code. The performance gap vs H750 is primarily the clock speed
and IPC difference (~4x), not the cache penalty.

This is a much better picture than a 2-cycle penalty would give — the XIP
cache essentially makes flash-resident code competitive with SRAM for hot
loops that fit in 16KB of cache.

### SRAM-Resident Code (hot path in SRAM)

| Metric | STM32H750 (ITCM) | RP2350B (SRAM) | Ratio |
|---|---|---|---|
| CPU clock | 400MHz | 150MHz | 2.67x |
| Wait states | 0 | 0 | Same |
| Effective instruction rate | ~600 MIPS | ~150 MIPS | **~4x** |
| Per-instruction time | ~1.67ns | ~6.67ns | 4x slower |

With SRAM execution, the gap is the same **~4x** as cached XIP, because the
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
compared to H750, which is better than the ~4x clock speed difference would
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

SRAM code placement is valuable for **guaranteed timing** (no cache miss risk)
and **required** for flash write safety (code must not be in XIP during flash
erase/write). The 1-cycle cache hit means it's less critical for pure
performance than originally feared.

| Code section | Size (est.) | Priority |
|---|---|---|
| Flash write routines | ~2KB | Critical (XIP safety during flash ops) |
| Scheduler fast path | ~2KB | High (guaranteed timing) |
| EKF3 core loop | ~15KB | High (eliminates cache miss risk) |
| Attitude controller | ~5KB | High (guaranteed timing) |
| IMU read + filter | ~3KB | High (guaranteed timing) |
| PID controllers | ~3KB | Medium |
| Math utilities (hot) | ~5KB | Medium |
| **Total SRAM code** | **~35KB** | |
| **Remaining SRAM** | **~485KB** | For .data, .bss, stacks, heap |

35KB of SRAM code is very manageable — it leaves 485KB for data, which is
still 2.5x more than the MatekF405's entire 192KB SRAM.

**However**, the 1-cycle cache hit means we may not need all of this in SRAM.
The flash write routines are mandatory (must be in SRAM during flash ops),
but the EKF and attitude controller may run acceptably from XIP cache. This
should be validated empirically in Step 3b.

## Key Differences from H750

| Aspect | H750 (works, borderline) | RP2350B | Impact |
|---|---|---|---|
| **CPU raw speed** | ~600 DMIPS | ~150 DMIPS (1 core) | 4x slower |
| **Internal flash** | 128KB for boot+critical | None | All code via XIP or SRAM |
| **ITCM** | 64KB zero-wait code | None (use SRAM) | RP2350 must use SRAM for guaranteed timing |
| **L1 I-cache hit** | 0 cycles | 1 cycle (XIP cache) | Similar — M33 pipeline hides 1-cycle hit |
| **Cache line size** | 32 bytes | 8 bytes | RP2350 has ~4x more misses per code path |
| **Cache lines** | 512 | 2048 | RP2350 covers more unique addresses |
| **Dual core** | No | Yes | RP2350 can offload I/O to Core 1 |
| **FPU** | Single+Double precision | Single precision only | Same constraint as F405 |
| **Overclock** | 480MHz (20% gain) | 200MHz (33% gain) | Overclock helps RP2350 more (%) |

### Why H750 is "borderline" despite being 4x faster

The H750's 400MHz Cortex-M7 is ~4x faster than RP2350, yet is "borderline"
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
to 2x than 4x** when accounting for feature set, FPU precision, and
dual-core benefits.

## Conclusions

1. **The 1-cycle XIP cache hit is the key enabler.** With 1-cycle hit, cached
   XIP code runs at near-full CPU speed (~150 MIPS), making pure XIP execution
   viable for many code paths. This is much better than a 2-cycle penalty
   would be.

2. **400Hz Copter appears feasible at 150MHz** from pure XIP cache, though
   tight (~20% headroom). With SRAM placement of ~35KB hot code, headroom
   improves to ~35%.

3. **SRAM placement is valuable but not strictly mandatory** for basic
   operation. It's **required** for:
   - Flash write safety (code must run from SRAM during flash erase/write)
   - Guaranteed timing (no cache miss jitter in control loop)
   - Additional headroom for 400Hz

4. **At 200MHz + dual-core, 400Hz Copter is very comfortable** (~65%
   headroom). This is the recommended production configuration.

5. **200Hz Copter at 150MHz requires no special optimization** — even pure
   XIP works with 60% headroom.

6. **400Hz Plane/Rover is easily achievable** at 150MHz — these vehicles
   have simpler control loops and lower EKF update rates.

7. **Step 3b (XIP memory model) remains critical** but the emphasis shifts:
   - Flash write SRAM placement: mandatory (XIP safety)
   - Performance SRAM placement: valuable optimization, not hard requirement
   - Performance baseline measurement: essential to validate these projections

## Recommendations for Implementation Plan

- **Default target: 400Hz Copter at 150MHz** with SRAM hot code (Step 5)
- **Stretch target: 400Hz Copter at 200MHz** with dual-core (Step 6f) for
  maximum headroom
- **SRAM placement macros must be in Step 3b** — mandatory for flash write
  safety, valuable for performance
- **Performance profiling infrastructure** early (GPIO toggle timing) to
  validate projections
- **Measure actual XIP cache hit rate** during flight loop to confirm the
  16KB cache covers the hot path adequately
