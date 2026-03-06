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
| **XIP cache** | None (L1 cache serves this role) | 16KB, 8B lines, 2-way set-associative |
| **XIP cache hit latency** | 0 cycles (L1 cache is core-integrated) | 2 system clock cycles |
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

## Cache Miss Latency Analysis

This is the critical metric — how long does the CPU stall when it needs an
instruction that isn't in cache?

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
   execute from internal flash at zero wait-states
2. **ITCM (64KB)**: Hot code copied here runs at zero wait-states, bypassing
   QSPI entirely. ArduPilot places scheduler, attitude controller, and EKF
   critical paths here.
3. **L1 I-cache (16KB, 32B lines)**: Core-integrated, zero-penalty cache hits.
   With 512 cache lines and 32B per line, this covers ~16KB of hot code with
   zero-cycle hits. Sequential code benefits from line fill (8 instructions
   per line).

**Effective H750 performance:** For hot code in ITCM, execution is at full
400MHz. For cached XIP code, hits are zero-cycle. Only cold/random code paths
pay the ~860ns miss penalty. This is why H750 is "borderline" but works —
the hot path mostly avoids XIP misses.

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
3. **XIP cache (16KB, 8B lines)**: 2048 cache lines, but only 8B each.
   **2-cycle hit latency** (not zero like H750's L1). Sequential code gets
   one instruction every 2 cycles from cache hits.
4. **SRAM code placement**: Functions marked `__not_in_flash_func()` are
   copied to SRAM at boot and execute at zero wait-states. This is the
   primary mitigation — equivalent to H750's ITCM but sharing the heap.

## Effective Execution Speed Comparison

### Cached XIP Code (hot path in cache, no misses)

| Metric | STM32H750 | RP2350B | Ratio |
|---|---|---|---|
| CPU clock | 400MHz | 150MHz | 2.67x |
| Cache hit penalty | 0 cycles | 2 cycles | — |
| Effective instruction rate | 400MHz × ~1.5 IPC = 600 MIPS | 150MHz / 3 cycles = 50 MIPS | **12x** |
| Per-instruction time | ~1.67ns | ~20ns | 12x slower |

**Note:** The 2-cycle XIP cache hit penalty is devastating. Even with a 100%
cache hit rate, RP2350B executes XIP code at effectively **50 MIPS** — one
instruction every 3 cycles (1 execute + 2 cache access). The H750's
core-integrated L1 cache has zero hit penalty.

Wait — this needs qualification. The Cortex-M33 pipeline can overlap cache
access with execution for sequential code (prefetch buffer). The actual
sustained throughput from XIP cache is likely closer to **1 instruction per
2 cycles** (75 MIPS) for sequential Thumb-2 code, not worst-case 3 cycles.
But it's still far below the 150 MIPS that SRAM execution would give.

### SRAM-Resident Code (hot path in SRAM)

| Metric | STM32H750 (ITCM) | RP2350B (SRAM) | Ratio |
|---|---|---|---|
| CPU clock | 400MHz | 150MHz | 2.67x |
| Wait states | 0 | 0 | Same |
| Effective instruction rate | ~600 MIPS | ~150 MIPS | **4x** |
| Per-instruction time | ~1.67ns | ~6.67ns | 4x slower |

With SRAM execution, the gap narrows to **4x** (just the clock speed ×
IPC difference). This is the achievable baseline for hot code.

### Code with Cache Misses (cold paths, large functions)

| Metric | STM32H750 | RP2350B | Ratio |
|---|---|---|---|
| Miss penalty (time) | ~860ns | ~347ns | H750 worse per miss |
| Miss penalty (CPU cycles) | ~344 cycles | ~52 cycles | H750 loses more cycles |
| Data per miss | 32 bytes (~8 insns) | 8 bytes (~2 insns) | H750 more efficient |
| Effective miss cost per insn | ~107ns | ~173ns | RP2350 1.6x worse |
| Amortized miss rate | Lower (32B lines) | Higher (8B lines) | RP2350 has more misses |

For cold code paths (initialization, rare branches), RP2350B's smaller cache
lines mean ~4x more cache misses for the same code sequence. Each miss is
cheaper in absolute time (347ns vs 860ns), but there are far more of them.

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

**Scenario A: All code from XIP (no SRAM placement)**

With ~50 MIPS effective (XIP cache hits at 2-cycle penalty), RP2350 would be
~3x slower than F405 (which has ~168 MIPS from internal flash):

| Phase | RP2350B @ 150MHz XIP | Budget |
|---|---|---|
| IMU read | ~150µs | |
| EKF update | ~2400µs | |
| Attitude control | ~600µs | |
| Motor output | ~150µs | |
| Logging + MAVLink | ~1200µs | |
| **Total** | **~4500µs** | **EXCEEDS 2500µs budget** |

**Result: 400Hz Copter impossible from pure XIP. Even 200Hz (5000µs) is tight.**

**Scenario B: Hot path in SRAM (~32KB), rest from XIP**

With critical code (EKF inner loop, attitude controller, IMU read, scheduler)
in SRAM at 150 MIPS, and cold paths from XIP:

| Phase | RP2350B SRAM+XIP | Notes |
|---|---|---|
| IMU read | ~100µs | SRAM (sensor read + filter) |
| EKF update | ~1600µs | SRAM for core, XIP for init/config |
| Attitude control | ~400µs | SRAM |
| Motor output (PIO) | ~50µs | PIO handles DShot, CPU just loads FIFO |
| Logging + MAVLink | ~800µs | XIP (not time-critical) |
| **Total** | **~2950µs** | **Exceeds 2500µs at 400Hz** |

**Result: 400Hz Copter marginal even with SRAM placement at 150MHz.**

**Scenario C: SRAM hot path + overclock to 200MHz + dual-core**

RP2350 commonly runs at 200MHz (well within safe overclocking range). With
Core 0 doing control + EKF, and Core 1 handling I/O:

| Phase | RP2350B @ 200MHz | Notes |
|---|---|---|
| IMU read | ~75µs | SRAM, 200MHz |
| EKF update | ~1200µs | SRAM core path, 200MHz |
| Attitude control | ~300µs | SRAM, 200MHz |
| Motor output (PIO) | ~30µs | PIO + SRAM FIFO load |
| **Core 0 total** | **~1605µs** | Control loop only |
| Logging + MAVLink | (Core 1) | Offloaded |
| **Headroom** | ~895µs (36%) | **400Hz feasible** |

**Result: 400Hz Copter likely achievable at 200MHz with SRAM + dual-core.**

**Scenario D: Conservative — 200Hz Copter at 150MHz**

At 200Hz (5000µs budget), even Scenario B works comfortably:

| Phase | RP2350B @ 150MHz | Budget |
|---|---|---|
| Total (from Scenario B) | ~2950µs | 5000µs available |
| **Headroom** | ~2050µs (41%) | **200Hz comfortable** |

## SRAM Budget for Code Placement

| Code section | Size (est.) | Priority |
|---|---|---|
| EKF3 core loop | ~15KB | Critical |
| Attitude controller | ~5KB | Critical |
| IMU read + filter | ~3KB | Critical |
| Scheduler fast path | ~2KB | Critical |
| Flash write routines | ~2KB | Critical (XIP safety) |
| PID controllers | ~3KB | High |
| Math utilities (hot) | ~5KB | High |
| **Total SRAM code** | **~35KB** | |
| **Remaining SRAM** | **~485KB** | For .data, .bss, stacks, heap |

35KB of SRAM code is very manageable — it leaves 485KB for data, which is
still 2.5x more than the MatekF405's entire 192KB SRAM.

## Key Differences from H750

| Aspect | H750 (works, borderline) | RP2350B | Impact |
|---|---|---|---|
| **CPU raw speed** | ~600 DMIPS | ~150 DMIPS (1 core) | 4x slower |
| **Internal flash** | 128KB for boot+critical | None | More SRAM code needed |
| **ITCM** | 64KB zero-wait code | None (use SRAM) | RP2350 must use SRAM, reducing heap |
| **L1 I-cache hit** | 0 cycles | 2 cycles | XIP hits 2-3x slower on RP2350 |
| **Cache line size** | 32 bytes | 8 bytes | RP2350 has 4x more misses per code path |
| **Dual core** | No | Yes | RP2350 can offload I/O to Core 1 |
| **FPU** | Single+Double precision | Single precision only | Same constraint as F405 |
| **Overclock** | 480MHz (20% gain) | 200MHz (33% gain) | Overclock helps RP2350 more (%) |

## Conclusions

1. **Pure XIP at 150MHz is not fast enough** for 400Hz Copter. The 2-cycle
   XIP cache hit penalty and 8-byte cache lines make XIP execution ~3x slower
   than internal flash execution on comparable hardware.

2. **SRAM placement is not optional — it's mandatory.** Unlike H750 where
   ITCM is a performance optimization, RP2350 SRAM code placement is required
   for any viable loop rate. ~35KB of hot code in SRAM is the minimum.

3. **400Hz Copter is achievable** but requires all three optimizations:
   - SRAM placement of ~35KB hot code
   - Overclock to 200MHz (well-established for RP2350)
   - Dual-core: I/O (logging, MAVLink, GPS) on Core 1

4. **200Hz Copter at 150MHz is comfortable** with just SRAM placement, no
   overclocking needed. This may be an acceptable initial target.

5. **400Hz Plane/Rover is easily achievable** — these vehicles have simpler
   control loops and lower EKF update rates, using well under 2500µs even
   with conservative settings.

6. **The H750 comparison is somewhat misleading** because H750 has 64KB ITCM +
   128KB internal flash to fall back on. RP2350 has none of this. But RP2350's
   dual-core compensates significantly by offloading I/O work.

7. **Step 3b (XIP memory model) is correctly identified as critical path.**
   The SRAM code placement infrastructure and performance baseline measurement
   will determine the achievable loop rates early in development.

## Recommendations for Implementation Plan

- **Default target: 200Hz Copter at 150MHz** for initial bringup (Step 5)
- **Stretch target: 400Hz Copter at 200MHz** with dual-core (Step 6f)
- **SRAM placement macros must be in Step 3b** — not a later optimization
- **Build-time SRAM budget tracking** — warn if SRAM code exceeds threshold
- **Performance profiling infrastructure** early (GPIO toggle timing) to
  measure actual vs projected loop times
- **Consider EKF2 instead of EKF3** for initial port — simpler, smaller,
  may fit better in SRAM budget
