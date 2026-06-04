# Laurel XIP Cache Locality / PGO Investigation

Tracking the effort to reduce RP2350 XIP cache thrashing on the EKF/AHRS hot
path. The RP2350 runs application code from external QSPI flash through a 16 KB
XIP cache; the EKF/AHRS/Copter hot path is ~130 KB, so it cannot all stay
cached. Today ~45 KB of the hottest code is hand-placed in SRAM via the
`__RAMFUNC2__` registry (`rp2350_ramfunc2_registry.txt`). This investigation
puts measurement behind that selection before deciding whether full
profile-guided optimisation (PGO) is worth the build complexity.

Plan: Step 0 measure, Step 1 data-driven SRAM relocation, Step 2 (only if
warranted) full GCC `-fprofile-use` with linker hot/cold clustering.

Status: Step 0 and Step 1 tooling landed and verified in software. Hardware
baseline numbers still to be collected.

## Step 0 - XIP cache hit-rate counter in perf_report

The RP2350 XIP controller exposes free-running cache counters (datasheet
12.3.8): `CTR_HIT` (`XIP_CTRL+0x0C`) and `CTR_ACC` (`XIP_CTRL+0x10`). Writing
any value clears a counter.

- `rp2350_xip_cache_stats(hit, acc)` in `board_rp2350.c` reads both counters and
  clears them, so each call returns the counts since the previous call.
- `Copter::perf_report()` (runs every ~10 s) calls it under `#if defined(RP2350)`
  and appends ` xip=NN%` (hit/acc over the interval) to the existing
  `Perf: main=.. rate=.. c0=.. c1=..` line, on both the console and the MAVLink
  STATUSTEXT.

Reading this: `xip=NN%` is the cache hit rate over the last reporting window.
Low hit rate = the in-flash working set is thrashing the 16 KB cache and is a
candidate for SRAM relocation; high hit rate = relocation will not help much and
Step 2 reordering is unlikely to be worth it.

Caveats:
- The counters are read a few cycles apart, so the ratio is approximate (fine
  over millions of accesses).
- If everything (`c0`, `c1`, `xip`) is 3 digits the STATUSTEXT can exceed the
  50-char MAVLink limit and the tail truncates; the console line is unaffected.
  Real release-build loads are 2-digit so this is an edge case.
- Counters saturate rather than wrap; not reachable within a 10 s window at any
  realistic XIP access rate.

Verification: builds and links clean for Laurel (`./waf copter`, 34 s);
`rp2350_xip_cache_stats` present in the ELF at `0x101863b0`.

## Step 1 - SWD statistical PC profiler

`Tools/debug/rp2350_pc_profiler.py` - standalone, stdlib-only (plus
`arm-none-eabi-nm`/`-c++filt`). Deliberately not a Claude skill so anyone can
run it.

It samples the Cortex-M33 DWT PCSR (`0xE000101C`) over OpenOCD's TCL RPC while
the target runs at full speed. PCSR reads are non-intrusive (no halt), so the
profile reflects real timing. Each PC is attributed to a function via the ELF
symbol table and bucketed by region (XIP flash vs SRAM vs ROM vs invalid).

Output:
1. Region breakdown - how much time is spent in XIP-cached flash vs already in
   SRAM vs idle (PCSR invalid, e.g. core parked in WFE).
2. Top-N functions by exclusive sample count, tagged by region.
3. Registry suggestions - the hottest XIP-resident functions above a threshold
   that are not already in `rp2350_ramfunc2_registry.txt`, printed as
   ready-to-paste `path|symbol` lines.

Usage (OpenOCD must already be running with a `tcl_port`):

```bash
# core0 (ChibiOS main loop)
Tools/debug/rp2350_pc_profiler.py --elf build/Laurel/bin/arducopter \
    --tcl-port 50001 --samples 30000 --out /tmp/prof_core0.md

# core1 (EKF / PID / attitude dispatch) - must select that core first
Tools/debug/rp2350_pc_profiler.py --target-select rp2350.cpu1 \
    --samples 30000 --out /tmp/prof_core1.md
```

Workflow to iterate the SRAM working set:
1. Fly/replay a representative workload on the bench (idle bench under-counts the
   EKF path - it only runs hot when armed/fusing).
2. Run the profiler on core0 and core1.
3. Paste the suggested `path|symbol` lines into `rp2350_ramfunc2_registry.txt`,
   rebuild, and watch `xip=NN%` in `perf_report` plus `read_AHRS` timing.
4. Stop when SRAM headroom runs low or the hit rate stops improving.

Verified in software: symbol parse (13775 functions), registry diff, region
attribution (`rp2350_xip_cache_stats`->xip at `0x101863b0`, `Copter::read_AHRS`
->sram at `0x20005800`), source-path relativisation, demangling, clean failure
when OpenOCD is absent; flake8 clean.

The hardware sampling loop itself is the one path not yet exercised on a live
target. The OpenOCD `profile` command is a fallback if PCSR reads misbehave.

## Baseline numbers (to fill in on hardware)

| Metric | Value | Notes |
|--------|-------|-------|
| `xip=` hit rate, armed, release build | TBD | from perf_report |
| `xip=` hit rate, idle bench | TBD | for comparison |
| core0 XIP / SRAM / invalid sample split | TBD | profiler region breakdown |
| core1 XIP / SRAM / invalid sample split | TBD | profiler region breakdown |
| `read_AHRS` AVG us | TBD | current ~4220 us baseline (FEATURE_GAP) |
| free SRAM | TBD | headroom for more relocation |

## Decision gate for Step 2 (full PGO)

Pursue `-fprofile-use` + linker `.text.hot`/`.text.unlikely` clustering only if,
after relocating the measured hot set to SRAM, the armed `xip=` hit rate is
still low AND a meaningful fraction of samples remain in XIP. If relocation
alone drives the hit rate high, the size mismatch (130 KB path vs 16 KB cache)
means in-flash reordering has little left to win and Step 2 is not worth the
build-system cost.
