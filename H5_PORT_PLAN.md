# ArduPilot STM32H5 Port — Finish Plan

Draft plan for the remaining work on branch `pr-h5-armv8` before the PR is mergeable.
**Scope: STM32H573 on the NucleoH573 dev board.** — there is no shipping H5 product board today, so the target is Nucleo-level support only.

## Ground Truth

- Actual effort to date: **~10 engineer-days, Gemini-assisted**. The 47 commits look like more because AI assistance produces commits faster than unassisted work — retrospective "commit count × typical eng-days" math doesn't apply.
- ChibiOS has been updated to 21.11.5 (hopefully with further upstream H5 fixes).
- **The bootloader compiles and runs far enough to be verified on a debugger without crashing** (some bugs were fixed to reach this point). However, **no USB device enumerates** — this is the current blocker.
- Beyond "it doesn't crash at boot," essentially nothing is tested yet.
- Further work will be **Claude-assisted**, so estimates below assume similar AI-accelerated productivity to what was observed on the first 10 days.
- Where H5 shares peripheral architecture with H7, problems are **not** anticipated — ChibiOS is good at that. Where peripherals differ, budget real debug time.
- **DMA and USB are expected to be the hardest items.** Estimates reflect that.

**Estimates** are calibrated against the observed pace of the first 10 eng-days, which delivered an unusually dense amount of subsystem work (kernel bump, clock tree with multiple debug iterations, GPDMA v1→v3, flash driver, bootloader protocol, and more). The empirical pace is roughly **1.2–1.5 "significant items" per eng-day including debug/iteration** when the item is AI-tractable. Hard items flagged as the exceptions (USB, ICACHE root-cause) are budgeted higher. Ranges are ±40%. Calendar time ~1.5–2× effort.

## What's Landed (retrospective)

47 commits representing ~10 engineer-days of Gemini-assisted work. Rough allocation:

| Area | Landed |
|------|--------|
| ChibiOS kernel v8 + ARMv8-M / Cortex-M33 startup, submodule tracking to 21.11.5 | `fa5311ef9b`, `821f8bdf1e`, multiple submodule bumps |
| `mcuconf` for STM32H573 | `7743973db2` |
| Clock tree (HSE 8 MHz, HSI48, USB constraints) — several iterations | `6ff909d324`, `dd308a1009`, `11fe26a9a6`, `72972549e3`, `0b057014a5`, `31f4d4f915`, `909101c500` |
| GPDMA / DMA3v1 — v1 → v3 iterations | `44bcc546e0`, `7eb5839e2e`, `ce040dd886`, `6109496fd8`, `d13a4eedc0`, `b4b151c5b5`, `126e22e58b`, `ef80253a6c` |
| Flash driver (8 KB sectors) | `71ecd3d4c5`, `1eaa3f36bb`, `9205590e02` |
| IWDG, ADC, analog driver, PAL lines, system timer, GPIO clock | `2bc00584df`, `b4fff7d681`, `d3ff8b3191`, `daa62148bb`, `b1f385d46e`, `1135cea304` |
| Bootloader protocol + H5 MCU IDs + Z-rev | `8d9bdab17e`, `07b69301c4`, `981868ebb3`, `f6edbfdbb8` |
| USB clock + power (enumeration not yet working) | `04d39de522`, `6bea2e50e6` |
| CPUID/UID workarounds for icache | `4629eb7b39`, `3094b4b6ca` |
| NucleoH573 hwdef (minimal) | `ea523cb2bc` |
| `SimOnHW` path on H5 | `ede9e4a965` |

Important: **most of this is code-complete, not verified.** Treat it as compiles-and-boots, not functional.

## Current Blocker

**USB does not enumerate.** Everything downstream (MAVLink to GCS, interactive bring-up, parameter upload) is gated on this. USB is also flagged as one of the two hardest items in this port. Fixing it is the top of M1.

## Gap Analysis

### Peripherals — unverified
- **USB CDC** — clock configured, enumeration broken. **Current blocker.**
- **GPDMA end-to-end** — driver iterated to v3, but no peripheral has exercised it under real workload. Second-hardest item.
- **SPI** — pins configured in NucleoH573 hwdef; IMU/baro probes commented out. Low risk (H7-similar) once DMA works.
- **I2C** — not yet touched. Low risk (H7-similar).
- **FDCAN** — not yet touched. Low risk (H7-similar); budget some debug time for DMA integration.
- **Timers (RCIN / RCOut)** — not yet touched. Low-to-medium risk; H5 timer block is similar but not identical to H7.

### Storage
- NucleoH573 uses `HAL_USE_EMPTY_STORAGE` — no parameter persistence. Needs internal flash sector carve-out using the new 8 KB sector layout.

### ARMv8-M specifics
- **ICACHE / UID workaround** — commit `3094b4b6ca` copies device UID at startup to avoid icache access errors. Root cause unknown; decide whether to root-cause or document as a permanent mitigation.
- **TrustZone / SAU** — confirm we run fully non-secure and document the choice. Deferred TrustZone work until a product board drives the need.
- **TRNG** — H5 hardware RNG not yet wired into HAL.

### Toolchain / build infra
- `AUTOBUILD_TARGETS None` on NucleoH573 — decide whether to enable for the Nucleo as a community target.
- Add NucleoH573 to CI: `Tools/scripts/build_binaries.py`, board lists.
- Commit NucleoH573 bootloader binary to `Tools/bootloaders/`.

### Upstream hygiene
- Branch is 47 commits over master; the ~2700-file diff stat suggests master has drifted. Rebase before PR.
- Multiple `modules/ChibiOS` submodule bumps should squash to a single final-SHA bump.

## Proposed Milestones

Milestones reordered to tackle the blocker first.

### M1 — Unblock: USB + DMA working

GPDMA v3 is in the tree but **entirely unverified on silicon**. Problems are anticipated, not hoped-against. A GPDMA bug cascades into most M2 items (SPI, UART, FDCAN), so M1.2 is both a bring-up task and a risk-retirement task — its outcome changes the shape of M2.

| # | Task | Effort (eng-days) | Risk |
|---|------|-------------------|------|
| 1 | Debug USB enumeration (clock, PHY power, descriptors, ChibiOS OTG path on H5) | 2–4 | **High** — flagged hardest; could stall on upstream |
| 2 | GPDMA end-to-end on silicon: one UART TX+RX under sustained traffic, plus SPI loopback. Expect driver/channel-mux/cache-coherency bugs. | 3–6 | **High** — anticipated problems; cascades into M2 |
| 3 | Root-cause or formally document the ICACHE/UID workaround | 2–4 | Medium — can rabbit-hole |
| | **M1 subtotal** | **7–14** | |

### M2 — Nucleo functional bring-up
Peripheral logic is mostly H7-similar, but **all DMA-using peripherals inherit whatever shape GPDMA takes after M1.2**. If M1.2 uncovers driver rework, re-estimate M2. Low-bounds below assume GPDMA behaves; high-bounds include some additional DMA debug per peripheral.

| # | Task | Effort (eng-days) | Risk |
|---|------|-------------------|------|
| 1 | SPI IMU + baro with GPDMA | 1–3 | Medium (DMA-dependent) |
| 2 | Internal-flash storage sector; drop `HAL_USE_EMPTY_STORAGE` | 1 | Low (no DMA) |
| 3 | I2C + one compass | 0.5–2 | Low-medium (DMA-dependent) |
| 4 | RCIN + RCOut (timer capture + PWM) | 1–2 | Medium (H5 TIM differences; DMA on RCOut) |
| 5 | FDCAN + DroneCAN GPS attach test | 1–3 | Medium (DMA-dependent) |
| 6 | TRNG → HAL random source | 0.5 | Low |
| 7 | Arming-capable bench test with MAVLink over USB | 0.5 | Low once M1 done |
| | **M2 subtotal** | **5.5–12** | |

### M3 — Release hygiene
| # | Task | Effort (eng-days) |
|---|------|-------------------|
| 1 | Rebase onto current master; squash per subsystem (per `CLAUDE.md`) | 1–2 |
| 2 | Single `modules/ChibiOS` submodule bump at correct commit | 0.25 |
| 3 | Commit NucleoH573 bootloader binary to `Tools/bootloaders/` | 0.25 |
| 4 | Add NucleoH573 to `build_binaries.py` + CI matrix | 0.5 |
| 5 | Enable `AUTOBUILD_TARGETS` if shipping Nucleo as community target | 0.25 |
| 6 | Build-every-commit verification | 0.5–1 |
| 7 | PR description with verified-vs-gaps feature matrix | 0.25 |
| | **M3 subtotal** | **3–4.5** |

### Totals

| Phase | Effort (eng-days) | Approx. calendar (1 engineer, ~1.5–2× multiplier) |
|-------|-------------------|---------------------------------------------------|
| Already done (Gemini-assisted) | ~10 | observed |
| **M1 — USB + DMA unblock** | 7–14 | 2–4 weeks |
| **M2 — peripheral bring-up** | 5.5–12 | 1.5–3 weeks |
| **M3 — release hygiene** | 3–4.5 | 0.5–1 week |
| **Remaining total** | **15.5–30.5 eng-days** | **~4–8 weeks calendar** |

At the observed pace, this is roughly **1.5–3× the effort already invested**. Three high-variance items drive the spread: USB enumeration, GPDMA-on-silicon, and ICACHE root-cause. GPDMA is the one most likely to force a mid-plan re-estimate, because whatever shape it settles into dictates the per-peripheral debug cost in M2. If GPDMA turns out to need deeper rework (or pushes a ChibiOS upstream change), add another 50% and accept that the ceiling is partly outside your control.

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| **GPDMA bugs on real silicon (anticipated)** | Every DMA-using peripheral in M2 inherits the issue — SPI, UART, FDCAN, RCOut all wobble | Retire the risk early in M1.2 with UART+SPI stress tests before starting M2; keep a minimal "DMA-off" fallback path so individual peripherals can make progress even if DMA work stalls |
| USB enumeration issue is a ChibiOS-upstream bug on H5 OTG | M1 slips indefinitely | Engage upstream early; fall back to UART-only for M2 validation if needed |
| ICACHE workaround is masking a real memory-access bug | Intermittent faults later | Root-cause during M1.3; don't mark port "done" until understood |
| H5 timer block diverges from H7 more than expected | M2.4 slips | Start RCIN/RCOut early in M2 to surface issues |
| Branch rebase conflicts due to 2700-file master drift | Days of rework | Rebase incrementally during M1/M2, not only at M3 |
