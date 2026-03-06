# Risks and Mitigations

## Critical Risks

### R1: Insufficient SRAM for Vehicle Firmware

**Risk:** 520KB SRAM may not be enough for a full vehicle build (Copter/Plane).

**Likelihood:** Medium-High
**Impact:** High - would limit port to AP_Periph only

**Mitigation:**
- Start with AP_Periph (fits easily in ~150KB RAM)
- Profile memory usage incrementally as features are added
- Use ESP32 board header as template for feature disabling
- Disable: QuadPlane, GyroFFT, Avoidance, Fence, Follow, Terrain, Camera,
  Mount, Gripper, Landing Gear, ADSB, Soaring, Generator
- Use single-precision EKF (`HAL_WITH_EKF_DOUBLE 0`)
- Consider EKF2 instead of EKF3 (smaller memory footprint)
- Target Rover/Plane first (less RAM than Copter)

**Fallback:** If vehicle builds don't fit, the port remains valuable as AP_Periph.

---

### R2: PIO UART Reliability at High Baud Rates

**Risk:** PIO-based UARTs may have reliability issues at high baud rates
(921600) needed for some telemetry/GPS links.

**Likelihood:** Low-Medium
**Impact:** Medium - degraded serial performance

**Mitigation:**
- PIO UARTs are well-proven in the Pico ecosystem at standard baud rates
- Use hardware UARTs for highest-speed links (GPS at 115200-230400)
- PIO UART with DMA provides hardware-like reliability
- Extensive testing at target baud rates before integration
- Fall back to lower baud rates if issues arise (115200 is always sufficient)

---

### R3: Sensor Sampling Rate Limitations

**Risk:** 150MHz clock may not sustain 1kHz+ IMU sampling with concurrent I/O.

**Likelihood:** Medium
**Impact:** High - affects flight controller stability

**Mitigation:**
- Cortex-M33 DSP extensions help with filter processing
- Dual-core: dedicate one core to sensor sampling, other to main loop
- DMA for all SPI transfers (CPU not blocked during sensor reads)
- Start with 400Hz loop rate, optimize toward 1kHz
- Profile CPU usage at each phase
- Consider running at 200MHz (RP2350 supports overclocking)

---

### R4: Flash XIP Latency

**Risk:** Executing code from external QSPI flash via XIP may cause
unpredictable latency spikes.

**Likelihood:** Medium
**Impact:** Medium - timing jitter in control loops

**Mitigation:**
- RP2350 has 16KB XIP cache (helps with hot code paths)
- Critical functions marked `__RAMFUNC__` to execute from SRAM
- Scheduler hot paths, interrupt handlers, and sensor reads placed in SRAM
- Pico SDK provides `__not_in_flash_func()` attribute
- Flash writes (storage) carefully scheduled outside control loop

---

### R5: PIO CAN Bus Reliability

**Risk:** PIO-based CAN implementation may not be robust enough for
safety-critical CAN communication (DroneCAN).

**Likelihood:** Medium
**Impact:** Medium-High for AP_Periph, Low for standalone vehicle

**Mitigation:**
- Option A: Use proven PIO CAN implementations from the community
- Option B: Use MCP2515 SPI-to-CAN external controller (rock-solid, well-tested)
- For AP_Periph, CAN reliability is critical → prefer MCP2515 initially
- PIO CAN can be developed and validated separately, then swapped in
- Test with CAN bus analyzer for bit-timing accuracy and error rates

---

## Major Risks

### R6: ChibiOS RP2350 Support Maturity

**Risk:** ChibiOS RP2350 support is in trunk, not a stable release. LLD
drivers may have undiscovered bugs that surface under ArduPilot's workload.

**Likelihood:** Low-Medium
**Impact:** Medium - driver bugs requiring workarounds or upstream fixes

**Mitigation:**
- LLD drivers appear complete and well-structured (all 11 drivers fully implemented)
- ChibiOS demo (`RT-RP2350-PICO2`) confirms basic functionality
- Start with simple peripheral usage, increasing complexity incrementally
- Maintain ability to patch ChibiOS locally if needed (submodule fork)
- Report bugs upstream to ChibiOS project for community benefit

---

### R7: ChibiOS SMP Dual-Core Complexity

**Risk:** ChibiOS SMP on dual-core RP2350 may introduce subtle concurrency
bugs (race conditions, priority inversions, inter-core scheduling issues).

**Likelihood:** Medium
**Impact:** Medium - hard-to-debug crashes

**Mitigation:**
- ChibiOS SMP is proven on RP2040 and now RP2350 (demo works)
- ArduPilot already handles ChibiOS threading (AP_HAL_ChibiOS uses same patterns)
- Start with single-core, add SMP later
- RP2350's hardware spinlock #31 used by ChibiOS for inter-core sync
- Inter-processor FIFO provides reliable cross-core communication

---

### R8: No Community Precedent

**Risk:** No existing ArduPilot port for RP2040/RP2350 means no community
experience to draw from.

**Likelihood:** High (fact, not risk)
**Impact:** Low-Medium - slower progress

**Mitigation:**
- AP_HAL_ChibiOS provides the closest reference (same RTOS, same API patterns)
- ESP32 port demonstrates a separate HAL approach works long-term
- ChibiOS RP2350 demo provides a working starting point
- RP2040/RP2350 has a large and active community (PIO programs, etc.)
- Engage ArduPilot Discord/forum early for feedback

---

### R9: USB 1.1 Bandwidth Limitation

**Risk:** USB 1.1 (12 Mbps) may be insufficient for high-rate MAVLink
telemetry and log download.

**Likelihood:** Low
**Impact:** Low - slower log download, but adequate for real-time telemetry

**Mitigation:**
- MAVLink at 115200 baud equivalent is well within USB 1.1 bandwidth
- Log download will be slower than USB 2.0 boards but functional
- WiFi module could supplement for higher bandwidth if needed

---

## Minor Risks

### R10: ADC Resolution and Noise

**Risk:** 12-bit ADC with only 4 channels may have more noise than STM32's
16-bit ADC with extensive analog filtering.

**Likelihood:** Low
**Impact:** Low - slightly noisier battery readings

**Mitigation:**
- Software oversampling and filtering
- Proper PCB layout with decoupling caps (per reference designs)
- 12-bit is adequate for battery monitoring (same as ESP32)

---

### R11: Watchdog Timer Differences

**Risk:** RP2350 watchdog behavior differs from STM32, may need careful
configuration to avoid false resets.

**Likelihood:** Low
**Impact:** Low - configurable

**Mitigation:**
- Pico SDK provides watchdog API
- Configure timeout appropriate for ArduPilot main loop (2-5 seconds)
- Feed from main loop and timer thread

---

### R12: GCC Toolchain Compatibility

**Risk:** ArduPilot uses GCC 10.2 (`gcc-arm-none-eabi-10-2020-q4-major`).
RP2350's Cortex-M33 may require a newer compiler.

**Likelihood:** Very Low (full build tested and confirmed)
**Impact:** Medium - would require dual toolchain or ArduPilot-wide upgrade

**Mitigation:**
- **CONFIRMED: Full ChibiOS RP2350 demo (88 source files, all LLD drivers,
  dual-core SMP) compiles with zero errors on GCC 10.2.1**
- CMSIS `core_cm33.h` V5.0.9 only requires GCC 4.5+
- ChibiOS uses standard C99/C11 and GCC `__attribute__` extensions
- No TrustZone CMSE intrinsics used (would need GCC 11+ with `-mcmse`)

**Status:** Resolved. See [developer-concerns.md](developer-concerns.md) section 6.

---

### R13: Performance at Target Loop Rates

**Risk:** 150MHz Cortex-M33 with external flash may not achieve adequate control
loop rates for flight-critical vehicles (Copter needs 200-400Hz).

**Likelihood:** Medium
**Impact:** High for Copter, Medium for Plane/Rover

**Mitigation:**
- Dual-core: offload I/O to Core 1, keep control loop on Core 0
- DMA for all SPI/ADC transfers (already in ChibiOS LLD)
- PIO offloads DShot, UART, SBUS from CPU entirely
- Place hot code in SRAM via `__RAMFUNC__` / `__not_in_flash_func()`
- Start at 200Hz, optimize toward 400Hz
- Overclock to 200MHz (common, well within spec)
- ESP32 at 240MHz achieves flight → RP2350 should be comparable

**Status:** Medium risk. See [developer-concerns.md](developer-concerns.md) section 8.

---

## Risk Summary Matrix

| Risk | Likelihood | Impact | Priority |
|---|---|---|---|
| R1: SRAM insufficient | Medium-High | High | **Critical** |
| R2: PIO UART reliability | Low-Medium | Medium | Medium |
| R3: Sampling rate | Medium | High | **High** |
| R4: XIP latency | Medium | Medium | Medium |
| R5: PIO CAN reliability | Medium | Medium-High | **High** |
| R6: ChibiOS RP2350 maturity | Low-Medium | Medium | Medium |
| R7: ChibiOS SMP bugs | Medium | Medium | Medium |
| R8: No precedent | High | Low-Medium | Medium |
| R9: USB 1.1 bandwidth | Low | Low | Low |
| R10: ADC noise | Low | Low | Low |
| R11: Watchdog | Low | Low | Low |
| R12: GCC toolchain | Very Low | Medium | **Resolved** (full build tested) |
| R13: Loop rate performance | Medium | High | **High** |
