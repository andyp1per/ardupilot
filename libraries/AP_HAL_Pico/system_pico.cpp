/*
 * RP2350 fault handlers for ArduPilot.
 *
 * Lifted out of AP_HAL_ChibiOS/system.cpp so the file there is no longer
 * a two-platform fork. The handlers run from SRAM (__RAMFUNC2__) so they
 * remain functional even when XIP flash is mis-clocked, which would
 * otherwise turn a fault into an unrecoverable double-fault lockup.
 */
#include "AP_HAL_Pico.h"

#if defined(RP2350) && !AP_CRASHDUMP_ENABLED

#include <hal.h>
#include <hrt.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

extern "C" {

/* Cortex-M exception frame (hardware auto-stacked on exception entry). */
struct ap_fault_frame_t {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;      /* LR at point of fault (link register / return address) */
    uint32_t pc;      /* PC at point of fault */
    uint32_t xpsr;
};

/* All fault information captured in SRAM — survives even if XIP is broken. */
struct ap_fault_info_t {
    uint32_t magic;         /* 0xDEADFA17 when valid */
    uint32_t ipsr;          /* exception number (3=HardFault, 4=MemManage, 5=BusFault, 6=UsageFault) */
    uint32_t cfsr;          /* Combined Fault Status Register */
    uint32_t hfsr;          /* HardFault Status Register */
    uint32_t dfsr;          /* Debug Fault Status Register */
    uint32_t bfar;          /* Bus Fault Address Register (valid if CFSR.BFARVALID) */
    uint32_t mmfar;         /* MemManage Fault Address Register (valid if CFSR.MMARVALID) */
    uint32_t exc_return;    /* EXC_RETURN value in LR on handler entry */
    ap_fault_frame_t frame; /* hardware-stacked registers from faulting context */
};

// Placed in .noinit so it is not zeroed by startup and persists across soft
// resets, allowing post-mortem inspection via JTAG/SWD after a reset.
volatile ap_fault_info_t fault_info __attribute__((section(".noinit")));

/* Forward declaration required since the naked trampolines reference this
 * via an asm branch before the C definition. */
__RAMFUNC2__
void fault_capture(uint32_t *frame, uint32_t exc_return);
/*
 * Core fault capture implementation — __RAMFUNC2__ so it runs from SRAM and
 * is immune to XIP flash reliability issues at overclocked frequencies.
 * @param frame Pointer to the hardware exception frame (from PSP or MSP) @param exc_return EXC_RETURN value (LR on handler entry), used to determine which stack was active
 */
__RAMFUNC2__
void fault_capture(uint32_t *frame, uint32_t exc_return)
{
    /* Write the magic cookie last so a partial write is not mistaken for valid. */
    fault_info.ipsr       = __get_IPSR();
    fault_info.cfsr       = SCB->CFSR;
    fault_info.hfsr       = SCB->HFSR;
    fault_info.dfsr       = SCB->DFSR;
    fault_info.bfar       = SCB->BFAR;
    fault_info.mmfar      = SCB->MMFAR;
    fault_info.exc_return = exc_return;

    /* Copy the hardware-stacked frame registers — no memcpy (avoids XIP). */
    fault_info.frame.r0   = frame[0];
    fault_info.frame.r1   = frame[1];
    fault_info.frame.r2   = frame[2];
    fault_info.frame.r3   = frame[3];
    fault_info.frame.r12  = frame[4];
    fault_info.frame.lr   = frame[5];
    fault_info.frame.pc   = frame[6];
    fault_info.frame.xpsr = frame[7];

    /* Commit valid marker. */
    fault_info.magic = 0xDEADFA17U;

#if AP_WATCHDOG_SAVE_FAULT_ENABLED
    save_fault_watchdog(__LINE__,
                        (FaultType)fault_info.ipsr,
                        fault_info.bfar,
                        fault_info.frame.lr);
#endif

/*
 * BKPT only when a debugger is connected.
 * without one, BKPT escalates to HardFault which (since we are already in HardFault) causes a double-fault lockup at 0xEFFFFFFE, defeating the whole handler.
 */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        __asm volatile ("bkpt #0");
    }
    while (1) {}  /* Never return — GDB: `print fault_info` to inspect */
}

/*
 * Naked trampolines: determine PSP vs MSP from EXC_RETURN bit[2], then tail-call fault_capture().
 * The naked attribute means the compiler emits no prologue/epilogue, so no stack space is used before we save the frame is critical: COPY_VECTORS_TO_RAM copies the vector POINTER TABLE
 * to SRAM and redirects VTOR there, so the vector index fetch is flash-free.
 * But without __RAMFUNC2__ the handler CODE still lives in XIP flash — the CPU
 * would fetch bad instructions if XIP is unreliable at the higher clock, causing
 * a double-fault lockup before any C runs.  __RAMFUNC2__ ensures both the vector
 * entry AND the code it points to are in SRAM.
 * The "b fault_capture" branch relies on both the trampoline and fault_capture being in the same.ramtext section (VMA in RAM), keeping the offset within the 16 MB range of the Thumb-2 unconditional branch instruction.
 */
__RAMFUNC2__
void HardFault_Handler(void) __attribute__((naked));
__RAMFUNC2__
void HardFault_Handler(void) {
    __asm volatile (
        "tst    lr, #4          \n"  /* test EXC_RETURN bit[2]: 0=MSP, 1=PSP */
        "ite    eq              \n"
        "mrseq  r0, msp         \n"  /* r0 = frame pointer (MSP) */
        "mrsne  r0, psp         \n"  /* r0 = frame pointer (PSP) */
        "mov    r1, lr          \n"  /* r1 = EXC_RETURN */
        "b      fault_capture   \n"  /* tail-call the RAMFUNC implementation */
    );
}

/*
 * BusFault, UsageFault, MemManage all share the same capture path.
 * Without SCB->SHCSR fault-enable bits set (ChibiOS doesn't set them by default) these escalate to HardFault anyway, but having explicit handlers gives cleaner CFSR decoding.
 */
__RAMFUNC2__
void BusFault_Handler(void) __attribute__((naked));
__RAMFUNC2__
void BusFault_Handler(void) {
    __asm volatile (
        "tst    lr, #4          \n"
        "ite    eq              \n"
        "mrseq  r0, msp         \n"
        "mrsne  r0, psp         \n"
        "mov    r1, lr          \n"
        "b      fault_capture   \n"
    );
}

__RAMFUNC2__
void UsageFault_Handler(void) __attribute__((naked));
__RAMFUNC2__
void UsageFault_Handler(void) {
    __asm volatile (
        "tst    lr, #4          \n"
        "ite    eq              \n"
        "mrseq  r0, msp         \n"
        "mrsne  r0, psp         \n"
        "mov    r1, lr          \n"
        "b      fault_capture   \n"
    );
}

__RAMFUNC2__
void MemManage_Handler(void) __attribute__((naked));
__RAMFUNC2__
void MemManage_Handler(void) {
    __asm volatile (
        "tst    lr, #4          \n"
        "ite    eq              \n"
        "mrseq  r0, msp         \n"
        "mrsne  r0, psp         \n"
        "mov    r1, lr          \n"
        "b      fault_capture   \n"
    );
}

} // extern "C"

#endif // RP2350 && !AP_CRASHDUMP_ENABLED
