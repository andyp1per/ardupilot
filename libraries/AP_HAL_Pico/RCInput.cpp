#include "RCInput.h"

#if defined(RP2350) && defined(HAL_RCIN_IS_GPIO)

#include <AP_RCProtocol/AP_RCProtocol.h>

void Pico::RCInput::init()
{
    ChibiOS::RCInput::init();
    sig_reader.init(HAL_RCIN_GPIO_LINE);
    pulse_input_enabled = true;
}

void Pico::RCInput::pulse_input_enable(bool enable)
{
    ChibiOS::RCInput::pulse_input_enable(enable);
    if (!enable) {
        sig_reader.disable();
    }
}

void Pico::RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
#if AP_RCPROTOCOL_ENABLED
    if (pulse_input_enabled) {
        sig_reader.enable();
        AP_RCProtocol &rcprot = AP::RC();
        uint32_t width_s0, width_s1;
        while (sig_reader.read(width_s0, width_s1)) {
            rcprot.process_pulse(width_s0, width_s1);
        }
    }
#endif
    ChibiOS::RCInput::_timer_tick();
}

#endif // RP2350 && HAL_RCIN_IS_GPIO
