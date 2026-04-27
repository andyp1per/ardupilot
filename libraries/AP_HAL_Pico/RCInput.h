#pragma once

#include "AP_HAL_Pico.h"

#if defined(RP2350) && defined(HAL_RCIN_IS_GPIO)

#include <hal.h>
#include <AP_HAL_ChibiOS/RCInput.h>
#include "SoftSigReaderRP2350.h"

class Pico::RCInput : public ChibiOS::RCInput {
public:
    void init() override;
    void pulse_input_enable(bool enable) override;
    void _timer_tick(void) override;

private:
    Pico::SoftSigReaderRP2350 sig_reader;
};

#endif // RP2350 && HAL_RCIN_IS_GPIO
