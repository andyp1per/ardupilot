#pragma once

#include "AP_HAL_Pico.h"

#if defined(RP2350)

#include <hal.h>
#include <AP_HAL_ChibiOS/Util.h>

class Pico::Util : public ChibiOS::Util {
protected:
    void _read_unique_id(uint8_t buf[12]) override;
};

#endif // RP2350
