/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   RCInput driver using libgpiod v2 edge detection for precise pulse timing.
   This provides hardware interrupt-driven edge detection with nanosecond
   timestamps, suitable for decoding SBUS, DSM, and PPM protocols.
 */
#pragma once

#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI && defined(HAVE_LIBGPIOD)

#include "RCInput.h"
#include <gpiod.h>

namespace Linux {

class RCInput_Gpiod : public RCInput
{
public:
    RCInput_Gpiod(const char *chip_path, uint32_t line_offset);
    ~RCInput_Gpiod();

    void init() override;
    void _timer_tick() override;
    void teardown() override;

private:
    // GPIO chip and line configuration
    const char *_chip_path;
    uint32_t _line_offset;

    // libgpiod v2 handles
    struct gpiod_chip *_chip;
    struct gpiod_line_request *_request;

    // Edge timing state
    struct {
        uint64_t last_edge_ns;      // timestamp of last edge in nanoseconds
        uint16_t low_width_us;      // width of low pulse (width_s0)
        bool last_was_rising;       // true if last edge was rising
        bool have_first_edge;       // true after first edge received
    } _state;

    bool _initialized;

    // Process accumulated edge events from the kernel buffer
    void _process_events();

    // Convert nanoseconds to microseconds with rounding
    static uint16_t _ns_to_us(uint64_t ns) {
        return static_cast<uint16_t>((ns + 500) / 1000);
    }
};

}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI && defined(HAVE_LIBGPIOD)
