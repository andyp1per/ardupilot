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
   RCInput driver using libgpiod v2 edge detection.

   This driver uses the Linux kernel's GPIO character device interface
   via libgpiod to receive hardware-timestamped edge events. This provides
   much more accurate pulse timing than DMA-based sampling approaches,
   as the kernel records the timestamp at interrupt time.

   The driver is suitable for decoding SBUS (100kbaud inverted), DSM,
   and PPM protocols from a single GPIO pin.
 */

#include "RCInput_Gpiod.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI && defined(HAVE_LIBGPIOD)

#include <AP_HAL/AP_HAL.h>
#include <errno.h>
#include <poll.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

RCInput_Gpiod::RCInput_Gpiod(const char *chip_path, uint32_t line_offset)
    : _chip_path(chip_path)
    , _line_offset(line_offset)
    , _chip(nullptr)
    , _request(nullptr)
    , _initialized(false)
{
    memset(&_state, 0, sizeof(_state));
}

RCInput_Gpiod::~RCInput_Gpiod()
{
    teardown();
}

void RCInput_Gpiod::teardown()
{
    if (_request) {
        gpiod_line_request_release(_request);
        _request = nullptr;
    }
    if (_chip) {
        gpiod_chip_close(_chip);
        _chip = nullptr;
    }
    _initialized = false;
}

void RCInput_Gpiod::init()
{
    RCInput::init();

    // Open GPIO chip
    _chip = gpiod_chip_open(_chip_path);
    if (!_chip) {
        hal.console->printf("RCInput_Gpiod: failed to open chip %s: %s\n",
                            _chip_path, strerror(errno));
        return;
    }

    // Configure line settings for edge detection on both edges
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        hal.console->printf("RCInput_Gpiod: failed to allocate line settings\n");
        teardown();
        return;
    }

    // Set direction to input with both edge detection
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH);

    // Use realtime clock for timestamps (better for timing analysis)
    gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC);

    // Create line config and add our line
    struct gpiod_line_config *line_config = gpiod_line_config_new();
    if (!line_config) {
        hal.console->printf("RCInput_Gpiod: failed to allocate line config\n");
        gpiod_line_settings_free(settings);
        teardown();
        return;
    }

    unsigned int offsets[] = { _line_offset };
    if (gpiod_line_config_add_line_settings(line_config, offsets, 1, settings) < 0) {
        hal.console->printf("RCInput_Gpiod: failed to add line settings\n");
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        teardown();
        return;
    }

    // Create request config
    struct gpiod_request_config *req_config = gpiod_request_config_new();
    if (!req_config) {
        hal.console->printf("RCInput_Gpiod: failed to allocate request config\n");
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        teardown();
        return;
    }
    gpiod_request_config_set_consumer(req_config, "ardupilot-rcin");

    // Request the line
    _request = gpiod_chip_request_lines(_chip, req_config, line_config);

    // Clean up config objects (no longer needed after request)
    gpiod_request_config_free(req_config);
    gpiod_line_config_free(line_config);
    gpiod_line_settings_free(settings);

    if (!_request) {
        hal.console->printf("RCInput_Gpiod: failed to request line %u: %s\n",
                            _line_offset, strerror(errno));
        teardown();
        return;
    }

    hal.console->printf("RCInput_Gpiod: initialized on %s line %u\n",
                        _chip_path, _line_offset);
    _initialized = true;
}

void RCInput_Gpiod::_timer_tick()
{
    if (!_initialized) {
        return;
    }

    _process_events();
}

void RCInput_Gpiod::_process_events()
{
    // Check if there are events available (non-blocking)
    int fd = gpiod_line_request_get_fd(_request);
    if (fd < 0) {
        return;
    }

    struct pollfd pfd = {
        .fd = fd,
        .events = POLLIN,
        .revents = 0
    };

    // Process all available events in the buffer
    while (true) {
        // Non-blocking poll to check for events
        int ret = poll(&pfd, 1, 0);
        if (ret <= 0) {
            // No more events or error
            break;
        }

        if (!(pfd.revents & POLLIN)) {
            break;
        }

        // Read one event
        struct gpiod_edge_event_buffer *event_buffer = gpiod_edge_event_buffer_new(1);
        if (!event_buffer) {
            break;
        }

        int num_events = gpiod_line_request_read_edge_events(_request, event_buffer, 1);
        if (num_events <= 0) {
            gpiod_edge_event_buffer_free(event_buffer);
            break;
        }

        struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(event_buffer, 0);
        if (!event) {
            gpiod_edge_event_buffer_free(event_buffer);
            continue;
        }

        // Get event details
        uint64_t timestamp_ns = gpiod_edge_event_get_timestamp_ns(event);
        enum gpiod_edge_event_type edge_type = gpiod_edge_event_get_event_type(event);
        bool is_rising = (edge_type == GPIOD_EDGE_EVENT_RISING_EDGE);

        gpiod_edge_event_buffer_free(event_buffer);

        // Process the edge
        if (!_state.have_first_edge) {
            // First edge - just record timestamp and type
            _state.last_edge_ns = timestamp_ns;
            _state.last_was_rising = is_rising;
            _state.have_first_edge = true;
            continue;
        }

        // Calculate time since last edge
        uint64_t delta_ns = timestamp_ns - _state.last_edge_ns;
        uint16_t delta_us = _ns_to_us(delta_ns);

        // Sanity check - ignore edges that are too far apart (> 100ms)
        // This handles wrap-around and long gaps
        if (delta_ns > 100000000ULL) {
            _state.last_edge_ns = timestamp_ns;
            _state.last_was_rising = is_rising;
            _state.low_width_us = 0;
            continue;
        }

        if (is_rising && !_state.last_was_rising) {
            // Rising edge after falling edge = end of low pulse
            // Store low width (width_s0)
            _state.low_width_us = delta_us;
        } else if (!is_rising && _state.last_was_rising) {
            // Falling edge after rising edge = end of high pulse
            // We now have a complete pulse pair: width_s0 (low) and width_s1 (high)
            uint16_t width_s0 = _state.low_width_us;
            uint16_t width_s1 = delta_us;

            // Only process if we have valid widths
            if (width_s0 > 0 && width_s1 > 0) {
                _process_rc_pulse(width_s0, width_s1);
            }
        }
        // Note: same-edge transitions (rising->rising or falling->falling)
        // indicate missed edges, we just update state and continue

        _state.last_edge_ns = timestamp_ns;
        _state.last_was_rising = is_rising;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI && defined(HAVE_LIBGPIOD)
