/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_CRSF_Out.cpp - High-level driver for CRSF RC Output
 *
 * This class provides the high-level "application" logic for the
 * CRSF RC Output feature. It is responsible for reading servo output values
 * from the main SRV_Channels and telling its underlying CRSF protocol instance
 * to send them at a user-configurable rate.
 */
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#if AP_CRSF_OUT_ENABLED

#include "AP_CRSF_Out.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>

//#define CRSF_RCOUT_DEBUG
#ifdef CRSF_RCOUT_DEBUG
# include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
static uint32_t last_update_debug_ms = 0;
# define debug_rcout(fmt, args...) do { if (hal.console) { hal.console->printf("CRSF_OUT: " fmt "\n", ##args); } } while(0)
#else
# define debug_rcout(fmt, args...)
#endif

const AP_Param::GroupInfo AP_CRSF_Out::var_info[] = {
    AP_GROUPINFO("RATE",  1, AP_CRSF_Out, _rate_hz, 50),
    AP_GROUPEND
};

AP_CRSF_Out* AP_CRSF_Out::_singleton;

AP_CRSF_Out::AP_CRSF_Out(void) :
    _state(State::WAITING_FOR_PORT)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

// Initialise the CRSF output driver
void AP_CRSF_Out::init()
{
    if (_state != State::WAITING_FOR_PORT) {
        return;
    }

    _crsf_port = AP_RCProtocol_CRSF::get_direct_attach_singleton(AP_SerialManager::SerialProtocol_CRSF_Output, 0);

    if (_crsf_port == nullptr) {
        debug_rcout("Init failed: No CRSF_Output port found");
        // not configured on any port
        return;
    }

    const uint16_t rate = _rate_hz.get();
    if (rate > 0) {
        _frame_interval_us = 1000000UL / rate;
    } else {
        _frame_interval_us = 20000; // 50Hz default
    }

    _state = State::NEGOTIATING_2M;
    _target_baudrate = 2000000;
    _crsf_port->reset_baud_negotiation();
    debug_rcout("Initialised, negotiating baudrate");
}

// Main update call, sends RC frames at the configured rate
void AP_CRSF_Out::update()
{
    if (_state == State::WAITING_FOR_PORT) {
#ifdef CRSF_RCOUT_DEBUG
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_update_debug_ms > 5000) { // print every 5s
            last_update_debug_ms = now_ms;
            debug_rcout("Update skipped: not initialised");
        }
#endif
        return;
    }

    const uint32_t now = AP_HAL::micros();
    const uint32_t BAUD_NEG_TIMEOUT_US = 1500000; // 1.5 second timeout for a response
    const uint32_t BAUD_NEG_INTERVAL_US = 500000; // send proposal every 500ms

    switch (_state) {
    case State::WAITING_FOR_PORT:
        // should have been handled above
        break;

    case State::NEGOTIATING_2M:
    case State::NEGOTIATING_1M: {
        // Check for response
        const auto result = _crsf_port->get_baud_negotiation_result();

        if (result == AP_RCProtocol_CRSF::BaudNegotiationResult::SUCCESS) {
            debug_rcout("%u baud negotiation successful", (unsigned)_target_baudrate);
            _state = State::RUNNING;
            break;
        }

        // Check for failure or timeout
        if (_baud_neg_start_us == 0) {
            _baud_neg_start_us = now;
        }
        const bool failed = (result == AP_RCProtocol_CRSF::BaudNegotiationResult::FAILED);
        const bool timeout = (now - _baud_neg_start_us > BAUD_NEG_TIMEOUT_US);

        if (failed || timeout) {
            if (_state == State::NEGOTIATING_2M) {
                debug_rcout("2M baud failed, trying 1M");
                _state = State::NEGOTIATING_1M;
                _target_baudrate = 1000000;
                _crsf_port->reset_baud_negotiation();
                _last_baud_neg_us = 0; // force immediate send
                _baud_neg_start_us = 0;
            } else { // NEGOTIATING_1M
                debug_rcout("1M baud failed, falling back to default");
                _state = State::RUNNING;
            }
            break;
        }

        // If pending, send proposal periodically
        if (now - _last_baud_neg_us > BAUD_NEG_INTERVAL_US) {
            _last_baud_neg_us = now;
            _crsf_port->send_speed_proposal(_target_baudrate);
            debug_rcout("Sent speed proposal for %u", (unsigned)_target_baudrate);
        }
        break;
    }
    case State::RUNNING:
        if (now - _last_frame_us < _frame_interval_us) {
            return;
        }
        _last_frame_us = now;

        uint16_t channels[CRSF_MAX_CHANNELS] {};
        const uint8_t nchan = MIN(NUM_SERVO_CHANNELS, (uint8_t)CRSF_MAX_CHANNELS);

        for (uint8_t i = 0; i < nchan; ++i) {
            SRV_Channel *c = SRV_Channels::srv_channel(i);
            if (c != nullptr) {
                channels[i] = c->get_output_pwm();
            } else {
                channels[i] = 1500; // Default to neutral if channel is null
            }
        }

#ifdef CRSF_RCOUT_DEBUG
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_update_debug_ms > 1000) {
            last_update_debug_ms = now_ms;
            debug_rcout("Updating channels. CH1=%u CH2=%u CH3=%u", channels[0], channels[1], channels[2]);
        }
#endif
        _crsf_port->send_rc_channels(channels, nchan);
        break;
    }
}

AP_CRSF_Out* AP_CRSF_Out::get_singleton()
{
    if (!_singleton) {
        _singleton = new AP_CRSF_Out();
    }
    return _singleton;
}

namespace AP {
    AP_CRSF_Out* crsf_out() {
        return AP_CRSF_Out::get_singleton();
    }
};

#endif // AP_CRSF_OUT_ENABLED
