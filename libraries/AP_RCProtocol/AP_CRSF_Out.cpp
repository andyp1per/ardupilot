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
 */
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#if AP_CRSF_OUT_ENABLED

#include "AP_CRSF_Out.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>

const AP_Param::GroupInfo AP_CRSF_Out::var_info[] = {
    AP_GROUPINFO("RATE",  1, AP_CRSF_Out, _rate_hz, 50),
    AP_GROUPEND
};

AP_CRSF_Out* AP_CRSF_Out::_singleton;

AP_CRSF_Out::AP_CRSF_Out(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

// Initialise the CRSF output driver
void AP_CRSF_Out::init()
{
    if (_initialised) {
        return;
    }

    _crsf_port = AP_RCProtocol_CRSF::get_direct_attach_singleton(AP_SerialManager::SerialProtocol_CRSF_Output, 0);

    if (_crsf_port == nullptr) {
        // not configured on any port
        return;
    }

    const uint16_t rate = _rate_hz.get();
    if (rate > 0) {
        _frame_interval_us = 1000000UL / rate;
    } else {
        _frame_interval_us = 20000; // 50Hz default
    }

    _initialised = true;
}

// Main update call, sends RC frames at the configured rate
void AP_CRSF_Out::update()
{
    if (!_initialised) {
        return;
    }

    const uint32_t now = AP_HAL::micros();
    if (now - _last_frame_us < _frame_interval_us) {
        return;
    }
    _last_frame_us = now;

    uint16_t channels[16] {};
    const uint8_t nchan = MIN(NUM_SERVO_CHANNELS, (uint8_t)16);

    for (uint8_t i = 0; i < nchan; ++i) {
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c != nullptr) {
            channels[i] = c->get_output_pwm();
        }
    }

    _crsf_port->send_rc_channels(channels, nchan);
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
