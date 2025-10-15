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
 * AP_CRSF_Protocol.h - a stateless parser and encoder for the CRSF wire protocol
 */
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_CRSF_ENABLED

#include <stdint.h>

class AP_CRSF_Protocol {
public:
    enum class ProtocolType {
        PROTOCOL_CRSF,
        PROTOCOL_TRACER,
        PROTOCOL_ELRS
    };

    // CRSF_FRAMETYPE_COMMAND
    struct PACKED CommandFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t command_id;
        uint8_t payload[9]; // 8 maximum for LED command + crc8
    };

    // CRSF_FRAMETYPE_PARAM_DEVICE_PING
    struct PACKED ParameterPingFrame {
        uint8_t destination;
        uint8_t origin;
    };

    // CRSF_FRAMETYPE_PARAM_DEVICE_INFO
    struct PACKED ParameterDeviceInfoFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t payload[58];   // largest possible frame is 60
    };

    struct VersionInfo {
        uint8_t minor;
        uint8_t major;
        bool use_rf_mode;
        ProtocolType protocol;
    };

    // decode channels from the standard 11bit format (CRSFv2)
    static void decode_11bit_channels(const uint8_t* payload, uint8_t nchannels, uint16_t *values);

    // decode channels from variable bit length format (CRSFv3)
    static void decode_variable_bit_channels(const uint8_t* payload, uint8_t frame_length, uint8_t nchannels, uint16_t *values);

    // encode channels into a variable bit length format (CRSFv3)
    // returns number of bytes written to payload
    static uint8_t encode_variable_bit_channels(uint8_t *payload, const uint16_t *values, uint8_t nchannels);

    // process a device info frame for version information
    static bool process_device_info_frame(ParameterDeviceInfoFrame* info, VersionInfo* version, bool fakerx);
};

#endif // AP_RCPROTOCOL_CRSF_ENABLED
