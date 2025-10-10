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
    // CRSF_FRAMETYPE_COMMAND
    struct PACKED CommandFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t command_id;
        uint8_t payload[9]; // 8 maximum for LED command + crc8
    };

    // decode channels from the standard 11bit format (CRSFv2)
    static void decode_11bit_channels(const uint8_t* payload, uint8_t nchannels, uint16_t *values);

    // decode channels from variable bit length format (CRSFv3)
    static void decode_variable_bit_channels(const uint8_t* payload, uint8_t frame_length, uint8_t nchannels, uint16_t *values);

    // encode channels into a variable bit length format (CRSFv3)
    // returns number of bytes written to payload
    static uint8_t encode_variable_bit_channels(uint8_t *payload, const uint16_t *values, uint8_t nchannels);
};

#endif // AP_RCPROTOCOL_CRSF_ENABLED
