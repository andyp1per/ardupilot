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
 * AP_CRSF_Protocol.cpp - a stateless parser and encoder for the CRSF wire protocol
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_CRSF_ENABLED

#include "AP_CRSF_Protocol.h"
#include "AP_RCProtocol_CRSF.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>

// Defines for CRSFv3 subset RC frame packing/unpacking
#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF

extern const AP_HAL::HAL& hal;

#define CRSF_PROTOCOL_DEBUG
#if defined(CRSF_PROTOCOL_DEBUG)
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

// unpack channels from a CRSFv2 11-bit fixed-width frame payload
void AP_CRSF_Protocol::decode_11bit_channels(const uint8_t* payload, uint8_t nchannels, uint16_t *values)
{
    // CRSFv2 channels are packed 11 bits each, 16 channels in 22 bytes.
    // This is the same packing as SBUS.
    // Explicitly cast to uint32_t to avoid narrowing conversion warnings.
    const uint32_t raw_channels[16] = {
        (( (uint32_t)payload[0]    | (uint32_t)payload[1] << 8) & 0x07FF),
        ((( (uint32_t)payload[1] >> 3 | (uint32_t)payload[2] << 5) & 0x07FF)),
        ((( (uint32_t)payload[2] >> 6 | (uint32_t)payload[3] << 2 | (uint32_t)payload[4] << 10) & 0x07FF)),
        ((( (uint32_t)payload[4] >> 1 | (uint32_t)payload[5] << 7) & 0x07FF)),
        ((( (uint32_t)payload[5] >> 4 | (uint32_t)payload[6] << 4) & 0x07FF)),
        ((( (uint32_t)payload[6] >> 7 | (uint32_t)payload[7] << 1 | (uint32_t)payload[8] << 9) & 0x07FF)),
        ((( (uint32_t)payload[8] >> 2 | (uint32_t)payload[9] << 6) & 0x07FF)),
        ((( (uint32_t)payload[9] >> 5 | (uint32_t)payload[10] << 3) & 0x07FF)),
        (( (uint32_t)payload[11]   | (uint32_t)payload[12] << 8) & 0x07FF),
        ((( (uint32_t)payload[12] >> 3 | (uint32_t)payload[13] << 5) & 0x07FF)),
        ((( (uint32_t)payload[13] >> 6 | (uint32_t)payload[14] << 2 | (uint32_t)payload[15] << 10) & 0x07FF)),
        ((( (uint32_t)payload[15] >> 1 | (uint32_t)payload[16] << 7) & 0x07FF)),
        ((( (uint32_t)payload[16] >> 4 | (uint32_t)payload[17] << 4) & 0x07FF)),
        ((( (uint32_t)payload[17] >> 7 | (uint32_t)payload[18] << 1 | (uint32_t)payload[19] << 9) & 0x07FF)),
        ((( (uint32_t)payload[18] >> 2 | (uint32_t)payload[19] << 6) & 0x07FF)),
        ((( (uint32_t)payload[20] >> 5 | (uint32_t)payload[21] << 3) & 0x07FF))
    };

    const uint8_t num_to_decode = MIN(nchannels, (uint8_t)16);
    for (uint8_t i = 0; i < num_to_decode; i++) {
        // scale from CRSFv2's native 172-1811 range to 1000-2000us PWM
        values[i] = (uint16_t)(((raw_channels[i] - 992) * 5 / 8) + 1500);
    }
}

// unpack channels from a CRSFv3 variable bit length frame payload
void AP_CRSF_Protocol::decode_variable_bit_channels(const uint8_t* payload, uint8_t frame_length, uint8_t nchannels, uint16_t *values)
{
    const AP_RCProtocol_CRSF::SubsetChannelsFrame* channel_data = (const AP_RCProtocol_CRSF::SubsetChannelsFrame*)payload;
    // We currently only support 11-bit resolution
    const uint8_t channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
    const uint16_t channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
    const float channelScale = 0.5f;

    // calculate the number of channels packed
    const uint8_t numOfChannels = MIN(uint8_t(((frame_length - 2) * 8 - CRSF_SUBSET_RC_STARTING_CHANNEL_BITS) / channelBits), (uint8_t)CRSF_MAX_CHANNELS);

    // unpack the channel data
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 1;

    for (uint8_t n = 0; n < numOfChannels; n++) {
        while (bitsMerged < channelBits) {
            if (readByteIndex >= CRSF_FRAME_PAYLOAD_MAX) {
                return;
            }
            uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        if (uint8_t(channel_data->starting_channel + n) >= nchannels) {
            return;
        }
        values[channel_data->starting_channel + n] =
            uint16_t(channelScale * float(uint16_t(readValue & channelMask)) + 988);
        readValue >>= channelBits;
        bitsMerged -= channelBits;
    }
}

// encode 16 channels of PWM values into a CRSFv3 variable bit length frame payload
uint8_t AP_CRSF_Protocol::encode_variable_bit_channels(uint8_t *payload, const uint16_t *values, uint8_t nchannels)
{
    // We will use 11-bit resolution which is standard for CRSFv3
    const uint8_t channelBits = CRSF_SUBSET_RC_RES_BITS_11B;

    AP_RCProtocol_CRSF::SubsetChannelsFrame* channel_data = (AP_RCProtocol_CRSF::SubsetChannelsFrame*)payload;
    memset(payload, 0, 23); // 1 byte header + 22 bytes for 16 channels

    channel_data->starting_channel = 0;
    channel_data->res_configuration = CRSF_SUBSET_RC_RES_CONF_11B;

    uint32_t writeValue = 0;
    uint8_t bitsMerged = 0;
    uint8_t writeByteIndex = 1;
    const uint8_t num_to_send = MIN(nchannels, (uint8_t)16);

    for (uint8_t n = 0; n < num_to_send; n++) {
        uint16_t channel_value = constrain_int16(lroundf((values[n] - 988) / 0.5f), 0, 2047);
        writeValue |= ((uint32_t)channel_value) << bitsMerged;
        bitsMerged += channelBits;
        while (bitsMerged >= 8) {
            if (writeByteIndex < 23) {
                payload[writeByteIndex++] = writeValue & 0xFF;
            }
            writeValue >>= 8;
            bitsMerged -= 8;
        }
    }
    if (bitsMerged > 0 && writeByteIndex < 23) {
        payload[writeByteIndex++] = writeValue & 0xFF;
    }

    return writeByteIndex;
}

// request for device info
bool AP_CRSF_Protocol::process_device_info_frame(ParameterDeviceInfoFrame* info, VersionInfo* version, bool fakerx)
{
    const uint8_t destination = fakerx ? AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER : AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    const uint8_t origin = fakerx ? AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER : AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER;

    if (info->destination != 0 && info->destination != destination) {
        return false; // request was not for us
    }

    // we are only interested in RC device info for firmware version detection
    if (info->origin != 0 && info->origin != origin) {
        return false;
    }

    /*
        Payload size is 58:
        char[] Device name ( Null-terminated string, max len is 42 )
        uint32_t Serial number
        uint32_t Hardware ID
        uint32_t Firmware ID (0x00:0x00:0xAA:0xBB AA=major, BB=minor)
        uint8_t Parameters count
        uint8_t Parameter version number
    */
    // get the terminator of the device name string
    const uint8_t offset = strnlen((char*)info->payload,42U);
    if (strncmp((char*)info->payload, "Tracer", 6) == 0) {
        version->protocol = ProtocolType::PROTOCOL_TRACER;
    } else if (strncmp((char*)&info->payload[offset+1], "ELRS", 4) == 0) {
        // ELRS magic number is ELRS encoded in the serial number
        // 0x45 'E' 0x4C 'L' 0x52 'R' 0x53 'S'
        version->protocol = ProtocolType::PROTOCOL_ELRS;
    }

    if (version->protocol != ProtocolType::PROTOCOL_ELRS) {
        /*
            fw major ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 3rd byte of sw id = 11bytes
            fw minor ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 4th byte of sw id = 12bytes
        */
        version->major = info->payload[offset+11];
        version->minor = info->payload[offset+12];
    } else {
        // ELRS does not populate the version field so cook up something sensible
        version->major = 1;
        version->minor = 0;
    }

    // should we use rf_mode reported by link statistics?
    if (version->protocol == ProtocolType::PROTOCOL_ELRS
        || (version->protocol != ProtocolType::PROTOCOL_TRACER
            && (version->major > 3 || (version->major == 3 && version->minor >= 72)))) {
        version->use_rf_mode = true;
    }

    debug("process_device_info_frame(): %u %u %u", version->major, version->minor, (unsigned)version->protocol);

    return true;
}

#endif // AP_RCPROTOCOL_CRSF_ENABLED

