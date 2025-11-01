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

#define CRSF_FRAMELEN_MAX   64U      // maximum possible framelength
#define CRSF_HEADER_LEN     2U       // header length
#define CRSF_FRAME_PAYLOAD_MAX (CRSF_FRAMELEN_MAX - CRSF_HEADER_LEN)     // maximum size of the frame length field in a packet

#include <stdint.h>

class AP_CRSF_Protocol {
public:

    enum FrameType {
        CRSF_FRAMETYPE_GPS = 0x02,
        CRSF_FRAMETYPE_VARIO = 0x07,
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
        CRSF_FRAMETYPE_BARO_VARIO = 0x09,
        CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
        CRSF_FRAMETYPE_VTX = 0x0F,
        CRSF_FRAMETYPE_VTX_TELEM = 0x10,
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED_11BIT = 0x18,
        CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
        CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
        CRSF_FRAMETYPE_ATTITUDE = 0x1E,
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
        // Extended Header Frames, range: 0x28 to 0x96
        CRSF_FRAMETYPE_PARAM_DEVICE_PING = 0x28,
        CRSF_FRAMETYPE_PARAM_DEVICE_INFO = 0x29,
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
        CRSF_FRAMETYPE_COMMAND = 0x32,
        // Custom Telemetry Frames 0x7F,0x80
        CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY = 0x7F,   // as suggested by Remo Masina for fw < 4.06
        CRSF_FRAMETYPE_AP_CUSTOM_TELEM = 0x80,          // reserved for ArduPilot by TBS, requires fw >= 4.06
    };

    // Command IDs for CRSF_FRAMETYPE_COMMAND
    enum CommandID {
        CRSF_COMMAND_FC = 0x01,
        CRSF_COMMAND_BLUETOOTH = 0x03,
        CRSF_COMMAND_OSD = 0x05,
        CRSF_COMMAND_VTX = 0x08,
        CRSF_COMMAND_LED = 0x09,
        CRSF_COMMAND_GENERAL = 0x0A,
        CRSF_COMMAND_RX = 0x10,
    };

    enum DeviceAddress {
        CRSF_ADDRESS_BROADCAST = 0x00,
        CRSF_ADDRESS_SYNC_BYTE = 0xC8,
        CRSF_ADDRESS_USB = 0x10,
        CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
        CRSF_ADDRESS_RESERVED1 = 0x8A,
        CRSF_ADDRESS_PNP_PRO_CURRENT_SENSOR = 0xC0,
        CRSF_ADDRESS_PNP_PRO_GPS = 0xC2,
        CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
        CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
        CRSF_ADDRESS_RESERVED2 = 0xCA,
        CRSF_ADDRESS_RACE_TAG = 0xCC,
        CRSF_ADDRESS_VTX = 0xCE,
        CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
        CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
        CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
    };

    // Commands for CRSF_COMMAND_GENERAL
    enum CommandGeneral {
        CRSF_COMMAND_GENERAL_CHILD_DEVICE_REQUEST = 0x04,
        CRSF_COMMAND_GENERAL_CHILD_DEVICE_FRAME = 0x05,
        CRSF_COMMAND_GENERAL_FIRMWARE_UPDATE_BOOTLOADER = 0x0A,
        CRSF_COMMAND_GENERAL_FIRMWARE_UPDATE_ERASE = 0x0B,
        CRSF_COMMAND_GENERAL_WRITE_SERIAL_NUMBER = 0x13,
        CRSF_COMMAND_GENERAL_USER_ID = 0x15,
        CRSF_COMMAND_GENERAL_SOFTWARE_PRODUCT_KEY = 0x60,
        CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,    // proposed new CRSF port speed
        CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE = 0x71,    // response to the proposed CRSF port speed
    };

    enum class ProtocolType {
        PROTOCOL_CRSF,
        PROTOCOL_TRACER,
        PROTOCOL_ELRS
    };

    struct Frame {
        uint8_t device_address;
        uint8_t length;
        uint8_t type;
        uint8_t payload[CRSF_FRAME_PAYLOAD_MAX - 1]; // type is already accounted for
    } PACKED;

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

    static const char* get_frame_type(uint8_t byte, uint8_t subtype = 0);

    // decode channels from the standard 11bit format (CRSFv2)
    static void decode_11bit_channels(const uint8_t* payload, uint8_t nchannels, uint16_t *values);

    // decode channels from variable bit length format (CRSFv3)
    static void decode_variable_bit_channels(const uint8_t* payload, uint8_t frame_length, uint8_t nchannels, uint16_t *values);

    // encode channels into a variable bit length format (CRSFv3)
    // returns number of bytes written to payload
    static uint8_t encode_variable_bit_channels(uint8_t *payload, const uint16_t *values, uint8_t nchannels);

    // process a device info frame for version information
    static bool process_device_info_frame(ParameterDeviceInfoFrame* info, VersionInfo* version, bool fakerx);

    // encode a device info frame for version information
    static uint32_t encode_device_info(ParameterDeviceInfoFrame& info, uint8_t num_params);

    static void encode_ping_frame(Frame& frame, DeviceAddress destination, DeviceAddress origin);

    static void encode_speed_proposal(Frame& frame, uint32_t baudrate, DeviceAddress destination, DeviceAddress origin);
};

#endif // AP_RCPROTOCOL_CRSF_ENABLED
