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
  CRSF protocol decoder based on betaflight implementation
  Code by Andy Piper
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_CRSF_ENABLED

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_CRSF.h"
#include "AP_CRSF_Protocol.h"
#include "AP_CRSF_Out.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_STARTING_CHANNEL_MASK        0x1F
#define CRSF_SUBSET_RC_RES_CONFIGURATION_BITS       2
#define CRSF_SUBSET_RC_RES_CONFIGURATION_MASK       0x03
#define CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS  1

#define CRSF_RC_CHANNEL_SCALE_LEGACY                0.62477120195241f
#define CRSF_SUBSET_RC_RES_CONF_10B                 0
#define CRSF_SUBSET_RC_RES_BITS_10B                 10
#define CRSF_SUBSET_RC_RES_MASK_10B                 0x03FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_10B            1.0f
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF
#define CRSF_SUBSET_RC_RES_CONF_12B                 2
#define CRSF_SUBSET_RC_RES_BITS_12B                 12
#define CRSF_SUBSET_RC_RES_MASK_12B                 0x0FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_12B            0.25f
#define CRSF_SUBSET_RC_RES_CONF_13B                 3
#define CRSF_SUBSET_RC_RES_BITS_13B                 13
#define CRSF_SUBSET_RC_RES_MASK_13B                 0x1FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_13B            0.125f

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 416666 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

extern const AP_HAL::HAL& hal;

//#define CRSF_DEBUG
//#define CRSF_DEBUG_CHARS
//#define CRSF_DEBUG_TELEM
//#define CRSF_DEBUG_PARAMS
#if defined(CRSF_DEBUG) || defined(CRSF_DEBUG_TELEM) || defined(CRSF_DEBUG_PARAMS)
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
static const char* get_frame_type(uint8_t byte, uint8_t subtype = 0)
{
    switch(byte) {
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_GPS:
        return "GPS";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_BATTERY_SENSOR:
        return "BATTERY";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_HEARTBEAT:
        return "HEARTBEAT";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX:
        return "VTX";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX_TELEM:
        return "VTX_TELEM";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        return "PING";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND:
        return "COMMAND";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_ATTITUDE:
        return "ATTITUDE";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_FLIGHT_MODE:
        return "FLIGHT_MODE";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        return "DEVICE_INFO";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ:
        return "PARAM_READ";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        return "SETTINGS_ENTRY";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_LINK_STATISTICS:
        return "LINK_STATS";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        return "RC";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
        return "RCv3";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_RC_CHANNELS_PACKED_11BIT:
        return "RCv3_11BIT";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_LINK_STATISTICS_RX:
        return "LINK_STATSv3_RX";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_LINK_STATISTICS_TX:
        return "LINK_STATSv3_TX";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_WRITE:
        return "PARAM_WRITE";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY:
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_AP_CUSTOM_TELEM:
        switch (subtype) {
        case AP_RCProtocol_CRSF::CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_SINGLE";
        case AP_RCProtocol_CRSF::CRSF_AP_CUSTOM_TELEM_STATUS_TEXT:
            return "AP_CUSTOM_TEXT";
        case AP_RCProtocol_CRSF::CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_MULTI";
        }
        return "AP_CUSTOM";
    }
    return "UNKNOWN";
}
#else
# define debug(fmt, args...)	do {} while(0)
#endif

#define CRSF_FRAME_TIMEOUT_US      50000U // 50ms to account for failure of the frame sync and long scheduling delays
#define CRSF_INTER_FRAME_TIME_US_250HZ    4000U // At fastest, frames are sent by the transmitter every 4 ms, 250 Hz
#define CRSF_INTER_FRAME_TIME_US_150HZ    6667U // At medium, frames are sent by the transmitter every 6.667 ms, 150 Hz
#define CRSF_INTER_FRAME_TIME_US_50HZ    20000U // At slowest, frames are sent by the transmitter every 20ms, 50 Hz
#define CRSF_HEADER_TYPE_LEN     (CRSF_HEADER_LEN + 1)           // header length including type

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_BAUDRATE_1MBIT      1000000U
#define CRSF_BAUDRATE_2MBIT      2000000U

const uint16_t AP_RCProtocol_CRSF::RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES] = {
    4, 50, 150, 250,    // CRSF
    4, 25, 50, 100, 100, 150, 200, 250, 333, 500, 250, 500, 500, 1000, 50  // ELRS
};

// Manager for CRSF instances
AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::Manager_State::_instances[HAL_NUM_SERIAL_PORTS];
AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::Manager_State::_rcin_singleton = nullptr;
uint32_t AP_RCProtocol_CRSF::Manager_State::_last_manager_check_ms;

// Manager init to find all configured CRSF ports
void AP_RCProtocol_CRSF::manager_init()
{
    if (AP_HAL::millis() - Manager_State::_last_manager_check_ms < 1000) {
        return;
    }

    AP_SerialManager &serial_manager = AP::serialmanager();

    for (uint8_t i = 0; i < SERIALMANAGER_MAX_PORTS; i++) {
        const AP_SerialManager::UARTState* port_state = serial_manager.get_state_by_id(i);
        if (port_state == nullptr) continue;
        AP_SerialManager::SerialProtocol protocol = port_state->get_protocol();
        AP_HAL::UARTDriver* uart = serial_manager.get_serial_by_id(i);
        PortMode mode;

        if (protocol == AP_SerialManager::SerialProtocol_CRSF) {
            mode = PortMode::DIRECT_VTX;
#if AP_CRSF_OUT_ENABLED
        } else if (protocol == AP_SerialManager::SerialProtocol_CRSF_Output) {
            mode = PortMode::DIRECT_RCOUT;
#endif
        } else {
            continue;
        }

        if (uart != nullptr) {
            // prevent creating duplicate instances
            if (Manager_State::_instances[i] == nullptr) {
                Manager_State::_instances[i] = new AP_RCProtocol_CRSF(AP::RC(), mode, uart);
#if AP_CRSF_OUT_ENABLED
                if (protocol == AP_SerialManager::SerialProtocol_CRSF_Output) {
                    AP::crsf_out()->init();
                }
#endif
            }
        }
    }
    Manager_State::_last_manager_check_ms = AP_HAL::millis();
}

// Manager update to poll all direct-attach ports
void AP_RCProtocol_CRSF::manager_update()
{
    // check for new ports on each call to update() to allow for runtime changes
    manager_init();

    for (uint8_t i = 0; i < HAL_NUM_SERIAL_PORTS; i++) {
        if (Manager_State::_instances[i] != nullptr && Manager_State::_instances[i]->_mode != PortMode::PASSTHROUGH_RCIN) {
            Manager_State::_instances[i]->update();
#if AP_CRSF_OUT_ENABLED
            AP::crsf_out()->update();
#endif
        }
    }
}

// constructor for RCIN "passthrough" mode
AP_RCProtocol_CRSF::AP_RCProtocol_CRSF(AP_RCProtocol &_frontend) :
    AP_RCProtocol_Backend(_frontend),
    _mode(PortMode::PASSTHROUGH_RCIN),
    _uart(nullptr)
{
    _protocol_helper = new AP_CRSF_Protocol();
    // This is the RCIN instance, register it as the singleton
    Manager_State::_rcin_singleton = this;
    _baud_negotiation_result = BaudNegotiationResult::PENDING;
}

// constructor for "direct-attach" modes
AP_RCProtocol_CRSF::AP_RCProtocol_CRSF(AP_RCProtocol &_frontend, PortMode mode, AP_HAL::UARTDriver* uart) :
    AP_RCProtocol_Backend(_frontend),
    _mode(mode),
    _uart(uart)
{
    _protocol_helper = new AP_CRSF_Protocol();
    start_uart();
    _baud_negotiation_result = BaudNegotiationResult::PENDING;
}

AP_RCProtocol_CRSF::~AP_RCProtocol_CRSF()
{
    delete _protocol_helper;
    for (uint8_t i = 0; i < HAL_NUM_SERIAL_PORTS; i++) {
        if (Manager_State::_instances[i] == this) {
            Manager_State::_instances[i] = nullptr;
        }
    }
    if (Manager_State::_rcin_singleton == this) {
        Manager_State::_rcin_singleton = nullptr;
    }
}

// get the protocol string
const char* AP_RCProtocol_CRSF::get_protocol_string(ProtocolType protocol) const {
    if (protocol == ProtocolType::PROTOCOL_ELRS) {
        return "ELRS";
    } else if (_crsf_v3_active) {
        return "CRSFv3";
    } else {
        return "CRSFv2";
    }
}

// return the link rate as defined by the LinkStatistics
uint16_t AP_RCProtocol_CRSF::get_link_rate(ProtocolType protocol) const {
    if (protocol == ProtocolType::PROTOCOL_ELRS) {
        return RF_MODE_RATES[_link_status.rf_mode + RFMode::CRSF_RF_MAX_MODES];
    } else if (protocol == ProtocolType::PROTOCOL_TRACER) {
        return 250;
    } else {
        return RF_MODE_RATES[_link_status.rf_mode];
    }
}

// process a byte provided by a uart from rc stack (Passthrough mode)
void AP_RCProtocol_CRSF::process_byte(uint8_t byte, uint32_t baudrate)
{
    // reject RC data if we have been configured for standalone mode
    if (_mode != PortMode::PASSTHROUGH_RCIN || (baudrate != CRSF_BAUDRATE && baudrate != CRSF_BAUDRATE_1MBIT && baudrate != CRSF_BAUDRATE_2MBIT)) {
        return;
    }
    _process_byte(byte);
}

// process a byte from any source
void AP_RCProtocol_CRSF::_process_byte(uint8_t byte)
{
    //debug("process_byte(0x%x)", byte);
    const uint32_t now = AP_HAL::micros();

    // extra check for overflow, should never happen since it will have been handled in check_frame()
    if (_frame_ofs >= sizeof(_frame)) {
        _frame_ofs = 0;
    }

    // check for long frame gaps
    // we took too long decoding, start again - the RX will only send complete frames so this is unlikely to fail,
    // however thread scheduling can introduce longer delays even when the data has been received
    if (_frame_ofs > 0 && (now - _start_frame_time_us) > CRSF_FRAME_TIMEOUT_US) {
        _frame_ofs = 0;
    }

    // start of a new frame
    if (_frame_ofs == 0) {
        _start_frame_time_us = now;
    }
    
    _frame_bytes[_frame_ofs++] = byte;
    
    if (!check_frame(now)) {
        skip_to_next_frame(now);
    }
}

// check if a frame is valid. Return false if the frame is definitely
// invalid. Return true if we need more bytes
bool AP_RCProtocol_CRSF::check_frame(uint32_t timestamp_us)
{
    // overflow check
    if (_frame_ofs >= sizeof(_frame)) {
        return false;
    }

    // need a header to get the length
    if (_frame_ofs < CRSF_HEADER_TYPE_LEN) {
        return true;
    }

#if AP_CRSF_OUT_ENABLED
    // in RCOUT mode, we are the master, so frames will be addressed from the FC
    if (_mode == PortMode::DIRECT_RCOUT) {
        if (_frame.device_address != DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER &&
            _frame.device_address != DeviceAddress::CRSF_ADDRESS_RADIO_TRANSMITTER) {
            return false;
        }
    } else
#endif
    {
        if (_frame.device_address != DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            return false;
        }
    }

    // check validity of the length byte if we have received it
    if (_frame_ofs >= CRSF_HEADER_TYPE_LEN &&
        _frame.length > CRSF_FRAME_PAYLOAD_MAX) {
        return false;
    }

    if (_frame.length < CRSF_FRAME_LENGTH_MIN) {
        // invalid short frame
        return false;
    }

    // decode whatever we got and expect
    if (_frame_ofs >= _frame.length + CRSF_HEADER_LEN) {
        const uint8_t crc = crc8_dvb_s2_update(0, &_frame_bytes[CRSF_HEADER_LEN], _frame.length - 1);

        //debug("check_frame(0x%x, 0x%x)", _frame.device_address, _frame.length);

        if (crc != _frame.payload[_frame.length - 2]) {
            return false;
        }

        log_data(AP_RCProtocol::CRSF, timestamp_us, (const uint8_t*)&_frame, _frame.length + CRSF_HEADER_LEN);

        // decode here
        if (decode_crsf_packet()) {
            _last_tx_frame_time_us = timestamp_us;  // we have received a frame from the transmitter
            add_input(MAX_CHANNELS, _channels, false, _link_status.rssi, _link_status.link_quality);
        }

        // we consumed the frame
        const auto len = _frame.length + CRSF_HEADER_LEN;
        _frame_ofs -= len;
        if (_frame_ofs > 0) {
            memmove(_frame_bytes, _frame_bytes+len, _frame_ofs);
        }

        _last_frame_time_us = _last_rx_frame_time_us = timestamp_us;

        return true;
    }

    // more bytes to come
    return true;
}

// called when parsing or CRC fails on a frame
void AP_RCProtocol_CRSF::skip_to_next_frame(uint32_t timestamp_us)
{
    if (_frame_ofs <= 1) {
        return;
    }

    /*
      look for a frame header in the remaining bytes
     */
    const uint8_t *new_header = (const uint8_t *)memchr(&_frame_bytes[1], DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, _frame_ofs - 1);
    if (new_header == nullptr) {
        _frame_ofs = 0;
        return;
    }

    /*
      setup the current state as the remaining bytes, if any
     */
    _frame_ofs -= (new_header - _frame_bytes);
    if (_frame_ofs > 0) {
        memmove(_frame_bytes, new_header, _frame_ofs);
    }

    _start_frame_time_us = timestamp_us;

    // we could now have a good frame
    check_frame(timestamp_us);
}

void AP_RCProtocol_CRSF::update(void)
{
    // if we are in direct attach mode, process data from the uart
    if (_mode != PortMode::PASSTHROUGH_RCIN) {
        uint32_t now = AP_HAL::millis();
        // for some reason it's necessary to keep trying to start the uart until we get data
        if (now - _last_uart_start_time_ms > 1000U && _last_frame_time_us == 0) {
            start_uart();
            _last_uart_start_time_ms = now;
        }
        if (_uart) {
            uint32_t n = _uart->available();
            for (uint32_t i = 0; i < n; i++) {
                int16_t b = _uart->read();
                if (b >= 0) {
                    _process_byte(uint8_t(b));
                }
            }
        }
    }

    // never received RC frames, but have received CRSF frames so make sure we give the telemetry opportunity to run
    uint32_t now = AP_HAL::micros();
    if ((_last_frame_time_us > 0
#if HAL_CRSF_TELEM_ENABLED
         || bind_in_progress()
#endif
        ) && (!get_rc_input_count() || !is_tx_active())
        && now - _last_frame_time_us > CRSF_INTER_FRAME_TIME_US_250HZ) {
        // don't send telemetry unless the UART we are dealing with is configured to send it
        AP_HAL::UARTDriver *uart = get_available_UART();
        if (_uart || (uart && (uart->get_baud_rate() == CRSF_BAUDRATE || uart->get_baud_rate() == ELRS_BAUDRATE))) {
            process_telemetry(false);
            _last_frame_time_us = now;
        }
    }

#if AP_RC_CHANNEL_ENABLED
    //Check if LQ is to be reported in place of RSSI
    _use_lq_for_rssi = rc().option_is_enabled(RC_Channels::Option::USE_CRSF_LQ_AS_RSSI);
#endif
}

// write out a frame of any type
void AP_RCProtocol_CRSF::write_frame(Frame* frame) const
{
    AP_HAL::UARTDriver *uart = get_current_UART();

    if (!uart) {
        return;
    }
    // calculate crc
    frame->payload[frame->length - 2] = crc8_dvb_s2_update(0, &frame->type, frame->length-1);

    uart->write((uint8_t*)frame, frame->length + 2);
    uart->flush();

#if defined(CRSF_DEBUG) || defined(CRSF_DEBUG_PARAMS)
#ifdef CRSF_DEBUG_PARAMS
    switch (frame->type) {
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        case CRSF_FRAMETYPE_PARAMETER_READ:
        case CRSF_FRAMETYPE_PARAMETER_WRITE:
        case CRSF_FRAMETYPE_COMMAND:
#endif

        hal.console->printf("CRSF: writing %s:", get_frame_type(frame->type, frame->payload[0]));
        for (uint8_t i = 0; i < frame->length + 2; i++) {
            uint8_t val = ((uint8_t*)frame)[i];
#ifdef CRSF_DEBUG_CHARS
            if (val >= 32 && val <= 126) {
                hal.console->printf(" 0x%x '%c'", val, (char)val);
            } else {
#endif
                hal.console->printf(" 0x%x", val);
#ifdef CRSF_DEBUG_CHARS
            }
#endif
        }
        hal.console->printf("\n");
#ifdef CRSF_DEBUG_PARAMS
    }
#endif
#endif // defined(CRSF_DEBUG) || defined(CRSF_DEBUG_PARAMS)
}

bool AP_RCProtocol_CRSF::decode_crsf_packet()
{
#ifdef CRSF_DEBUG
    hal.console->printf("CRSF: received %s:", get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
#ifdef CRSF_DEBUG_CHARS
        if (fptr[i] >= 32 && fptr[i] <= 126) {
            hal.console->printf(" 0x%x '%c'", fptr[i], (char)fptr[i]);
        } else {
#endif
            hal.console->printf(" 0x%x", fptr[i]);
#ifdef CRSF_DEBUG_CHARS
        }
#endif
    }
    hal.console->printf("\n");
#endif

    bool rc_active = false;

    switch (_frame.type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if (_mode == PortMode::PASSTHROUGH_RCIN) {
                // scale factors defined by TBS - TICKS_TO_US(x) ((x - 992) * 5 / 8 + 1500)
                AP_CRSF_Protocol::decode_11bit_channels((const uint8_t*)(&_frame.payload), MAX_CHANNELS, _channels);
                _crsf_v3_active = false;
                rc_active = true;
            }
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            process_link_stats_frame((uint8_t*)&_frame.payload);
            break;
        case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
            if (_mode == PortMode::PASSTHROUGH_RCIN) {
                AP_CRSF_Protocol::decode_variable_bit_channels((const uint8_t*)(&_frame.payload), _frame.length, MAX_CHANNELS, _channels);
                _crsf_v3_active = true;
                rc_active = true;
            }
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS_RX:
            process_link_stats_rx_frame((uint8_t*)&_frame.payload);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS_TX:
            process_link_stats_tx_frame((uint8_t*)&_frame.payload);
            break;
        case CRSF_FRAMETYPE_COMMAND: {
            const AP_CRSF_Protocol::CommandFrame* cmd = (const AP_CRSF_Protocol::CommandFrame*)&_frame.payload[-1]; // cmd starts at type
            if (cmd->command_id == CRSF_COMMAND_GENERAL && cmd->payload[0] == CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE) {
                const bool success = cmd->payload[2];
                if (success) {
                    _baud_negotiation_result = BaudNegotiationResult::SUCCESS;
                    // change baud on our end now
                    change_baud_rate(_new_baud_rate);
                } else {
                    _baud_negotiation_result = BaudNegotiationResult::FAILED;
                }
            }
            break;
        }
        default:
            break;
    }
#if HAL_CRSF_TELEM_ENABLED
#if AP_CRSF_OUT_ENABLED
    // RC Out mode does not use the ArduPilot telemetry system, but can parse link stats
    if (_mode == PortMode::DIRECT_RCOUT) {
        return rc_active; // will be false, which is correct
    }
#endif

    if (AP_CRSF_Telem::process_frame(FrameType(_frame.type), (uint8_t*)&_frame.payload, _frame.length - 2U)) {
#if defined(CRSF_DEBUG_TELEM) || defined(CRSF_DEBUG_PARAMS)
        switch (_frame.type) {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_LINK_STATISTICS:
            case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_LINK_STATISTICS_RX:
            case CRSF_FRAMETYPE_LINK_STATISTICS_TX:
                break;
#ifdef CRSF_DEBUG_PARAMS
            case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            case CRSF_FRAMETYPE_PARAMETER_READ:
            case CRSF_FRAMETYPE_PARAMETER_WRITE:
            case CRSF_FRAMETYPE_COMMAND:
#else
            default:
#endif
                hal.console->printf("CRSF: received %s:", get_frame_type(_frame.type));
                uint8_t* fptr = (uint8_t*)&_frame;
                for (uint8_t i = 0; i < _frame.length + 2; i++) {
                    hal.console->printf(" 0x%x", fptr[i]);
                }
                hal.console->printf("\n");
        }
#endif // defined(CRSF_DEBUG_TELEM) || defined(CRSF_DEBUG_PARAMS)
        process_telemetry();
    }
    // process any pending baudrate changes before reading another frame
    if (_new_baud_rate > 0) {
        AP_HAL::UARTDriver *uart = get_current_UART();

        if (uart) {
            // wait for all the pending data to be sent
            while (uart->tx_pending()) {
                hal.scheduler->delay_microseconds(10);
            }
            // now wait for 4ms to account for RX transmission and processing
            hal.scheduler->delay(4);
            // change the baud rate
            uart->begin(_new_baud_rate);
        }
        _new_baud_rate = 0;
    }
#endif

    return rc_active;
}

#if AP_CRSF_OUT_ENABLED
// send RC channel frame out
bool AP_RCProtocol_CRSF::send_rc_channels(const uint16_t* channels, uint8_t nchannels)
{
    if (_mode != PortMode::DIRECT_RCOUT) {
        return false;
    }
    Frame frame;
    frame.device_address = DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER;
    frame.type = CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED;
    uint8_t payload_len = _protocol_helper->encode_variable_bit_channels(frame.payload, channels, nchannels);
    frame.length = payload_len + 2; // +1 for type, +1 for CRC
    write_frame(&frame);
    return true;
}

// send a baudrate proposal
void AP_RCProtocol_CRSF::send_speed_proposal(uint32_t baudrate)
{
    if (_mode != PortMode::DIRECT_RCOUT || !_uart) {
        return;
    }
    _new_baud_rate = baudrate;

    Frame frame;
    frame.device_address = DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER;

    AP_CRSF_Protocol::CommandFrame* cmd = (AP_CRSF_Protocol::CommandFrame*)&frame.type;
    cmd->destination = DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER;
    cmd->origin = DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    cmd->command_id = CRSF_COMMAND_GENERAL;
    cmd->payload[0] = CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL;
    cmd->payload[1] = 0; // port ID, 0 for UART
    // payload[2-5] is the baudrate
    uint32_t baud_be = htobe32(baudrate);
    memcpy(&cmd->payload[2], &baud_be, sizeof(baud_be));

    const uint8_t payload_len = 6; // subcommand + port_id + baudrate
    const uint8_t cmd_len = 3 + payload_len + 1; // dest, origin, cmd_id, payload, crc
    frame.type = CRSF_FRAMETYPE_COMMAND;
    frame.length = cmd_len + 1; // +1 for type

    // calculate command crc which includes frame type
    uint8_t* crcptr = &frame.type;
    uint8_t crc = 0;
    // CRC is over type, dest, origin, cmd_id, and payload
    for (uint8_t i=0; i< (4+payload_len); i++) {
        crc = crc8_dvb(crc, crcptr[i], 0xBA);
    }
    frame.payload[payload_len + 2] = crc;

    write_frame(&frame);
}
#endif

// send out telemetry
bool AP_RCProtocol_CRSF::process_telemetry(bool check_constraint) const
{
#if HAL_CRSF_TELEM_ENABLED
    AP_HAL::UARTDriver *uart = get_current_UART();
    if (!uart) {
        return false;
    }

    if (!telem_available) {
        if (AP_CRSF_Telem::get_telem_data(this, (Frame*)&_telemetry_frame, is_tx_active())) {
            telem_available = true;
        } else {
            return false;
        }
    }
    write_frame((Frame*)&_telemetry_frame);
    // get fresh telem_data in the next call
    telem_available = false;

    return true;
#else
    return false;
#endif
}

#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
    // Define the static tx powers array
    constexpr uint16_t AP_RCProtocol_CRSF::tx_powers[];
#endif

// process link statistics to get RSSI
void AP_RCProtocol_CRSF::process_link_stats_frame(const void* data)
{
    const LinkStatisticsFrame* link = (const LinkStatisticsFrame*)data;

    uint8_t rssi_dbm;
    if (link->active_antenna == 0) {
        rssi_dbm = link->uplink_rssi_ant1;
    } else {
        rssi_dbm = link->uplink_rssi_ant2;
    }
    _link_status.link_quality = link->uplink_status;

    if (_use_lq_for_rssi) {
        _link_status.rssi = derive_scaled_lq_value(link->uplink_status);
    } else{
        // AP rssi: -1 for unknown, 0 for no link, 255 for maximum link
        if (rssi_dbm < 50) {
            _link_status.rssi = 255;
        } else if (rssi_dbm > 120) {
            _link_status.rssi = 0;
        } else {
            // this is an approximation recommended by Remo from TBS
            _link_status.rssi = int16_t(roundf((1.0f - (rssi_dbm - 50.0f) / 70.0f) * 255.0f));
        }
    }

    // Define the max number of RFModes based on ELRS modes, which is larger than Crossfire
    const uint8_t max_modes = (RFMode::RF_MODE_MAX_MODES - RFMode::CRSF_RF_MAX_MODES) - 1U; // Subtract 1 due to zero-indexing
    _link_status.rf_mode = MIN(link->rf_mode, max_modes); // Cap to avoid memory spills in the conversion tables

#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
    // Populate the extra data items
    if (link->uplink_status > 0) {
        _link_status.rssi_dbm = rssi_dbm;
        _link_status.tx_power = -1;
        if (link->uplink_tx_power < ARRAY_SIZE(AP_RCProtocol_CRSF::tx_powers)) {
            _link_status.tx_power = AP_RCProtocol_CRSF::tx_powers[link->uplink_tx_power];
        }
        _link_status.snr = link->uplink_snr;
        _link_status.active_antenna = link->active_antenna;
    } else {
        // This means LQ is zero, so set all values to "no signal" state
        _link_status.rssi_dbm = -1;
        _link_status.tx_power = -1;
        _link_status.snr = INT8_MIN;
        _link_status.active_antenna = -1;
    }
#endif
}

// process link statistics to get RX RSSI
void AP_RCProtocol_CRSF::process_link_stats_rx_frame(const void* data)
{
    const LinkStatisticsRXFrame* link = (const LinkStatisticsRXFrame*)data;

    if (_use_lq_for_rssi) {
        _link_status.rssi = derive_scaled_lq_value(link->link_quality);
    } else {
        _link_status.rssi = link->rssi_percent * 255.0f * 0.01f;
    }
}

// process link statistics to get TX RSSI
void AP_RCProtocol_CRSF::process_link_stats_tx_frame(const void* data)
{
    const LinkStatisticsTXFrame* link = (const LinkStatisticsTXFrame*)data;

    if (_use_lq_for_rssi) {
        _link_status.rssi = derive_scaled_lq_value(link->link_quality);
    } else {
        _link_status.rssi = link->rssi_percent * 255.0f * 0.01f;
    }
}

// start the uart if we have one
void AP_RCProtocol_CRSF::start_uart()
{
    if (!_uart) {
        return;
    }
    _uart->configure_parity(0);
    _uart->set_stop_bits(1);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_options(_uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
    _uart->begin(get_bootstrap_baud_rate());
}

// change the baudrate of the protocol if we are able
bool AP_RCProtocol_CRSF::change_baud_rate(uint32_t baudrate)
{
    AP_HAL::UARTDriver* uart = get_current_UART();
    if (uart == nullptr) {
        return false;
    }
#if !defined(STM32H7)
    if (baudrate > get_bootstrap_baud_rate() && !uart->is_dma_enabled()) {
        return false;
    }
#endif
    if (baudrate > CRSF_BAUDRATE_2MBIT) {
        return false;
    }

    _new_baud_rate = baudrate;

    return true;
}

// change the bootstrap baud rate to ELRS standard if configured
void AP_RCProtocol_CRSF::process_handshake(uint32_t baudrate)
{
    AP_HAL::UARTDriver *uart = get_available_UART();

    // only change the baudrate if we are bootstrapping CRSF
    if (_mode != PortMode::PASSTHROUGH_RCIN || uart == nullptr ||
        baudrate != CRSF_BAUDRATE
        || baudrate == get_bootstrap_baud_rate()
        || uart->get_baud_rate() == get_bootstrap_baud_rate()
        || !protocol_enabled(AP_RCProtocol::CRSF)) {
        return;
    }

    uart->begin(get_bootstrap_baud_rate());
}

//returns uplink link quality on 0-255 scale
int16_t AP_RCProtocol_CRSF::derive_scaled_lq_value(uint8_t uplink_lq)
{
    return int16_t(roundf(constrain_float(uplink_lq*2.5f,0,255)));
}

// start bind
#if HAL_CRSF_TELEM_ENABLED
void AP_RCProtocol_CRSF::start_bind(void)
{
    AP_CRSF_Telem* telem = AP::crsf_telem();
    if (telem != nullptr) {
        telem->start_bind();
    }
}

bool AP_RCProtocol_CRSF::bind_in_progress(void)
{
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    AP_CRSF_Telem* telem = AP::crsf_telem();
    if (telem != nullptr) {
        return telem->bind_in_progress();
    }
#endif
    return false;
}
#endif

// Static singleton accessor for RCIN (legacy)
AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::get_rcin_singleton()
{
    // The RCIN instance is created by the frontend and registers itself.
    return Manager_State::_rcin_singleton;
}

#if AP_CRSF_OUT_ENABLED
// Static singleton accessor for direct attach ports
AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::get_direct_attach_singleton(AP_SerialManager::SerialProtocol protocol, uint8_t instance)
{
    manager_init(); // ensure manager is running
    const int8_t port_num = AP::serialmanager().find_portnum(protocol, instance);
    if (port_num >= 0 && port_num < HAL_NUM_SERIAL_PORTS) {
        return Manager_State::_instances[port_num];
    }
    return nullptr;
}
#endif

namespace AP {
    // legacy accessor
    AP_RCProtocol_CRSF* crsf() {
        return AP_RCProtocol_CRSF::get_rcin_singleton();
    }
};

#endif  // AP_RCPROTOCOL_CRSF_ENABLED
