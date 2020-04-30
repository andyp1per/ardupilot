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

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_SRXL.h"
#include "AP_RCProtocol_CRSF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_RCTelemetry/AP_CRSF.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>

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

// #define CRSF_DEBUG
#ifdef CRSF_DEBUG
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
static const char* get_frame_type(uint8_t byte)
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
    default:
        return "UNKNOWN";
    }
}
#else
# define debug(fmt, args...)	do {} while(0)
#endif

#define CRSF_MAX_FRAME_TIME_US      1100U // 700us + 400us for potential ad-hoc request
#define CRSF_INTER_FRAME_TIME_US    6667U // At fastest, frames are sent by the transmitter every 6.667 ms, 150 Hz
#define CSRF_HEADER_LEN     2

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811


AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::_singleton;

AP_RCProtocol_CRSF::AP_RCProtocol_CRSF(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Duplicate CRSF handler");
    }

    _singleton = this;
}

void AP_RCProtocol_CRSF::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

void AP_RCProtocol_CRSF::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    //debug("process_byte(0x%x)", byte);
    // we took too long decoding, start again
    if (_frame_ofs > 0 && (timestamp_us - _start_frame_time_us) > CRSF_MAX_FRAME_TIME_US) {
        _frame_ofs = 0;
    }

    // there will be at least a 5ms gap between successive CRSF frames. if we see it
    // assume we are starting a new frame
    if ((timestamp_us - _last_rx_time_us) > (CRSF_INTER_FRAME_TIME_US - CRSF_MAX_FRAME_TIME_US)) {
        _frame_ofs = 0;
    }

    _last_rx_time_us = timestamp_us;

    // overflow check
    if (_frame_ofs >= CRSF_FRAMELEN_MAX) {
        _frame_ofs = 0;
    }

    // start of a new frame
    if (_frame_ofs == 0) {
        _start_frame_time_us = timestamp_us;
    }

    add_to_buffer(_frame_ofs++, byte);

    // need a header to get the length
    if (_frame_ofs < CSRF_HEADER_LEN) {
        return;
    }

    // parse the length
    if (_frame_ofs == CSRF_HEADER_LEN) {
        // check for garbage frame
        if (_frame.length > CRSF_FRAMELEN_MAX) {
            _frame_ofs = 0;
        }
        return;
    }

    // overflow check
    if (_frame_ofs > _frame.length + CSRF_HEADER_LEN) {
        _frame_ofs = 0;
        return;
    }

    // decode whatever we got and expect
    if (_frame_ofs == _frame.length + CSRF_HEADER_LEN) {
        log_data(AP_RCProtocol::CRSF, timestamp_us, (const uint8_t*)&_frame, _frame_ofs - CSRF_HEADER_LEN);

        // we consumed the partial frame, reset
        _frame_ofs = 0;

        uint8_t crc = crc8_dvb_s2(0, _frame.type);
        for (uint8_t i = 0; i < _frame.length - 2; i++) {
            crc = crc8_dvb_s2(crc, _frame.payload[i]);
        }

        // bad CRC
        if (crc != _frame.payload[_frame.length - CSRF_HEADER_LEN]) {
            return;
        }

        _last_frame_time_us = timestamp_us;
        // decode here
        if (decode_csrf_packet()) {
            add_input(MAX_CHANNELS, _channels, false, -1);
        }
    }    
}

void AP_RCProtocol_CRSF::update(void)
{
    // never received RC frames, but have received CRSF frames so make sure we give the telemetry opportunity to run
    if (_last_frame_time_us > 0 && !get_rc_frame_count() && _last_frame_time_us > CRSF_INTER_FRAME_TIME_US) {
        process_telemetry(false);
        _last_frame_time_us =  AP_HAL::micros();
    }
}

// write out a frame of any type
void AP_RCProtocol_CRSF::write_frame(Frame* frame)
{
    AP_HAL::UARTDriver *uart = get_available_UART();

    if (!uart) {
        return;
    }

    uart->write(frame->device_address);
    uart->write(frame->length);
    uart->write(frame->type);
    uint8_t crc = crc8_dvb_s2(0, frame->type);
    for (uint8_t i = 0; i < frame->length - 2; i++) {
        crc = crc8_dvb_s2(crc, frame->payload[i]);
    }
    frame->payload[frame->length - 2] = crc;
    uart->write(frame->payload, frame->length - 1);

#ifdef CRSF_DEBUG
    hal.console->printf("CRSF: writing %s: 0x%x 0x%x 0x%x", get_frame_type(frame->type), frame->device_address, frame->length, frame->type);
    for (uint8_t i = 0; i < frame->length - 1; i++) {
        hal.console->printf(" 0x%x", frame->payload[i]);
    }
    hal.console->printf("\n");
#endif
}

bool AP_RCProtocol_CRSF::decode_csrf_packet()
{
#ifdef CRSF_DEBUG
    hal.console->printf("CRSF: received %s:", get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
        hal.console->printf(" 0x%x", fptr[i]);
    }
    hal.console->printf("\n");
#endif

    bool rc_active = false;

#define CRSF_RC_SCALE(value) (((int32_t)value * 1194) / 2048 + 903);

    switch (_frame.type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
            // unpack the RC channels
            const RCChannelsPayload* rcChannels = (RCChannelsPayload*)&_frame.payload;
            _channels[0] = CRSF_RC_SCALE(rcChannels->chan0);
            _channels[1] = CRSF_RC_SCALE(rcChannels->chan1);
            _channels[2] = CRSF_RC_SCALE(rcChannels->chan2);
            _channels[3] = CRSF_RC_SCALE(rcChannels->chan3);
            _channels[4] = CRSF_RC_SCALE(rcChannels->chan4);
            _channels[5] = CRSF_RC_SCALE(rcChannels->chan5);
            _channels[6] = CRSF_RC_SCALE(rcChannels->chan6);
            _channels[7] = CRSF_RC_SCALE(rcChannels->chan7);
            _channels[8] = CRSF_RC_SCALE(rcChannels->chan8);
            _channels[9] = CRSF_RC_SCALE(rcChannels->chan9);
            _channels[10] = CRSF_RC_SCALE(rcChannels->chan10);
            _channels[11] = CRSF_RC_SCALE(rcChannels->chan11);
            _channels[12] = CRSF_RC_SCALE(rcChannels->chan12);
            _channels[13] = CRSF_RC_SCALE(rcChannels->chan13);
            _channels[14] = CRSF_RC_SCALE(rcChannels->chan14);
            _channels[15] = CRSF_RC_SCALE(rcChannels->chan15);
        }
            rc_active = true;
            break;
        default:
            break;
    }

#if HAL_CRSF_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    AP::crsf_telem()->process_frame(FrameType(_frame.type), (uint8_t*)&_frame.payload);
#endif
    process_telemetry();

    return rc_active;
}

// send out telemetry
bool AP_RCProtocol_CRSF::process_telemetry(bool check_constraint)
{

    AP_HAL::UARTDriver *uart = get_available_UART();
    if (!uart) {
        return false;
    }

    if (!telem_available) {
#if HAL_CRSF_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
        if (AP_CRSF_Telem::get_telem_data(&_telemetry_frame)) {
            telem_available = true;
        } else {
            return false;
        }
#else
        return false;
#endif
    }
    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will corrupt the next
      incoming control frame
     */
    uint64_t tend = uart->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > CRSF_MAX_FRAME_TIME_US && check_constraint) {
        // we've been too slow in responding
        return false;
    }

    write_frame(&_telemetry_frame);
    // get fresh telem_data in the next call
    telem_available = false;

    return true;
}

// process a byte provided by a uart
void AP_RCProtocol_CRSF::process_byte(uint8_t byte, uint32_t baudrate)
{
    if (baudrate != CRSF_BAUDRATE) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}

