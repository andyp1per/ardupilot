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
 *
 * Port of betaflight CRSF support by Andy Piper
 */


#include "AP_CRSF.h"

#if 0

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo AP_CRSF::var_info[] = {
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// singleton instance
AP_CRSF *AP_CRSF::_singleton;

AP_CRSF::AP_CRSF()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CRSF must be singleton");
    }
    _singleton = this;
}

// init the runcam device by finding a serial device configured for the RunCam protocol
void AP_CRSF::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager) {
        uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_CSRF, 0);
    }

    if (uart == nullptr) {
        return;
    }
    uart->begin(CRSF_BAUDRATE);
}

void AP_CRSF::crsfProtocolPackOutOfRangeCli(StreamBuffer*dst)
{
    dst->write_uint8(CRSF_OUT_OF_RANGE);
    dst->write_uint8(0);
}

void AP_CRSF::crsfProtocolPackFolderCli(StreamBuffer*dst, const clivalue_s *value, crsfProtocolParamPackString_e paramPackString)
{
    dst->write_uint8(CRSF_FOLDER);
    if (paramPackString == CRSF_PARAM_INCLUDE_STRING) {
        dst->write_string_with_zero_terminator(value->name);
    } else {
        dst->write_uint8(0);
    }
}

void AP_CRSF::crsfProtocolPackU8Cli(StreamBuffer*dst, const clivalue_s *value, crsfProtocolParamPackString_e paramPackString)
{
    dst->write_uint8(CRSF_UINT8);
    if (paramPackString == CRSF_PARAM_INCLUDE_STRING) {
        dst->write_string_with_zero_terminator(value->name);
    } else {
        dst->write_uint8( 0);
    }
    const uint8_t *valuePtr = cliGetValuePointer(value);
    if (valuePtr) {
        dst->write_uint8( *valuePtr);
    }
    dst->write_uint8( value->config.minmax.min);
    dst->write_uint8( value->config.minmax.max);
    const uint8_t *copy = cliGetDefaultPointer(value);
    dst->write_uint8( *copy);
    if (paramPackString == CRSF_PARAM_INCLUDE_STRING) {
        dst->write_uint8( 0); // units not currently supported, just write null string
    } else {
        dst->write_uint8( 0);
    }
}

/*
 * CRSF_FRAMETYPE_PARAMETER_WRITE: // 0x2D
 * Payload:
 * uint8_t Parameter index
 * Data ( size depending on data type )
 */
void AP_CRSF::crsfProtocolUnpackU8Cli(const clivalue_s *value, const uint8_t *payload)
{
    uint8_t val = payload[CRSF_PARAMETER_WRITE_DATA_OFFSET];
    if (val < value->config.minmax.min) {
        val = value->config.minmax.min;
    }
    if (val > value->config.minmax.max) {
        val = value->config.minmax.max;
    }
    uint8_t *valuePtr = cliGetValuePointer(value);
    if (valuePtr) {
        *valuePtr = val;
    }
}

void AP_CRSF::crsfProtocolPackU16Cli(StreamBuffer*dst, const clivalue_s *value, crsfProtocolParamPackString_e paramPackString)
{
    dst->write_uint8( CRSF_UINT16);
    if (paramPackString == CRSF_PARAM_INCLUDE_STRING) {
        dst->write_string_with_zero_terminator(value->name);
    } else {
        dst->write_uint8( 0);
    }
    const uint16_t *valuePtr = cliGetValuePointer(value);
    if (valuePtr) {
        dst->write_uint16_big_endian(*valuePtr);
    }
    dst->write_uint16_big_endian(value->config.minmax.min);
    dst->write_uint16_big_endian(value->config.minmax.max);
    const uint16_t *copy = cliGetDefaultPointer(value);
    dst->write_uint16_big_endian(*copy);
    if (paramPackString == CRSF_PARAM_INCLUDE_STRING) {
        dst->write_uint8( 0); // units not currently supported, just write null string
    } else {
        dst->write_uint8( 0);
    }
}

void AP_CRSF::crsfProtocolUnpackU16Cli(const clivalue_s *value, const uint8_t *payload)
{
    uint16_t val = payload[CRSF_PARAMETER_WRITE_DATA_OFFSET] << 8 | payload[CRSF_PARAMETER_WRITE_DATA_OFFSET+1];
    if (val < value->config.minmax.min) {
        val = value->config.minmax.min;
    }
    if (val > value->config.minmax.max) {
        val = value->config.minmax.max;
    }
    uint16_t *valuePtr = cliGetValuePointer(value);
    if (valuePtr) {
        *valuePtr = val;
    }
}

void AP_CRSF::crsfParameterWriteEntryHeader(StreamBuffer*dst, uint8_t destinationAddress)
{

    dst->write_uint8(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY);
    dst->write_uint8(destinationAddress);
    dst->write_uint8(CRSF_ADDRESS_FLIGHT_CONTROLLER); // our address
    dst->write_uint8(currentParamIndex);
    dst->write_uint8(currentChunkIndex);
    int i = currentParamIndex;
    if (valueTable[i].type == FOLDER_VALUE) {
        while (i > 0) {
            i--;
            /*!!TODOif (parameterList[i] == reinterpret_cast<const parameterList_s *>(folder->parent)) {
                break;
            }*/
        }
    } else {
        // step back to find parent folder
        while (i > 0) {
            i--;
            if (valueTable[i].type == FOLDER_VALUE) {
                break;
            }
        }
    }
    dst->write_uint8( i);
}


enum { CHUNKS_REMAINING_FRAME_OFFSET = 5, CHUNK_HEADER_SIZE = 6 };
enum { CHUNK_SIZE_MAX = CRSF_FRAME_SIZE_MAX - CHUNK_HEADER_SIZE };

/*
 *CRSF_FRAMETYPE_PARAMETER_READ: 0x2C
 * Host wants to read parameter from FC
 * Payload:
 *     uint8_t Parameter number (starting from 1)
 *     uint8_t Parameter chunks remaining ( Chunks )
 */
void AP_CRSF::crsfProtocolParameterRead(StreamBuffer*dst, const uint8_t *payload, uint8_t originAddress, crsfProtocolParamPackString_e paramPackString)
{
    currentParamIndex = payload[0];
    currentChunkIndex = payload[1];
    StreamBuffer sbuf(readBuf, ARRAYEND(readBuf));

    // write zero for frame length, since we don't know it yet
    uint8_t *lengthPtr = sbuf.data();
    sbuf.write_uint8(0);
    crsfParameterWriteEntryHeader(&sbuf, originAddress);

    bool dataWritten = true;
    const clivalue_t *value = &valueTable[currentParamIndex];
    if ((value->type & VALUE_SECTION_MASK) == FOLDER_VALUE) {
        crsfProtocolPackFolderCli(sbuf, value, paramPackString);
    } else if ((value->type & VALUE_MODE_MASK) == MODE_EOF) {
        crsfProtocolPackOutOfRangeCli(sbuf);
    } else {
        switch (value->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
        case VAR_INT8:
            crsfProtocolPackU8Cli(sbuf, value, paramPackString);
            break;
        case VAR_UINT16:
        case VAR_INT16:
            crsfProtocolPackU16Cli(sbuf, value, paramPackString);
            break;
        default:
            dataWritten = false;
            break;
        }
    }
    if (dataWritten) {
        int frameSize = sbuf.data() - &readBuf[0];
        if (frameSize >= CRSF_FRAME_SIZE_MAX) {
            // data does not fit into one frame, we must split it into chunks
            uint16_t chunkCount = (frameSize - CRSF_FRAME_SIZE_MAX) / (CRSF_FRAME_SIZE_MAX - CHUNK_HEADER_SIZE);
            if ((frameSize - CRSF_FRAME_SIZE_MAX) % (CRSF_FRAME_SIZE_MAX - CHUNK_HEADER_SIZE) != 0 ) {
                // round up if deviation above is not equal 0
                ++chunkCount;
            }
            if (currentChunkIndex > chunkCount) {
                currentChunkIndex = 0;
            }
            if (currentChunkIndex == 0) {
                frameSize = CRSF_FRAME_SIZE_MAX;
            } else {
                const uint8_t chunkStart = (currentChunkIndex - 1) * (CRSF_FRAME_SIZE_MAX  - CHUNK_HEADER_SIZE) + CRSF_FRAME_SIZE_MAX - 1;
                const uint8_t chunkSize = frameSize - chunkStart > CHUNK_SIZE_MAX ? CHUNK_SIZE_MAX : frameSize - chunkStart + 1;
                // copy the current chunk to the start of the readBuf, memmove handles overlap
                memmove(&readBuf[6], &readBuf[chunkStart], chunkSize);
                frameSize = CHUNK_HEADER_SIZE + chunkSize;
            }
            readBuf[5] = chunkCount - currentChunkIndex;
            readBuf[0] = (uint8_t)(frameSize - 2);
            readBuf[frameSize - 1] = crc8_dvb_s2_update(0, &readBuf[1], frameSize - 2);
            ++currentChunkIndex;
        } else {
            currentChunkIndex = 0;
            *lengthPtr = frameSize; // length and length byte
            crc8_dvb_s2_sbuf_append(sbuf, lengthPtr + CRSF_FRAME_LENGTH_TYPE);
            ++frameSize;
        }
        // copy readBuf to dst
        dst.write_data(readBuf, frameSize);
    }
}

/*
 * CRSF_FRAMETYPE_PARAMETER_WRITE: // 0x2D
 * Payload:
 * uint8_t Parameter index
 * Data ( size depending on data type )
 */
void AP_CRSF::crsfProtocolParameterWrite(const uint8_t *payload)
{
    const uint8_t paramIndex = payload[0];

    if (paramIndex < valueTableEntryCount) {
        const clivalue_t *value = &valueTable[paramIndex];
        switch (value->type) {
        case VAR_UINT8:
        case VAR_INT8:
            crsfProtocolUnpackU8Cli(value, payload);
            break;
        case VAR_UINT16:
        case VAR_INT16:
            crsfProtocolUnpackU16Cli(value, payload);
            break;
        default:
            break;
        }
        currentParamIndex = paramIndex;
        currentChunkIndex = 0;
    } else {
        currentParamIndex = 0;
    }
}

/*
 * Interpret a received Crossfire Protocol Extended Frame.
 * First byte of the frame is the SYNC_BYTE.
 * Returns true if reply frame should be transmitted.
 *
 * Extended Frame:
 *     uint8_t sync byte or device address
 * Frame:
 *     uint8_t length
 *     uint8_t type
 *     uint8_t Destination node address
 *     uint8_t Origin node address
 * Payload:
 *     uint8_t Parameter number (starting from 1)
 *     uint8_t Parameter chunks remaining ( Chunks )
 */
bool AP_CRSF::crsfInterpretExtendedFrame(StreamBuffer*dst, uint8_t *extendedFrame)
{
    bool ret = false;
    const crsfAddress_e destinationAddress = (crsfAddress_e)extendedFrame[CRSF_EXTENDED_FRAME_DESTINATION_OFFSET];
    const crsfFrameType_e frameType = (crsfFrameType_e)extendedFrame[CRSF_EXTENDED_FRAME_TYPE_OFFSET];
    const uint8_t* payload = extendedFrame + CRSF_EXTENDED_FRAME_PAYLOAD_OFFSET;

    switch (frameType) {
    case CRSF_FRAMETYPE_DEVICE_PING:
        if (destinationAddress == CRSF_ADDRESS_BROADCAST || destinationAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            //crsfProtocolParameterPackDeviceInfo(dst, payload);
            ret = true;
        }
        break;
    case CRSF_FRAMETYPE_PARAMETER_READ: // 0x2C
        // sent when host wants to read a parameter
        // reply with FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B frame with parameter data
        if (destinationAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            const crsfAddress_e originAddress = (crsfAddress_e)extendedFrame[CRSF_EXTENDED_FRAME_ORIGIN_OFFSET];
            crsfProtocolParameterRead(dst, payload, originAddress, CRSF_PARAM_INCLUDE_STRING);
            ret = true;
        }
        break;
    case CRSF_FRAMETYPE_PARAMETER_WRITE: // 0x2D
        // sent when host wants to write a parameter
        // update parameter and reply with FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B frame to confirm parameter data
       if (destinationAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            crsfProtocolParameterWrite(payload);
            const crsfAddress_e originAddress = (crsfAddress_e)extendedFrame[CRSF_EXTENDED_FRAME_ORIGIN_OFFSET];
            crsfProtocolParameterRead(dst, payload, originAddress, CRSF_PARAM_SKIP_STRING);
            ret = true;
        }
        break;
    case CRSF_FRAMETYPE_COMMAND:
    default:
        break;
    }
    return ret;
}

#endif