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
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */
#pragma once

#include "AP_RCProtocol_config.h"
#include <AP_RCTelemetry/AP_RCTelemetry_config.h>

#if AP_RCPROTOCOL_CRSF_ENABLED

#include "AP_CRSF_Protocol.h"
#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "SoftSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_OSD/AP_OSD_config.h>

#define CRSF_MAX_CHANNELS   24U      // Maximum number of channels from crsf datastream
#define CRSF_FRAME_LENGTH_MIN 2 // min value for _frame.length
#define CRSF_BAUDRATE      416666U
#define ELRS_BAUDRATE      420000U
#define CRSF_TX_TIMEOUT    500000U   // the period after which the transmitter is considered disconnected (matches copters failsafe)
#define CRSF_RX_TIMEOUT    150000U   // the period after which the receiver is considered disconnected (>ping frequency)

class AP_CRSF_Protocol;

class AP_RCProtocol_CRSF : public AP_RCProtocol_Backend {
public:
    // a CRSF port can be used for RC IN, VTX control, or RC OUT
    enum class PortMode : uint8_t {
        PASSTHROUGH_RCIN,
        DIRECT_VTX,
        DIRECT_RCOUT,
    };

    // Constructor for RCIN "passthrough" mode (called by AP_RCProtocol)
    AP_RCProtocol_CRSF(AP_RCProtocol &_frontend);
    // Constructor for "direct-attach" modes (called by manager)
    AP_RCProtocol_CRSF(AP_RCProtocol &_frontend, PortMode mode, AP_HAL::UARTDriver* uart);
    virtual ~AP_RCProtocol_CRSF();
    // entry point for bytes from high-level RCIN protocol discriminator
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    void process_handshake(uint32_t baudrate) override;
    // main update call, used for polling UART in direct-attach modes
    void update(void) override;
#if HAL_CRSF_TELEM_ENABLED
    void start_bind(void) override;
    bool bind_in_progress(void);
#endif
    // support for CRSF v3
    bool change_baud_rate(uint32_t baudrate);
    // bootstrap baudrate
    uint32_t get_bootstrap_baud_rate() const {
#if AP_RC_CHANNEL_ENABLED
        return rc().option_is_enabled(RC_Channels::Option::ELRS_420KBAUD) ? ELRS_BAUDRATE : CRSF_BAUDRATE;
#else
        return CRSF_BAUDRATE;
#endif
    }

    // is the receiver active, used to detect power loss and baudrate changes
    bool is_rx_active() const override {
        // later versions of CRSFv3 will send link rate frames every 200ms
        // but only before an initial failsafe
        return _last_rx_frame_time_us != 0 && AP_HAL::micros() - _last_rx_frame_time_us < CRSF_RX_TIMEOUT;
    }

    // is the transmitter active, used to adjust telemetry data
    bool is_tx_active() const {
        // this is the same as the Copter failsafe timeout
        return _last_tx_frame_time_us != 0 && AP_HAL::micros() - _last_tx_frame_time_us < CRSF_TX_TIMEOUT;
    }

#if AP_CRSF_OUT_ENABLED
    // send RC channels out
    bool send_rc_channels(const uint16_t* channels, uint8_t nchannels);
    // send a baudrate proposal
    void send_speed_proposal(uint32_t baudrate);
    // send a ping frame
    void send_ping_frame();
    // send a device info frame
    void send_device_info();
    // send the link stats frame
    void send_link_stats_tx(uint32_t fps);
    void reset_bootstrap_baudrate();
#endif

    // Manager functions
    static void manager_init();
    // get singleton instance for RCIN
    static AP_RCProtocol_CRSF* get_rcin_singleton();
    // get singleton instance for any direct attach port
    static AP_RCProtocol_CRSF* get_direct_attach_singleton(AP_SerialManager::SerialProtocol protocol, uint8_t instance);

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
        CRSF_COMMAND_ACK = 0xFF,
    };

    // Commands for CRSF_COMMAND_FC
    enum CommandFC {
        CRSF_COMMAND_FC_DISARM = 0x01,
        CRSF_COMMAND_SCALE_CHANNEL = 0x02,
    };

    // Commands for CRSF_COMMAND_BLUETOOTH
    enum CommandBluetooth {
        CRSF_COMMAND_BLUETOOTH_RESET = 0x01,
        CRSF_COMMAND_BLUETOOTH_ENABLE = 0x02,
        CRSF_COMMAND_BLUETOOTH_ECHO = 0x64,
    };

    // Commands for CRSF_COMMAND_OSD
    enum CommandOSD {
        CRSF_COMMAND_OSD_SEND = 0x01,
    };

    // Commands for CRSF_COMMAND_VTX
    enum CommandVTX {
        CRSF_COMMAND_VTX_CHANNEL = 0x01,
        CRSF_COMMAND_VTX_FREQ = 0x02,
        CRSF_COMMAND_VTX_POWER = 0x03,
        CRSF_COMMAND_VTX_PITMODE = 0x04,
        CRSF_COMMAND_VTX_PITMODE_POWERUP = 0x05,
        CRSF_COMMAND_VTX_POWER_DBM = 0x08,
    };

    // Commands for CRSF_COMMAND_LED
    enum CommandLED {
        CRSF_COMMAND_LED_SET_DEFAULT = 0x01,
        CRSF_COMMAND_LED_COLOR = 0x02,
        CRSF_COMMAND_LED_PULSE = 0x03,
        CRSF_COMMAND_LED_BLINK = 0x04,
        CRSF_COMMAND_LED_SHIFT = 0x05,
    };

    // Commands for CRSF_COMMAND_RX
    enum CommandRX {
        CRSF_COMMAND_RX_BIND = 0x01,
        CRSF_COMMAND_RX_CANCEL_BIND = 0x02,
        CRSF_COMMAND_RX_SET_BIND_ID = 0x03,
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

    // SubType IDs for CRSF_FRAMETYPE_CUSTOM_TELEM
    enum CustomTelemSubTypeID : uint8_t {
        CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH = 0xF0,
        CRSF_AP_CUSTOM_TELEM_STATUS_TEXT = 0xF1,
        CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH = 0xF2,
    };

    using DeviceAddress = AP_CRSF_Protocol::DeviceAddress;

    enum ExtendedFrameOffset {
        CRSF_EXTENDED_FRAME_LENGTH_OFFSET = 1,
        CRSF_EXTENDED_FRAME_TYPE_OFFSET = 2,
        CRSF_EXTENDED_FRAME_DESTINATION_OFFSET = 3,
        CRSF_EXTENDED_FRAME_ORIGIN_OFFSET = 4,
        CRSF_EXTENDED_FRAME_PAYLOAD_OFFSET = 5,
    };

    struct LinkStatisticsFrame {
        uint8_t uplink_rssi_ant1; // ( dBm * -1 )
        uint8_t uplink_rssi_ant2; // ( dBm * -1 )
        uint8_t uplink_status; // Package success rate / Link quality ( % )
        int8_t uplink_snr; // ( db )
        uint8_t active_antenna; // Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
        uint8_t rf_mode; // ( enum 4fps = 0 , 50fps, 150hz)
        uint8_t uplink_tx_power; // ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
        uint8_t downlink_rssi; // ( dBm * -1 )
        uint8_t downlink_status; // Downlink package success rate / Link quality ( % )
        int8_t downlink_dnr; // ( db )
    } PACKED;

    struct LinkStatisticsRXFrame {
        uint8_t rssi_db;        // RSSI(dBm*-1)
        uint8_t rssi_percent;   // RSSI in percent
        uint8_t link_quality;   // Package success rate / Link quality ( % )
        int8_t snr;             // SNR(dB)
        uint8_t rf_power_db;    // rf power in dBm
    } PACKED;

    struct SubsetChannelsFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint8_t starting_channel:5;     // which channel number is the first one in the frame
        uint8_t res_configuration:2;    // configuration for the RC data resolution (10 - 13 bits)
        uint8_t digital_switch_flag:1;  // configuration bit for digital channel
        uint8_t channels[CRSF_FRAME_PAYLOAD_MAX - 2]; // payload less byte above
        // uint16_t channel[]:res;      // variable amount of channels (with variable resolution based
                                        // on the res_configuration) based on the frame size
        // uint16_t digital_switch_channel[]:10; // digital switch channel
    } PACKED;

    // Source for ELRS RF modes: https://www.expresslrs.org/info/signal-health/#rf-mode-indexes-rfmd
    enum RFMode {
        CRSF_RF_MODE_4HZ = 0,
        CRSF_RF_MODE_50HZ,
        CRSF_RF_MODE_150HZ,
        CRSF_RF_MODE_250HZ,
        CRSF_RF_MAX_MODES = 4,
        ELRS_RF_MODE_4HZ = 4,
        ELRS_RF_MODE_25HZ,
        ELRS_RF_MODE_50HZ,
        ELRS_RF_MODE_100HZ,
        ELRS_RF_MODE_100HZ_FULL,
        ELRS_RF_MODE_150HZ,
        ELRS_RF_MODE_200HZ,
        ELRS_RF_MODE_250HZ,
        ELRS_RF_MODE_333HZ_FULL,        
        ELRS_RF_MODE_500HZ,
        ELRS_RF_MODE_D250HZ,
        ELRS_RF_MODE_D500HZ,
        ELRS_RF_MODE_F500HZ,
        ELRS_RF_MODE_F1000HZ,
        ELRS_RF_MODE_D50HZ,
        RF_MODE_MAX_MODES,
        RF_MODE_UNKNOWN,
    };

#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
    // These power levels are valid for both Crossfire and ELRS systems
    static constexpr uint16_t tx_powers[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };    
#endif

    struct LinkStatus {
        int16_t rssi = -1;
        int16_t link_quality = -1;
        uint8_t rf_mode;
        uint8_t fps;    // fps / 10
#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
        // Add the extra data fields to be used by the OSD panels
        int16_t tx_power = -1;
        int8_t rssi_dbm = -1;
        int8_t snr = INT8_MIN;
        int8_t active_antenna = -1;
#endif
    };


    // this will be used by AP_CRSF_Telem to access link status data
    // from within AP_RCProtocol_CRSF thread so no need for cross-thread synch
    const volatile LinkStatus& get_link_status() const {
        return _link_status;
    }

    // return the link rate as defined by the LinkStatistics
    uint16_t get_link_rate(AP_CRSF_Protocol::ProtocolType protocol) const;

    // return the protocol string
    const char* get_protocol_string(AP_CRSF_Protocol::ProtocolType protocol) const;

    // write a CRSF Frame structure to the managed uart
    void write_frame(AP_CRSF_Protocol::Frame* frame) const;

private:
    // private class to hold static state for the manager
    class Manager_State {
        friend class AP_RCProtocol_CRSF;
        static AP_RCProtocol_CRSF* _instances[HAL_NUM_SERIAL_PORTS];
        static AP_RCProtocol_CRSF* _rcin_singleton;
        static uint32_t _last_manager_check_ms;
    };

    struct AP_CRSF_Protocol::Frame _frame;
    uint8_t *_frame_bytes = (uint8_t*)&_frame;
    struct AP_CRSF_Protocol::Frame _telemetry_frame;
    uint8_t _frame_ofs;

    const uint8_t MAX_CHANNELS = MIN((uint8_t)CRSF_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    AP_CRSF_Protocol* _protocol_helper;

    void _process_byte(uint8_t byte);
    bool check_frame(uint32_t timestamp_us);
    void skip_to_next_frame(uint32_t timestamp_us);
    bool decode_crsf_packet();
    bool process_telemetry(bool check_constraint = true) const;
    void process_link_stats_frame(const void* data);
    void process_link_stats_rx_frame(const void* data);
    void process_link_stats_tx_frame(const void* data);

    void start_uart();
    AP_HAL::UARTDriver* get_current_UART() const {
        if (_uart) return _uart;
        // Fallback for RCIN mode
        return get_available_UART();
    }

    uint16_t _channels[CRSF_MAX_CHANNELS];    /* buffer for extracted RC channel data as pulsewidth in microseconds */

    uint32_t _last_frame_time_us;
    uint32_t _last_tx_frame_time_us;
    uint32_t _last_uart_start_time_ms;
    uint32_t _last_rx_frame_time_us;
    uint32_t _start_frame_time_us;
    mutable bool telem_available;
    uint32_t _new_baud_rate;
    bool _crsf_v3_active;
    PortMode _mode;

    bool _use_lq_for_rssi;
    int16_t derive_scaled_lq_value(uint8_t uplink_lq);

    volatile struct LinkStatus _link_status;

    static const uint16_t RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES];

    AP_HAL::UARTDriver *_uart;
};

namespace AP {
    // Note: this function is now a legacy accessor for the RCIN singleton instance
    AP_RCProtocol_CRSF* crsf();
};

#endif  // AP_RCPROTOCOL_CRSF_ENABLED


