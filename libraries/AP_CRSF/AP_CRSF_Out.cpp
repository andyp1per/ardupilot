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
 * Code by Andy Piper <github@andypiper.com>
 */

/*
 * AP_CRSF_Out.cpp - High-level driver for CRSF RC Output
 *
 * This class provides the high-level "application" logic for the
 * CRSF RC Output feature. It is responsible for reading servo output values
 * from the main SRV_Channels and telling its underlying CRSF protocol instance
 * to send them at a user-configurable rate.
 */
#include "AP_CRSF_config.h"

#if AP_CRSF_OUT_ENABLED

#include "AP_CRSF_Out.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

// Include the external AHRS CRSF header if it is enabled to allow data passing
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS_CRSF.h>
#endif
#include <AP_Scheduler/AP_Scheduler.h>

//#define CRSF_RCOUT_DEBUG
//#define CRSF_RCOUT_DEBUG_FRAME
#ifdef CRSF_RCOUT_DEBUG
# include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
static uint32_t last_update_debug_ms = 0;
static uint32_t num_frames = 0;
# define debug_rcout(fmt, args...) do { if (hal.console) { hal.console->printf("CRSF_OUT: " fmt "\n", ##args); } } while(0)
#else
# define debug_rcout(fmt, args...)
#endif

#define DEFAULT_CRSF_OUTPUT_RATE      250U // equivalent to tracer

AP_CRSF_Out* AP_CRSF_Out::singleton;

extern const AP_HAL::HAL& hal;

AP_CRSF_Out::AP_CRSF_Out(AP_HAL::UARTDriver& _uart, uint8_t instance, AP_CRSF_OutManager& _frontend) :
    instance_idx(instance), uart(_uart), frontend(_frontend)
{
    // in the future we could consider supporting multiple output handlers
    if (singleton != nullptr) {
        AP_HAL::panic("Duplicate CRSF_Out handler");
    }

    singleton = this;

    init(uart);
}

// get the configured output rate
uint16_t AP_CRSF_Out::get_configured_update_rate() const
{
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
    return AP::externalAHRS().get_IMU_rate();
#else
    return frontend.rate_hz.get();
#endif
}

// Initialise the CRSF output driver
bool AP_CRSF_Out::init(AP_HAL::UARTDriver& _uart)
{
    if (state != State::WAITING_FOR_PORT) {
        return false;
    }

    crsf_port = NEW_NOTHROW AP_RCProtocol_CRSF(AP::RC(), AP_RCProtocol_CRSF::PortMode::DIRECT_RCOUT, &_uart);
    _instance_idx = 0;

    if (crsf_port == nullptr) {
        debug_rcout("Init failed: could not create CRSF output port");
        return false;
    }

    const uint16_t rate = get_configured_update_rate();

    if (rate > 0) {
<<<<<<< HEAD
        frame_interval_us = 1000000UL / rate;
    } else {
        frame_interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
    }

    scheduler.init(tasks, rate);
    state = State::WAITING_FOR_RC_LOCK;
    scheduler.set_task_rate(REPORTING, frontend.reporting_rate_hz);
=======
        _loop_rate_hz = rate;
        _frame_interval_us = 1000000UL / rate;
    } else {
        _loop_rate_hz = DEFAULT_CRSF_OUTPUT_RATE;
        _frame_interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
    }

    _scheduler.init(_tasks, uint16_t(_loop_rate_hz));
    _state = State::WAITING_FOR_RC_LOCK;
    _scheduler.set_task_rate(REPORTING, _frontend._reporting_rate_hz);
>>>>>>> 276eb0a1dd (AP_CRSF: add IMU rate calibration for external AHRS)


    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CRSF_Out::crsf_out_thread, void), "crsf", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        delete crsf_port;
        crsf_port = nullptr;
        debug_rcout("Failed to create CRSF_Out thread");
        return false;
    }

    return true;
}

void AP_CRSF_Out::update_rates_status()
{
    const float report_rate = frontend.reporting_rate_hz.get();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: IMU: %uHz, GPS: %uHz, Baro: %uHz, Mag: %uHz, RC: %uHz, Lat: %.1fms",
                    int16_t(rate_imu_counter*report_rate),
                    int16_t(rate_gps_counter*report_rate),
                    int16_t(rate_baro_counter*report_rate),
                    int16_t(rate_mag_counter*report_rate),
                    int16_t(rate_rc_counter*report_rate), latency_us / 1000.0f);
    rate_imu_counter = rate_gps_counter = rate_baro_counter = rate_mag_counter = rate_rc_counter = 0;
}

/*
  Calibrate loop rate to achieve target IMU rate.

  The CRSF link carries multiple sensor types (IMU, GPS, Baro, Mag) which share
  bandwidth. To achieve a target IMU rate, we need to run the loop faster to
  compensate for the bandwidth used by other sensors.

  This uses adaptive filtering similar to AP_InertialSensor_Backend::_update_sensor_rate():
  - First 30s: fast convergence (filter=0.8, limits ±50%)
  - After 30s: slow convergence (filter=0.98, limits ±5%)
  - No adjustment when armed
*/
void AP_CRSF_Out::update_imu_rate_calibration()
{
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
    const uint32_t now_us = AP_HAL::micros();

    // Initialize measurement window on first call
    if (_imu_cal_start_us == 0) {
        _imu_cal_start_us = now_us;
        _imu_cal_count = 0;
        _calibration_start_ms = AP_HAL::millis();
        return;
    }

    // Check if 1 second has elapsed
    if (now_us - _imu_cal_start_us < 1000000UL) {
        return;
    }

    // Don't adjust when armed - maintain stable rate
    if (hal.util->get_soft_armed()) {
        _imu_cal_start_us = now_us;
        _imu_cal_count = 0;
        return;
    }

    const float target_rate = get_configured_update_rate();
    if (target_rate <= 0) {
        _imu_cal_start_us = now_us;
        _imu_cal_count = 0;
        return;
    }

    // Calculate observed IMU rate over the measurement window
    const float elapsed_s = (now_us - _imu_cal_start_us) * 1.0e-6f;
    const float observed_imu_rate = _imu_cal_count / elapsed_s;

    // Determine convergence parameters based on time since start
    const uint32_t elapsed_ms = AP_HAL::millis() - _calibration_start_ms;
    const bool converging = elapsed_ms < 30000;

    float filter_constant;
    float upper_limit;
    float lower_limit;

    if (converging) {
        // Fast convergence for first 30s
        filter_constant = 0.8f;
        upper_limit = 1.5f;
        lower_limit = 0.5f;
    } else {
        // Slow convergence after 30s
        filter_constant = 0.98f;
        upper_limit = 1.05f;
        lower_limit = 0.95f;

        // Print convergence message once
        if (!_calibration_converged) {
            _calibration_converged = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: IMU rate converged, loop=%.0fHz", _loop_rate_hz);
        }
    }

    // Calculate the required loop rate to achieve target IMU rate
    // If observed < target, we need to increase loop rate
    // new_loop_rate = current_loop_rate * (target / observed)
    float rate_ratio = 1.0f;
    if (observed_imu_rate > 0) {
        rate_ratio = target_rate / observed_imu_rate;
    }

    // Constrain the ratio to prevent wild swings
    rate_ratio = constrain_float(rate_ratio, lower_limit, upper_limit);

    // Calculate new loop rate
    float new_loop_rate = _loop_rate_hz * rate_ratio;

    // Apply low-pass filter to smooth the adjustment
    _loop_rate_hz = filter_constant * _loop_rate_hz + (1 - filter_constant) * new_loop_rate;

    // Clamp to reasonable bounds (±20% of target rate)
    _loop_rate_hz = constrain_float(_loop_rate_hz, target_rate * 0.8f, target_rate * 1.2f);

    // Update the frame interval and scheduler
    _frame_interval_us = 1000000UL / uint32_t(_loop_rate_hz);
    _scheduler.set_loop_rate(uint16_t(_loop_rate_hz));

    debug_rcout("IMU rate cal: observed=%.0f target=%.0f loop=%.0f",
                observed_imu_rate, target_rate, _loop_rate_hz);

    // Reset for next measurement window
    _imu_cal_start_us = now_us;
    _imu_cal_count = 0;
#endif
}

/*
  Reset IMU rate calibration state.
  Call this when link conditions change (e.g., baud rate negotiation completes)
  to restart the fast convergence period.
*/
void AP_CRSF_Out::reset_imu_rate_calibration()
{
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
    const uint16_t target_rate = get_configured_update_rate();

    // Reset to target rate and restart convergence
    _loop_rate_hz = target_rate > 0 ? target_rate : DEFAULT_CRSF_OUTPUT_RATE;
    _frame_interval_us = 1000000UL / uint32_t(_loop_rate_hz);
    _scheduler.set_loop_rate(uint16_t(_loop_rate_hz));

    _imu_cal_start_us = 0;
    _imu_cal_count = 0;
    _calibration_start_ms = AP_HAL::millis();
    _calibration_converged = false;

    debug_rcout("IMU rate calibration reset, loop_rate=%.0f", _loop_rate_hz);
#endif
}


void AP_CRSF_Out::crsf_out_thread()
{
    // make sure the current thread is the uart owner
    crsf_port->start_uart();

    while (true) {

#ifdef CRSF_RCOUT_DEBUG
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_update_debug_ms > 1000) {
            debug_rcout("Frame rate: %uHz, wanted: %uHz.", unsigned(num_frames), unsigned(get_configured_update_rate()));
            last_update_debug_ms = now_ms;
            num_frames = 0;
        }
#endif

        const uint32_t now_us = AP_HAL::micros();
        uint32_t interval_us = frame_interval_us;

        // if we have not negotiated a faster baudrate do not go above the default output rate
        if (get_configured_update_rate() > DEFAULT_CRSF_OUTPUT_RATE && uart.get_baud_rate() == CRSF_BAUDRATE) {
            interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
        }

        const uint32_t timeout_remaining_us = AP_HAL::timeout_remaining(last_frame_us, now_us, interval_us);
        if (timeout_remaining_us > 0) {
            hal.scheduler->delay_microseconds(timeout_remaining_us);
        }

        // Check for overrun - if we are late by more than 50% of an interval,
        // give up on the old timeline and reset.
        if (AP_HAL::timeout_remaining(last_frame_us, now_us, interval_us + interval_us/2) == 0) {
            last_frame_us = now_us;
        } else {
            // Use scheduled frame time to maintain precise rate
            last_frame_us = last_frame_us + interval_us;  // this may wrap, but that is still correct
        }

#ifdef CRSF_RCOUT_DEBUG
        num_frames++;
#endif
        if (state == State::RUNNING && crsf_port->is_rx_active()) {
            const bool send_rc_frame = pwm_is_fresh;
            pwm_is_fresh = false;

            if (send_rc_frame) {
                scheduler.run_task_immediately(AETR_RC_FRAME);
            }

            if (!scheduler.update() && !send_rc_frame) {
                send_heartbeat();
            }

            // Calibrate loop rate to achieve target IMU rate
            update_imu_rate_calibration();
        } else {
            run_state_machine();
        }

        // process bytes from the UART
        crsf_port->update_uart();
    }
}

// callback from the RC thread
void AP_CRSF_Out::update()
{

}

// PWM push called from SRV_Channels::push
void AP_CRSF_Out::push()
{
    pwm_is_fresh = true;
}

// run the state machine to get us to the running state
void AP_CRSF_Out::run_state_machine()
{
    const uint32_t now = AP_HAL::micros();
    const uint32_t now_ms = AP_HAL::millis();

    const uint32_t BAUD_NEG_TIMEOUT_US = 300000; // 300ms timeout for a response
    const uint32_t BAUD_NEG_INTERVAL_US = 100000; // send proposal every 100ms
    const uint32_t LIVENESS_CHECK_TIMEOUT_US = 200000; // 200ms timeout for health check (TBS downgrade threshold)
    const uint8_t BAUD_NEG_MAX_RETRIES = 1; // retry same baud rate once before falling back

    switch (state) {
    case State::WAITING_FOR_PORT:
        // should have been handled above
        break;

    case State::WAITING_FOR_RC_LOCK:
        // ArduPilot requires 3 good RC frames before it considers the protocol
        // detected, so keep sending RC frames until the rx registers as active
        // because we received something back
        send_aetr_rc_frame();

        if (AP_HAL::timeout_expired(last_status_update_ms, now_ms, 1000UL)) {
            last_status_update_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: waiting for RC lock");
        }

        if (crsf_port->is_rx_active()) {
            state = State::WAITING_FOR_DEVICE_INFO;
            debug_rcout("Telemetry active, requesting device information");
        }
        break;

    case State::WAITING_FOR_DEVICE_INFO:
        send_ping_frame();

        if (version.major > 0 && crsf_port->is_rx_active()) {
            // Start at highest rate not rejected this session
            target_baudrate = _max_allowed_baudrate;
            if (target_baudrate >= 2000000) {
                state = State::NEGOTIATING_2M;
            } else if (target_baudrate >= 1000000) {
                state = State::NEGOTIATING_1M;
            } else {
                // Both rates were rejected, skip negotiation
                state = State::RUNNING;
                debug_rcout("Initialised, using 416kBd (higher rates rejected)");
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 416kBd may limit EAHRS IMU rate");
                reset_imu_rate_calibration();
#endif
                break;
            }
            crsf_port->reset_bootstrap_baudrate();
            baud_negotiation_result = BaudNegotiationResult::PENDING;
            baud_neg_retries = 0;
            debug_rcout("Initialised, negotiating %ukBd", unsigned(target_baudrate/1000));
        }
        break;

    case State::NEGOTIATING_2M:
    case State::NEGOTIATING_1M: {
        // Continue sending ping frames to keep the link alive
        send_ping_frame();

        // Check for response
        if (baud_negotiation_result == BaudNegotiationResult::SUCCESS) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: negotiated %uMBd", unsigned(target_baudrate/1000000));
            state = State::RUNNING;
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
            if (target_baudrate < 2000000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 1MBd may limit EAHRS IMU rate");
            }
            reset_imu_rate_calibration();
#endif
            break;
        }

        if (baud_neg_start_us == 0) {
            baud_neg_start_us = now;
        }

        // Check for explicit failure (rejection)
        if (baud_negotiation_result == BaudNegotiationResult::FAILED) {
            if (state == State::NEGOTIATING_2M) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: 2MBd rejected, trying 1MBd");
                max_allowed_baudrate = 1000000;  // Remember rejection for this session
                state = State::NEGOTIATING_1M;
                target_baudrate = 1000000;
                crsf_port->reset_bootstrap_baudrate();
                baud_negotiation_result = BaudNegotiationResult::PENDING;
                last_baud_neg_us = 0; // force immediate send
                baud_neg_start_us = 0;
                baud_neg_retries = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 1MBd rejected, using 416kBd");
                max_allowed_baudrate = 416000;  // Remember rejection for this session
                scheduler.set_loop_rate(DEFAULT_CRSF_OUTPUT_RATE);
                state = State::RUNNING;
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 416kBd may limit EAHRS IMU rate");
                reset_imu_rate_calibration();
#endif
            }
            break;
        }

        // Check for timeout
        if (now - baud_neg_start_us > BAUD_NEG_TIMEOUT_US) {
            baud_neg_retries++;
            if (baud_neg_retries <= BAUD_NEG_MAX_RETRIES) {
                // Retry same baud rate
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: %uMBd timeout, retry %u/%u",
                              unsigned(target_baudrate/1000000), baud_neg_retries, BAUD_NEG_MAX_RETRIES);
                crsf_port->reset_bootstrap_baudrate();
                baud_negotiation_result = BaudNegotiationResult::PENDING;
                last_baud_neg_us = 0; // force immediate send
                baud_neg_start_us = 0;
            } else if (state == State::NEGOTIATING_2M) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: 2MBd failed, trying 1MBd");
                state = State::NEGOTIATING_1M;
                target_baudrate = 1000000;
                crsf_port->reset_bootstrap_baudrate();
                baud_negotiation_result = BaudNegotiationResult::PENDING;
                last_baud_neg_us = 0; // force immediate send
                baud_neg_start_us = 0;
                baud_neg_retries = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 1MBd failed, using 416kBd");
                state = State::RUNNING;
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: 416kBd may limit EAHRS IMU rate");
                reset_imu_rate_calibration();
#endif
            }
            break;
        }

        // If pending, send proposal periodically
        if (now - last_baud_neg_us > BAUD_NEG_INTERVAL_US) {
            last_baud_neg_us = now;
            send_speed_proposal(target_baudrate);
            debug_rcout("Sent speed proposal for %u", (unsigned)target_baudrate);
        }
        break;
    }

    case State::RUNNING:
        if (!crsf_port->is_rx_active()) {
            debug_rcout("Connection lost, checking liveness");
            last_liveness_check_us = now;
            state = State::HEALTH_CHECK_PING;
            send_ping_frame(true);
        }
        break;

    case State::HEALTH_CHECK_PING:
        if (crsf_port->is_rx_active()) {
            state = State::RUNNING;
        } else if (now - last_liveness_check_us > LIVENESS_CHECK_TIMEOUT_US) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: connection lost");
            crsf_port->reset_bootstrap_baudrate();
            baud_negotiation_result = BaudNegotiationResult::PENDING;
            state = State::WAITING_FOR_RC_LOCK;
        }
        break;
    }
}

bool AP_CRSF_Out::decode_crsf_packet(const AP_CRSF_Protocol::Frame& _frame)
{
#ifdef CRSF_RCOUT_DEBUG_FRAME
    hal.console->printf("CRSFOut: received %s:", AP_CRSF_Protocol::get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
        hal.console->printf(" 0x%x", fptr[i]);
    }
    hal.console->printf("\n");
#endif

    switch (_frame.type) {
        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_COMMAND: {
            const AP_CRSF_Protocol::CommandFrame* cmd = (const AP_CRSF_Protocol::CommandFrame*)_frame.payload;
            if (cmd->origin == AP_CRSF_Protocol::DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                cmd->command_id == AP_CRSF_Protocol::CRSF_COMMAND_GENERAL &&
                cmd->payload[0] == AP_CRSF_Protocol::CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE) {
                const bool success = cmd->payload[2];
                if (success) {
                    debug_rcout("CRSF Speed Response Received: SUCCESS");
                    baud_negotiation_result = BaudNegotiationResult::SUCCESS;
                    // change baud on our end now
                    crsf_port->change_baud_rate(target_baudrate);
                } else {
                    debug_rcout("CRSF Speed Response Received: FAILED");
                    baud_negotiation_result = BaudNegotiationResult::FAILED;
                }
            }
        }
            break;
        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
            if (last_latency_ping_us > 0) {
                latency_us = AP_HAL::micros() - last_latency_ping_us;
                last_latency_ping_us = 0;
            }
            AP_CRSF_Protocol::process_device_info_frame((AP_CRSF_Protocol::ParameterDeviceInfoFrame*)_frame.payload,
                                                         &version, true);
            break;

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
            send_device_info();
            break;

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_ACCGYRO: {
            Vector3f acc, gyro;
            float gyro_temp;
            uint32_t sample_us;
            if (AP_CRSF_Protocol::process_accgyro_frame((AP_CRSF_Protocol::AccGyroFrame*)_frame.payload, acc, gyro, gyro_temp, sample_us)) {
                rate_imu_counter++;
                imu_cal_count++;
                // Pass the decoded IMU data to the external AHRS CRSF module
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                AP_ExternalAHRS_CRSF* crsf_ahrs = AP::external_ahrs_crsf();
                if (crsf_ahrs != nullptr) {
                    // Pass this instance's index for filtering in the AHRS backend
                    crsf_ahrs->handle_acc_gyro_frame(_instance_idx, acc, gyro, gyro_temp, sample_us);
                }
#endif
            }
            break;
        }

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_BARO: {
            float pressure, temperature;
            if (AP_CRSF_Protocol::process_baro_frame((AP_CRSF_Protocol::BaroFrame*)_frame.payload, pressure, temperature)) {
                rate_baro_counter++;
                // Pass the decoded Baro data to the external AHRS CRSF module
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                AP_ExternalAHRS_CRSF* crsf_ahrs = AP::external_ahrs_crsf();
                if (crsf_ahrs != nullptr) {
                    // Pass this instance's index for filtering in the AHRS backend
                    crsf_ahrs->handle_baro_frame(_instance_idx, pressure, temperature);
                }
#endif
            }
            break;
        }

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_MAG: {
            Vector3f mag_field;
            if (AP_CRSF_Protocol::process_mag_frame((AP_CRSF_Protocol::MagFrame*)_frame.payload, mag_field)) {
                rate_mag_counter++;
                // Pass the decoded Mag data to the external AHRS CRSF module
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
                AP_ExternalAHRS_CRSF* crsf_ahrs = AP::external_ahrs_crsf();
                if (crsf_ahrs != nullptr) {
                    // Pass this instance's index for filtering in the AHRS backend
                    crsf_ahrs->handle_mag_frame(_instance_idx, mag_field);
                }
#endif
            }
            break;
        }

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_GPS_EXTENDED:
            AP_CRSF_Protocol::process_extended_gps_frame((AP_CRSF_Protocol::GPSExtendedFrame*)_frame.payload, &gps_state);
            break;

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_GPS_TIME:
            AP_CRSF_Protocol::process_gps_time_frame((AP_CRSF_Protocol::GPSTimeFrame*)_frame.payload, &gps_state);
            break;

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_GPS: {
            //debug_rcout("CRSF_FRAMETYPE_GPS");
            AP_CRSF_Protocol::process_gps_frame((AP_CRSF_Protocol::GPSFrame*)_frame.payload, &gps_state);
            rate_gps_counter++;
            // Pass the decoded gps data to the external AHRS CRSF module
#if AP_EXTERNAL_AHRS_CRSF_ENABLED
            AP_ExternalAHRS_CRSF* crsf_ahrs = AP::external_ahrs_crsf();
            if (crsf_ahrs != nullptr) {
                // Pass this instance's index for filtering in the AHRS backend
                crsf_ahrs->handle_gps_frame(_instance_idx, gps_state);
            }
#endif
            break;
        }
        default:
            break;
    }

    return true;
}

// send control frame, this goes out at the loop rate and so
// needs to be kept small
void AP_CRSF_Out::send_aetr_rc_frame()
{
    send_rc_frame(0, 4);
    rate_rc_counter++;
}

// send aux frame, this can go out at a low rate - 50Hz
void AP_CRSF_Out::send_aux_rc_frame()
{
    send_rc_frame(4, MIN(NUM_SERVO_CHANNELS, (uint8_t)CRSF_MAX_CHANNELS) - 4);
}

// sends RC frames at the configured rate
void AP_CRSF_Out::send_rc_frame(uint8_t start_chan, uint8_t nchan)
{
    uint16_t channels[CRSF_MAX_CHANNELS] {};
    const bool armed = hal.util->get_soft_armed();

    for (uint8_t i = start_chan; i < start_chan + nchan; ++i) {
        SRV_Channel *c = SRV_Channels::srv_channel(i);

        if (c == nullptr) { // Default to neutral if channel is null
            channels[i] = 1500;
            continue;
        }

        // When disarmed, output trim values for AETR channels (0-3) to prevent
        // feedback loops with external flight controllers like BetaFlight
        if (!armed && i < 4) {
            channels[i] = c->get_trim();
            continue;
        }

        // don't pass on the arming on a switch channel when disarmed in case the other end is also using this for arming
        SRV_Channel::Function func = c->get_function();
        if (func >= SRV_Channel::Function::k_rcin1 && func <= SRV_Channel::Function::k_rcin16) {
            RC_Channels* rc = RC_Channels::get_singleton();
            if (rc != nullptr) {
                RC_Channel *chan = rc->channel(func - SRV_Channel::Function::k_rcin1);
                if (!armed
                    && (chan->option.get() == int16_t(RC_Channel::AUX_FUNC::ARMDISARM)
                        || chan->option.get() == int16_t(RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE))) {
                    channels[i] = 1000;
                    continue;
                }
            }
        }

        channels[i] = c->get_output_pwm();
    }

    AP_CRSF_Protocol::Frame frame {};

    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = FrameType::CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED;
    uint8_t payload_len = AP_CRSF_Protocol::encode_variable_bit_channels(frame.payload, channels, nchan, start_chan);
    frame.length = payload_len + 2; // +1 for type, +1 for CRC

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_latency_ping_frame()
{
    last_latency_ping_us = AP_HAL::micros();
    send_ping_frame(true);
}

void AP_CRSF_Out::send_ping_frame(bool force)
{
    // only send pings at 50Hz max
    const uint32_t now_ms = AP_HAL::millis();
    if (!AP_HAL::timeout_expired(last_ping_frame_ms, now_ms, 20UL) && !force) {
        return;
    }
    last_ping_frame_ms = now_ms;

    debug_rcout("send_ping_frame()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_ping_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

// send a baudrate proposal
void AP_CRSF_Out::send_speed_proposal(uint32_t baudrate)
{
    debug_rcout("send_speed_proposal(%u)", (unsigned)baudrate);

    crsf_port->change_baud_rate(baudrate);

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_speed_proposal(baudrate, frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_device_info()
{
    debug_rcout("send_device_info()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_device_info_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_link_stats_tx()
{
    debug_rcout("send_link_stats_tx()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_link_stats_tx_frame(get_configured_update_rate(), frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_heartbeat()
{
    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_heartbeat_frame(frame, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

namespace AP {
    AP_CRSF_Out* crsf_out() {
        return AP_CRSF_Out::get_singleton();
    }
};

#endif // AP_CRSF_OUT_ENABLED