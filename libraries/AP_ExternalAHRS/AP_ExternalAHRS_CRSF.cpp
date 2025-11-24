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
  Support for external AHRS based on CRSF AccGyroFrame data.
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_CRSF_ENABLED

#include "AP_ExternalAHRS_CRSF.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_CRSF* AP_ExternalAHRS_CRSF::_singleton = nullptr;

// Constructor: Renaming 'frontend' to '_frontend' and 'state' to '_state' resolves the shadowing warning/error.
AP_ExternalAHRS_CRSF::AP_ExternalAHRS_CRSF(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    // The AP_ExternalAHRS frontend instantiates this class.
    // Since we are passive, we do not need to open a UART or create a thread.
    // We just register the singleton.
    if (_singleton == nullptr) {
        _singleton = this;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS_CRSF initialised");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS_CRSF already initialised");
    }

    // CRSF IMU only provides IMU data, so set the default sensors accordingly.
    // This allows the EAHRS_SENSORS parameter to default to only IMU.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
}

// Global accessor for the singleton instance
AP_ExternalAHRS_CRSF* AP_ExternalAHRS_CRSF::get_singleton()
{
    return _singleton;
}

// Global namespace accessor
namespace AP {
    AP_ExternalAHRS_CRSF* external_ahrs_crsf()
    {
        return AP_ExternalAHRS_CRSF::get_singleton();
    }
};

// Returns the default instance index. Due to API restrictions, this must return 0.
// Auto-detection logic is removed here to prevent layering violations.
int8_t AP_ExternalAHRS_CRSF::get_default_instance_index()
{
    return 0;
}


// Returns the external AHRS port number. Since CRSF is a passive consumer here, return -1.
int8_t AP_ExternalAHRS_CRSF::get_port(void) const
{
    return _instance_idx;
}

// Get model/type name
const char* AP_ExternalAHRS_CRSF::get_name() const
{
    return "CRSF";
}

// Handles the received and decoded IMU data from AP_CRSF_Out.
void AP_ExternalAHRS_CRSF::handle_acc_gyro_frame(uint8_t instance_idx, const Vector3f &accel, const Vector3f &gyro, const float gyro_temp)
{
    // CRITICAL: Only process data if the sender's index matches the configured primary CRSF source index.
    if (instance_idx != _instance_idx) {
        return;
    }

    WITH_SEMAPHORE(state.sem);

    state.accel = accel;
    state.gyro = gyro;

    last_imu_pkt_ms = AP_HAL::millis();

    // The CRSF frame provides raw accel and gyro data. We pass it through
    // AP::ins().handle_external as required for an external IMU.
    AP_ExternalAHRS::ins_data_message_t ins {
        accel: accel,
        gyro: gyro,
        // Set temperature to a low value to disable internal calibrations if we don't have a reading
        temperature: gyro_temp
    };
    AP::ins().handle_external(ins);
}

// Check if data is being received within a healthy window (e.g., last 10ms for high-rate IMU)
bool AP_ExternalAHRS_CRSF::healthy(void) const
{
    const uint32_t now = AP_HAL::millis();
    return (now - last_imu_pkt_ms < 10);
}

// Check if the AHRS has received an initial packet
bool AP_ExternalAHRS_CRSF::initialised(void) const
{
    return last_imu_pkt_ms != 0;
}

// Pre-arm check: ensure we are receiving data
bool AP_ExternalAHRS_CRSF::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "CRSF-IMU unhealthy (no recent data)");
        return false;
    }

    return true;
}

// Since this is only an IMU feed, we can only confirm attitude, not position or velocity.
void AP_ExternalAHRS_CRSF::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (initialised()) {
        status.flags.initalized = true;
    }
    if (healthy()) {
        // If we are healthy, we have attitude information from the IMU
        status.flags.attitude = true;
    }
}

// Get variances: since CRSF IMU provides raw data without covariance estimates,
// we return zero/default values, indicating no filter-derived data is available.
bool AP_ExternalAHRS_CRSF::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar = Vector3f();
    return false; // Return false to indicate no full AHRS data is being provided
}


#endif // AP_EXTERNAL_AHRS_CRSF_ENABLED