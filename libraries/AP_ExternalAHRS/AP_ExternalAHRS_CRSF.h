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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_CRSF_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_RCProtocol/AP_CRSF_Protocol.h>
#include <AP_Math/vector3.h>
#include <AP_HAL/AP_HAL.h>

class AP_ExternalAHRS_CRSF: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_CRSF(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled. CRSF AHRS is passive, so this is -1.
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // This module is passive (data is pushed to it), so the update() is a no-op.
    void update() override {};

    // Mandatory method to receive the decoded AccGyro frame data from AP_CRSF_Out.
    // The instance_idx identifies which AP_CRSF_Out is sending the data.
    void handle_acc_gyro_frame(uint8_t instance_idx, const Vector3f &acc, const Vector3f &gyro);

    // Global accessor for the singleton instance, used by AP_CRSF_Out
    static AP_ExternalAHRS_CRSF* get_singleton();

    // Set the expected CRSF instance index from the parameter system (EAHRS_CRSF_IDX)
    void set_instance_idx(uint8_t instance_idx) {
        _instance_idx = instance_idx;
    }

    // Helper function for the AHRS frontend. Due to CRSF-specific API issues, this is simplified.
    static int8_t get_default_instance_index();

protected:
    // CRSF IMU does not provide a GPS feed.
    uint8_t num_gps_sensors(void) const override {
        return 0;
    }

private:

    // Data structure to hold the last received IMU data
    struct CRSF_IMU_Data {
        Vector3f acc; // Acceleration (m/s/s)
        Vector3f gyro; // Angular velocity (rad/s)
    } imu_data;

    // Singleton pointer
    static AP_ExternalAHRS_CRSF* _singleton;
    uint32_t last_imu_pkt_ms; // Timestamp of the last received packet in milliseconds

    // The CRSF index (0, 1, 2...) that this AHRS is configured to listen to.
    uint8_t _instance_idx;
};

namespace AP {
    AP_ExternalAHRS_CRSF* external_ahrs_crsf();
};

#endif // AP_EXTERNAL_AHRS_CRSF_ENABLED