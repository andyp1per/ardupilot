#include <AP_HAL/AP_HAL.h>
#include "INS_GYR.h"
#include <AP_Logger/AP_Logger.h>
#include <fcntl.h>

const extern AP_HAL::HAL& hal;

static const uint32_t LOOP_RATE_HZ = 400;
static const uint32_t LOOP_DELTA_US = 1000000 / LOOP_RATE_HZ;

INS_GYR::INS_GYR(AP_InertialSensor &imu, LogReader& _reader) :
    AP_InertialSensor_Backend(imu), reader(_reader)
{
}

bool INS_GYR::update(void) 
{
    for (uint8_t i=0; i<instances; i++) {
        update_accel(i);
        update_gyro(i);
    }
    return true;
}

void INS_GYR::accumulate()
{
    uint64_t sample_time_us = AP_HAL::micros64();
    uint64_t gyr_time_us;
    do {
        const log_GYR& gyr = reader.get_next_gyr_msg();
        gyr_time_us = gyr.time_us;
        Vector3f gyro(gyr.GyrX, gyr.GyrY, gyr.GyrY);
        instances = MAX(gyr.instance + 1, instances);
        _rotate_and_correct_gyro(gyr.instance, gyro);
        _notify_new_gyro_raw_sample(gyr.instance, gyro, gyr.time_us);
    }
    while (gyr_time_us < sample_time_us + LOOP_DELTA_US);
}

void INS_GYR::start()
{
    for (uint8_t i=0; i<3; i++) {
        uint8_t instance;
        _imu.register_gyro(instance, 1000,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 0, 1, DEVTYPE_SITL));
    }
}
