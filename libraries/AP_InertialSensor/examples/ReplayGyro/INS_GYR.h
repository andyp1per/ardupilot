#pragma once

#include "LogReader.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>

class INS_GYR : public AP_InertialSensor_Backend
{
public:
    INS_GYR(AP_InertialSensor &imu, LogReader& reader);

    /* update accel and gyro state */
    bool update() override;
    void start() override;
    void accumulate() override;

private:
    uint8_t instances;
    LogReader& reader;
};
