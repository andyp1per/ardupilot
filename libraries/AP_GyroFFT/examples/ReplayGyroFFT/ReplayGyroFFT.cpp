#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <SITL/SITL.h>

#if HAL_WITH_DSP
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static const uint32_t LOOP_RATE_HZ = 400;
static const uint32_t LOOP_DELTA_US = 1000000 / LOOP_RATE_HZ;

void setup();
void loop();

static AP_SerialManager serial_manager;
static AP_BoardConfig board_config;
static AP_InertialSensor ins;
static AP_GyroFFT fft;
static AP_Baro baro;
AP_Int32 logger_bitmask;
static AP_Logger logger{logger_bitmask};
#if HAL_EXTERNAL_AHRS_ENABLED
static AP_ExternalAHRS external_ahrs;
#endif
static SITL::SIM sitl;
static AP_Scheduler scheduler;

class DummyVehicle {
public:
};

//static DummyVehicle vehicle;
// create fake gcs object
GCS_Dummy _gcs;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
        AP_GROUPEND
};

uint32_t frame_num = 0;

void setup()
{
    hal.console->printf("ReplayGyroFFT\n");
    board_config.init();   
    serial_manager.init();
    ins.init(LOOP_RATE_HZ);
    baro.init();
    fft.init(LOOP_RATE_HZ);
    fft.update_parameters();
}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }

    ins.update();
    fft.sample_gyros();
    fft.update();

    hal.scheduler->delay(LOOP_DELTA_US);
}

AP_HAL_MAIN();

#else

#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void loop() { }
static void setup()
{
    printf("Board not currently supported\n");
}

AP_HAL_MAIN();

#endif
