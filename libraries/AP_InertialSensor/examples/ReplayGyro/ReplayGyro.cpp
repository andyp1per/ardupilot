#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Arming/AP_Arming.h>
#include <SITL/SITL.h>
#include "LogReader.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static const uint32_t LOOP_RATE_HZ = 400;
static const uint32_t LOOP_DELTA_US = 1000000 / LOOP_RATE_HZ;

void setup();
void loop();

static AP_SerialManager serial_manager;
static AP_BoardConfig board_config;
static AP_InertialSensor ins;
static AP_Baro baro;
AP_Int32 logger_bitmask;
static AP_Logger logger{logger_bitmask};
#if HAL_EXTERNAL_AHRS_ENABLED
static AP_ExternalAHRS external_ahrs;
#endif
static SITL::SIM sitl;
static AP_Scheduler scheduler;

// create fake gcs object
GCS_Dummy _gcs;

const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES
};

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

class Arming : public AP_Arming {
public:
    Arming() : AP_Arming() {}
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override {
        armed = true;
        return true;
    }
};

static Arming arming;
const char *filename;

class ReplayGyro {
public:
    void init() {
        if (!reader.open_log(filename)) {
            ::printf("open(%s): %m\n", filename);
            exit(1);
        }
    }

    void loop() {
        // calibrate the FFT
        uint32_t now = AP_HAL::millis();
        if (now - last_output_ms > 1000) {
            hal.console->printf(".");
            last_output_ms = now;
        }

        if (!reader.update()) {
            exit(0);
        }
    }
    //AP_InertialSensor fft;
    uint32_t last_output_ms;

    LogReader reader{};
};

static ReplayGyro replay;

void setup()
{
    uint8_t argc;
    char * const *argv;

    hal.util->commandline_arguments(argc, argv);

    if (argc > 0) {
        filename = argv[0];
    }

    hal.console->printf("ReplayGyro\n");
    board_config.init();   
    serial_manager.init();

    logger_bitmask.set(128);    // IMU
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
    ins.init(LOOP_RATE_HZ);
    baro.init();

    replay.init();
}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }

    ins.wait_for_sample();
    uint32_t sample_time_us = AP_HAL::micros();

    ins.update();
    ins.periodic();
    logger.periodic_tasks();
    ins.Write_IMU();
    replay.loop();

    uint32_t elapsed = AP_HAL::micros() - sample_time_us;
    if (elapsed < LOOP_DELTA_US) {
        hal.scheduler->delay_microseconds(LOOP_DELTA_US - elapsed);
    }
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
