#include <GCS_MAVLink/GCS.h>

#include <sys/types.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/DroneShowNotificationBackend.h>
#include <AP_Param/AP_Param.h>

#include "AC_DroneShowManager.h"
#include <AC_Fence/AC_Fence.h>

#include <skybrush/skybrush.h>

#include "DroneShow_Constants.h"
#include "DroneShow_CustomPackets.h"
#include "DroneShowLEDFactory.h"
#include "DroneShowPyroDeviceFactory.h"

extern const AP_HAL::HAL &hal;

// LED factory that is used to create new RGB LED instances
static DroneShowLEDFactory _rgb_led_factory_singleton;

// Pyro device factory that is used to create new pyro device instances
static DroneShowPyroDeviceFactory _pyro_device_factory_singleton;

AC_DroneShowManager::AC_DroneShowManager() :
    hard_fence(),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _sock_rgb(true),
    _sock_rgb_open(false),
#endif
    _show_data(0),
    _trajectory_valid(false),
    _light_program_valid(false),
    _yaw_control_valid(false),
    _stage_in_drone_show_mode(DroneShow_Off),
    _start_time_requested_by(StartTimeSource::NONE),
    _start_time_on_internal_clock_usec(0),
    _start_time_unix_usec(0),
    _takeoff_time_sec(0),
    _landing_time_sec(0),
    _crtl_start_time_sec(0),
    _total_duration_sec(0),
    _trajectory_is_circular(false),
    _cancel_requested(false),
    _controller_update_delta_msec(1000 / DEFAULT_UPDATE_RATE_HZ),
    _pyro_device(0),
    _rgb_led(0),
    _rc_switches_blocked_until(0),
    _boot_count(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    _trajectory = new sb_trajectory_t;
    sb_trajectory_init_empty(_trajectory);

    _trajectory_player = new sb_trajectory_player_t;
    sb_trajectory_player_init(_trajectory_player, _trajectory);

    _light_program = new sb_light_program_t;
    sb_light_program_init_empty(_light_program);

    _light_player = new sb_light_player_t;
    sb_light_player_init(_light_player, _light_program);

    _yaw_control = new sb_yaw_control_t;
    sb_yaw_control_init_empty(_yaw_control);

    _yaw_player = new sb_yaw_player_t;
    sb_yaw_player_init(_yaw_player, _yaw_control);

    _event_list = new sb_event_list_t;
    sb_event_list_init(_event_list, 0);

    _event_list_player = new sb_event_list_player_t;
    sb_event_list_player_init(_event_list_player, _event_list);

    // Don't call _update_rgb_led_instance() or _update_pyro_device_instance()
    // here, servo framework is not set up yet
}

AC_DroneShowManager::~AC_DroneShowManager()
{
    sb_event_list_player_destroy(_event_list_player);
    delete _event_list_player;

    sb_event_list_destroy(_event_list);
    delete _event_list;

    sb_yaw_player_destroy(_yaw_player);
    delete _yaw_player;

    sb_yaw_control_destroy(_yaw_control);
    delete _yaw_control;

    sb_light_player_destroy(_light_player);
    delete _light_player;

    sb_light_program_destroy(_light_program);
    delete _light_program;

    sb_trajectory_player_destroy(_trajectory_player);
    delete _trajectory_player;

    sb_trajectory_destroy(_trajectory);
    delete _trajectory;
}

void AC_DroneShowManager::early_init()
{
    _create_show_directory();
}

void AC_DroneShowManager::init(const AC_WPNav* wp_nav)
{
    // Get the boot count from the parameters
    enum ap_var_type ptype;
    AP_Int16* boot_count_param = static_cast<AP_Int16*>(AP_Param::find("STAT_BOOTCNT", &ptype));
    _boot_count = boot_count_param ? (*boot_count_param) : 0;

    // Get references to the RGB LED and pyro device factories
    _pyro_device_factory = &_pyro_device_factory_singleton;
    _rgb_led_factory = &_rgb_led_factory_singleton;
    
    // Store a reference to wp_nav so we can ask what the takeoff speed will be
    _wp_nav = wp_nav;

    // Clear start time and authorization now; at this point the parameter
    // subsystem has already loaded back the previous value from the EEPROM so
    // we are safe to overwrite it
    _params.start_time_gps_sec.set(-1);
    _params.authorization.set(DroneShowAuthorization_Revoked);

    _load_show_file_from_storage();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _open_rgb_led_socket();
#endif
    _update_pyro_device_instance();
    _update_rgb_led_instance();

    // initialise safety features
    hard_fence.init();
    bubble_fence.init();
}


bool AC_DroneShowManager::clear_scheduled_start_time(bool force)
{
    if (!force && _stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    _params.start_time_gps_sec.set(-1);
    _start_time_on_internal_clock_usec = 0;
    _start_time_requested_by = StartTimeSource::NONE;
    _start_time_unix_usec = 0;

    return true;
}

bool AC_DroneShowManager::configure_show_coordinate_system(
    int32_t lat, int32_t lng, int32_t amsl_mm, float orientation_deg
) {
    if (!check_latlng(lat, lng)) {
        return false;
    }

    if (amsl_mm >= LARGEST_VALID_AMSL) {
        return false;
    }

    // We need to set the new values _and_ save them to the EEPROM. The save
    // operation is asynchronous; it might not go through immediately, but it
    // should go through in a few milliseconds.
    _params.origin_lat.set_and_save(lat);
    _params.origin_lng.set_and_save(lng);
    _params.origin_amsl_mm.set_and_save(amsl_mm);
    _params.orientation_deg.set_and_save(orientation_deg);

#if HAL_LOGGING_ENABLED
    // Log the new values
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter("SHOW_ORIGIN_LAT", static_cast<float>(lat));
        logger->Write_Parameter("SHOW_ORIGIN_LNG", static_cast<float>(lng));
        logger->Write_Parameter("SHOW_ORIGIN_AMSL", static_cast<float>(amsl_mm));
        logger->Write_Parameter("SHOW_ORIENTATION", static_cast<float>(orientation_deg));
    }
#endif

    return true;
}

AC_BubbleFence::FenceAction AC_DroneShowManager::get_bubble_fence_action()
{
    Vector3f dist;
    AC_BubbleFence::FenceAction action;

    // Check the distance from the desired position during the performance
    // only, not in any of the other stages
    if (get_stage_in_drone_show_mode() == DroneShow_Performing) {
        get_distance_from_desired_position(dist);
        action = bubble_fence.notify_distance_from_desired_position(dist);
    } else {
        action = AC_BubbleFence::FenceAction::NONE;
    }

    return action;
}

bool AC_DroneShowManager::get_current_guided_mode_command_to_send(
    GuidedModeCommand& command,
    int32_t default_yaw_cd,
    bool altitude_locked_above_takeoff_altitude
) {
    Location loc;

    static uint8_t invalid_velocity_warning_sent = 0;
    static uint8_t invalid_acceleration_warning_sent = 0;
    static uint8_t invalid_yaw_warning_sent = 0;
    static uint8_t invalid_yaw_rate_warning_sent = 0;
    // static uint8_t counter = 0;

    float elapsed = get_elapsed_time_since_start_sec();
    float yaw_cd = default_yaw_cd;
    float yaw_rate_cds = 0;
    
    get_desired_global_position_at_seconds(elapsed, loc);

    command.clear();
    command.yaw_cd = default_yaw_cd;

    if (loaded_yaw_control_data_successfully())
    {
        // TODO(vasarhelyi): handle auto yaw mode as well

        yaw_cd = get_desired_yaw_cd_at_seconds(elapsed);

        // Prevent invalid yaw information from leaking into the guided
        // mode controller
        if (isnan(yaw_cd) || isinf(yaw_cd))
        {
            if (!invalid_yaw_warning_sent)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid yaw command; not using yaw control");
                invalid_yaw_warning_sent = true;
            }
        }
        else
        {
            yaw_rate_cds = get_desired_yaw_rate_cds_at_seconds(elapsed);

            // Prevent invalid yaw rate information from leaking into the guided
            // mode controller
            if (isnan(yaw_rate_cds) || isinf(yaw_rate_cds))
            {
                if (!invalid_yaw_rate_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid yaw rate command; not using yaw control");
                    invalid_yaw_rate_warning_sent = true;
                }
            }
            else
            {
                command.yaw_cd = yaw_cd;
                command.yaw_rate_cds = yaw_rate_cds;
            }
        }
    }

    if (loc.get_vector_from_origin_NEU(command.pos))
    {
        /*
        counter++;
        if (counter > 4) {
            gcs().send_text(MAV_SEVERITY_INFO, "%.2f %.2f %.2f -- %.2f %.2f %.2f", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        }
        */

        if (is_velocity_control_enabled())
        {
            float gain = get_velocity_feedforward_gain();

            if (gain > 0)
            {
                get_desired_velocity_neu_in_cms_per_seconds_at_seconds(elapsed, command.vel);
                command.vel *= gain;
            }

            // Prevent invalid velocity information from leaking into the guided
            // mode controller
            if (command.vel.is_nan() || command.vel.is_inf())
            {
                if (!invalid_velocity_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid velocity command; using zero");
                    invalid_velocity_warning_sent = true;
                }
                command.vel.zero();
            }
        }

        if (is_acceleration_control_enabled())
        {
            get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(elapsed, command.acc);

            // Prevent invalid acceleration information from leaking into the guided
            // mode controller
            if (command.acc.is_nan() || command.acc.is_inf())
            {
                if (!invalid_acceleration_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid acceleration command; using zero");
                    invalid_acceleration_warning_sent = true;
                }
                command.acc.zero();
            }
        }

        // Prevent the drone from temporarily sinking below the takeoff altitude
        // if the "real" trajectory has a slow takeoff
        if (altitude_locked_above_takeoff_altitude) {
            int32_t target_altitude_above_home_cm;
            int32_t takeoff_altitude_cm;

            if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_altitude_above_home_cm)) {
                takeoff_altitude_cm = get_takeoff_altitude_cm();
                if (target_altitude_above_home_cm < takeoff_altitude_cm) {
                    // clamp the position to the target altitude, and zero out
                    // the Z component of the velocity and the acceleration
                    loc.set_alt_cm(takeoff_altitude_cm, Location::AltFrame::ABOVE_HOME);
                    if (loc.get_vector_from_origin_NEU(command.pos)) {
                        command.vel.z = 0;
                        command.acc.z = 0;
                    } else {
                        // this should not happen either, but let's handle this
                        // gracefully
                        command.unlock_altitude = true;
                    }
                } else {
                    // we want to go above the takeoff altitude so we can
                    // release the lock
                    command.unlock_altitude = true;
                }
            } else {
                // let's not blow up if get_alt_cm() fails, it's not mission-critical,
                // just release the lock
                command.unlock_altitude = true;
            }
        }

        // Prevent invalid position information from leaking into the guided
        // mode controller
        if (command.pos.is_nan() || command.pos.is_inf())
        {
            return false;
        }

        return true;
    }
    else
    {
        // No EKF origin yet, this should not have happened
        return false;
    }
}

void AC_DroneShowManager::get_desired_global_position_at_seconds(float time, Location& loc)
{
    sb_vector3_with_yaw_t vec;
    sb_trajectory_player_get_position_at(_trajectory_player, time, &vec);
    _show_coordinate_system.convert_show_to_global_coordinate(vec, loc);
}

void AC_DroneShowManager::get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel)
{
    sb_vector3_with_yaw_t vec;
    float vel_north, vel_east;
    float orientation_rad = _show_coordinate_system.orientation_rad;

    sb_trajectory_player_get_velocity_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    vel_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    vel_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have mm/s so far, need to convert to cm/s
    vel.x = vel_north / 10.0f;
    vel.y = vel_east / 10.0f;
    vel.z = vec.z / 10.0f;
}

void AC_DroneShowManager::get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(float time, Vector3f& acc)
{
    sb_vector3_with_yaw_t vec;
    float acc_north, acc_east;
    float orientation_rad = _show_coordinate_system.orientation_rad;

    sb_trajectory_player_get_acceleration_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    acc_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    acc_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have mm/s/s so far, need to convert to cm/s/s
    acc.x = acc_north / 10.0f;
    acc.y = acc_east / 10.0f;
    acc.z = vec.z / 10.0f;
}

float AC_DroneShowManager::get_desired_yaw_cd_at_seconds(float time)
{
    float value;
    sb_yaw_player_get_yaw_at(_yaw_player, time, &value);

    return _show_coordinate_system.convert_show_to_global_yaw_and_scale_to_cd(value);
}

float AC_DroneShowManager::get_desired_yaw_rate_cds_at_seconds(float time)
{
    float value;
    sb_yaw_player_get_yaw_rate_at(_yaw_player, time, &value);

    return value * 100.0f; /* [deg] -> [cdeg] */
}

void AC_DroneShowManager::get_distance_from_desired_position(Vector3f& vec) const
{
    if (_stage_in_drone_show_mode == DroneShow_Performing) {
        if (!get_current_relative_position_NED_origin(vec)) {
            // EKF does not know its own position yet
            vec.zero();
        } else {
            // Setpoints are in centimeters, so we need to convert the units.
            // Furthermore, the relative position is given in NED but the
            // setpoint is in NEU so we need to invert the Z axis.
            vec.z *= -1;
            vec -= (_last_setpoint.pos / 100.0f);
        }
    } else {
        vec.zero();
    }
}

// returns the elapsed time since the start of the show, in microseconds
int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    uint64_t now, reference, diff;
    
    // AP::gps().time_epoch_usec() is smart enough to handle the case when
    // the GPS fix was lost so no need to worry about loss of GPS fix here.
    if (uses_gps_time_for_show_start()) {
        now = AP::gps().time_epoch_usec();
        reference = _start_time_unix_usec;
    } else {
        now = AP_HAL::micros64();
        reference = _start_time_on_internal_clock_usec;
    }

    if (reference > 0) {
        if (reference > now) {
            diff = reference - now;
            if (diff < INT64_MAX) {
                return -diff;
            } else {
                return INT64_MIN;
            }
        } else if (reference < now) {
            diff = now - reference;
            if (diff < INT64_MAX) {
                return diff;
            } else {
                return INT64_MAX;
            }
        } else {
            return 0;
        }
    } else {
        return INT64_MIN;
    }
}

int32_t AC_DroneShowManager::get_elapsed_time_since_start_msec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    if (elapsed_usec <= -86400000000) {
        return -86400000;
    } else if (elapsed_usec >= 86400000000) {
        return 86400000;
    } else {
        return static_cast<int32_t>(elapsed_usec / 1000);
    }
}

float AC_DroneShowManager::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    return elapsed_usec == INT64_MIN ? -86400 : static_cast<float>(elapsed_usec / 1000) / 1000.0f;
}

int64_t AC_DroneShowManager::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

float AC_DroneShowManager::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

float AC_DroneShowManager::get_time_until_landing_sec() const
{
    return get_time_until_start_sec() + get_relative_landing_time_sec();
}

void AC_DroneShowManager::notify_drone_show_mode_initialized()
{
    _cancel_requested = false;
    _update_pyro_device_instance();
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
}

void AC_DroneShowManager::notify_drone_show_mode_entered_stage(DroneShowModeStage stage)
{
    if (stage == _stage_in_drone_show_mode) {
        return;
    }

    _stage_in_drone_show_mode = stage;

    // Whenever we change the state, we clear the scheduled start time of a
    // collective RTL trajectory
    clear_scheduled_collective_rtl(/* force = */ true);

    // Force-update preflight checks so we see the errors immediately if we
    // switched to the "waiting for start time" stage
    _update_preflight_check_result(/* force = */ true);

    // Call callbacks for certain stages
    if (_stage_in_drone_show_mode == DroneShow_Landed) {
        _handle_switch_to_landed_state();
    }
}

void AC_DroneShowManager::notify_drone_show_mode_exited()
{
    _cancel_requested = false;
    _update_pyro_device_instance();
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
    _last_setpoint.clear();
}

void AC_DroneShowManager::notify_guided_mode_command_sent(const GuidedModeCommand& command)
{
    _last_setpoint = command;   
}

void AC_DroneShowManager::handle_rc_start_switch()
{
    if (_are_rc_switches_blocked())
    {
        return;
    }

    if (schedule_delayed_start_after(10000 /* msec */)) {
        // Rewrite the start time source to be "RC switch", not
        // "start method", even though we implemented it using
        // the schedule_delayed_start_after() method
        if (_start_time_requested_by == StartTimeSource::START_METHOD) {
            _start_time_requested_by = StartTimeSource::RC_SWITCH;
        }
    }
}

bool AC_DroneShowManager::schedule_delayed_start_after(uint32_t delay_ms)
{
    bool success = false;

    _cancel_requested = false;

    if (_stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    if (uses_gps_time_for_show_start()) {
        if (_is_gps_time_ok()) {
            // We are modifying a parameter directly here without notifying the
            // param subsystem, but this is okay -- we do not want to save the
            // start time into the EEPROM, and it is reset at the next boot anyway.
            // Delay is rounded down to integer seconds.
            _params.start_time_gps_sec.set(((AP::gps().time_week_ms() + delay_ms) / 1000) % GPS_WEEK_LENGTH_SEC);
            success = true;
        }
    } else {
        _start_time_on_internal_clock_usec = AP_HAL::micros64() + (delay_ms * 1000);
        success = true;
    }

    if (success) {
        _start_time_requested_by = StartTimeSource::START_METHOD;
    }

    return success;
}

void AC_DroneShowManager::stop_if_running()
{
    _cancel_requested = true;
}

void AC_DroneShowManager::update()
{
    static bool main_cycle = true;

    if (main_cycle) {
        _check_changes_in_parameters();
        _check_events();
        _check_radio_failsafe();
        _update_preflight_check_result();
        _trigger_show_events();
        _update_lights();
        _update_pyro_device();
    } else {
        _repeat_last_rgb_led_command();
    }

    main_cycle = !main_cycle;
}

bool AC_DroneShowManager::_are_rc_switches_blocked()
{
    return _rc_switches_blocked_until && _rc_switches_blocked_until >= AP_HAL::millis();
}

void AC_DroneShowManager::_check_events()
{
    DroneShowNotificationBackend* backend = DroneShowNotificationBackend::get_singleton();

    if (DroneShowNotificationBackend::events.compass_cal_failed) {
        _flash_leds_after_failure();
    } else if (DroneShowNotificationBackend::events.compass_cal_saved) {
        _flash_leds_after_success();
    }

    if (backend) {
        backend->clear_events();
    }
}

void AC_DroneShowManager::_check_radio_failsafe()
{
    if (AP_Notify::flags.failsafe_radio) {
        // Block the handling of RC switches for the next second so we don't
        // accidentally trigger a function if the user changes the state of the
        // RC switch while we are not connected to the RC.
        //
        // (E.g., switch is low, drone triggers RC failsafe because it is out
        // of range, user changes the switch, then the drone reconnects)
        _rc_switches_blocked_until = AP_HAL::millis() + 1000;
    }
}

void AC_DroneShowManager::_clear_start_time_if_set_by_switch()
{
    if (_start_time_requested_by == StartTimeSource::RC_SWITCH) {
        clear_scheduled_start_time(/* force = */ true);
    }
 }

bool AC_DroneShowManager::_is_at_expected_position() const
{
    if (_stage_in_drone_show_mode != DroneShow_Performing)
    {
        // Not performing a show; any position is suitable
        return true;
    }

    Location expected_loc(_last_setpoint.pos.tofloat(), Location::AltFrame::ABOVE_ORIGIN);
    return _is_close_to_position(
        expected_loc, _params.max_xy_drift_during_show_m,
        _params.max_z_drift_during_show_m
    );
}

bool AC_DroneShowManager::_is_close_to_position(
    const Location& target_loc, float xy_threshold, float z_threshold
) const
{
    Location current_loc;
    ftype alt_dist;

    if (!get_current_location(current_loc)) {
        // EKF does not know its own position yet so we report that we are not
        // at the target position
        return false;
    }

    // Location.get_distance() checks XY distance only so this is okay
    if (xy_threshold > 0 && current_loc.get_distance(target_loc) > xy_threshold) {
        return false;
    }

    if (z_threshold > 0) {
        if (!current_loc.get_alt_distance(target_loc, alt_dist)) {
            // Altitude frame is not usable; this should not happen
            return false;
        }

        if (alt_dist > z_threshold) {
            return false;
        }
    }

    return true;
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}

void AC_DroneShowManager::_update_preflight_check_result(bool force)
{
    static uint32_t last_updated_at = 0;

    uint32_t now = AP_HAL::millis();
    if (!force && (now - last_updated_at) < 1000) {
        return;
    }

    last_updated_at = now;

    _preflight_check_failures = 0;

    if (_stage_in_drone_show_mode != DroneShow_WaitForStartTime) {
        /* We are not in the "waiting for start time" stage so we don't perform
         * any checks */
        return;
    }

    if (
        !loaded_show_data_successfully() ||
        !has_explicit_show_origin_set_by_user() ||
        !has_explicit_show_orientation_set_by_user()
    ) {
        _preflight_check_failures |= DroneShowPreflightCheck_ShowNotConfiguredYet;
    }

    if (_tentative_show_coordinate_system.is_valid() && !_is_at_takeoff_position_xy()) {
        _preflight_check_failures |= DroneShowPreflightCheck_NotAtTakeoffPosition;
    }
}

void AC_DroneShowManager::ShowCoordinateSystem::clear()
{
    origin_lat = origin_lng = origin_amsl_mm = 0;
    orientation_rad = 0;
    origin_amsl_valid = false;
}

float AC_DroneShowManager::ShowCoordinateSystem::convert_show_to_global_yaw_and_scale_to_cd(
    float yaw
) const {
    // show coordinates are in degrees relative to X axis orientation,
    // we need centidegrees relative to North
    return (degrees(orientation_rad) + yaw) * 100.0f;
}

void AC_DroneShowManager::ShowCoordinateSystem::convert_global_to_show_coordinate(
    const Location& loc, sb_vector3_with_yaw_t& vec
) const {
    Location origin;
    Vector3f diff;

    origin.zero();
    origin.lat = origin_lat;
    origin.lng = origin_lng;

    if (origin_amsl_valid) {
        // Show is controlled in AMSL
        origin.set_alt_cm(
            origin_amsl_mm / 10.0 /* [mm] -> [cm] */,
            Location::AltFrame::ABSOLUTE
        );
    } else {
        // Show is controlled in AGL. We use altitude above home because the
        // EKF origin could be anywhere -- it is typically established early
        // during the initialization process, while the home is set to the
        // point where the drone is armed.
        origin.set_alt_cm(0.0f, Location::AltFrame::ABOVE_HOME);
    }

    diff = origin.get_distance_NED_alt_frame(loc);

    diff.x *= 1000.0f;   // [m] -> [mm]
    diff.y *= -1000.0f;   // [m] -> [mm], East --> West
    diff.z *= -10.0f;     // [cm] -> [mm], Down --> Up

    // We need to rotate the X axis by -orientation_rad radians so it points
    // towards the orientation of the show axis
    vec.x = cosf(-orientation_rad) * diff.x + sinf(-orientation_rad) * diff.y;
    vec.y = -sinf(-orientation_rad) * diff.x + cosf(-orientation_rad) * diff.y;
    vec.z = diff.z;
}

void AC_DroneShowManager::ShowCoordinateSystem::convert_show_to_global_coordinate(
    sb_vector3_with_yaw_t vec, Location& loc
) const {
    float offset_north, offset_east, altitude;
    
    // We need to rotate the X axis by -orientation_rad radians so it points
    // North. At the same time, we also flip the Y axis so it points East and
    // not West.
    offset_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    offset_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have millimeters so far, need to convert the North and East offsets
    // to meters in the XY plane first. In the Z axis, we will need centimeters.
    offset_north = offset_north / 1000.0f; // [mm] -> [m]
    offset_east = offset_east / 1000.0f;   // [mm] -> [m]
    altitude = vec.z / 10.0f;              // [mm] -> [cm]

    // Finally, we need to offset the show origin with the calculated North and
    // East offset to get a global position

    loc.zero();
    loc.lat = origin_lat;
    loc.lng = origin_lng;

    if (origin_amsl_valid) {
        // Show is controlled in AMSL
        loc.set_alt_cm(
            static_cast<int32_t>(altitude) /* [cm] */ +
            origin_amsl_mm / 10.0 /* [mm] -> [cm] */,
            Location::AltFrame::ABSOLUTE
        );
    } else {
        // Show is controlled in AGL. We use altitude above home because the
        // EKF origin could be anywhere -- it is typically established early
        // during the initialization process, while the home is set to the
        // point where the drone is armed.
        loc.set_alt_cm(
            static_cast<int32_t>(altitude) /* [cm] */,
            Location::AltFrame::ABOVE_HOME
        );
    }

    loc.offset(offset_north, offset_east);
}
