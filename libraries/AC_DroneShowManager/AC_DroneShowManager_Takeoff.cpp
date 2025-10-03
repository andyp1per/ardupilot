#include "AC_DroneShowManager.h"

#include <skybrush/skybrush.h>

#include <GCS_MAVLink/GCS.h>

bool AC_DroneShowManager::get_global_takeoff_position(Location& loc) const
{
    // This function may be called any time, not only during the show, so we
    // need to take the parameters provided by the user, convert them into a
    // ShowCoordinateSystem object, and then use that to get the GPS coordinates
    sb_vector3_with_yaw_t vec;

    if (!_tentative_show_coordinate_system.is_valid())
    {
        return false;
    }

    vec.x = _takeoff_position_mm.x;
    vec.y = _takeoff_position_mm.y;
    vec.z = _takeoff_position_mm.z;

    _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

    return true;
}

float AC_DroneShowManager::get_motor_spool_up_time_sec() const {
    float value = 0.0f;

    if (AP_Param::get("MOT_SPOOL_TIME", value)) {
        if (value >= 0.0f) {
            return value;
        }
    }

    return DEFAULT_MOTOR_SPOOL_UP_TIME_SEC;
}

float AC_DroneShowManager::get_time_until_takeoff_sec() const
{
    return get_time_until_start_sec() + get_relative_takeoff_time_sec();
}

bool AC_DroneShowManager::is_prepared_to_take_off() const
{
    return (!_preflight_check_failures && _is_gps_time_ok());
}

bool AC_DroneShowManager::notify_takeoff_attempt()
{
    if (!is_prepared_to_take_off())
    {
        return false;
    }
    
    if (!_copy_show_coordinate_system_from_parameters_to(_show_coordinate_system))
    {
        return false;
    }

    // If the trajectory is circular (i.e. drone is supposed to land where it
    // took off from), tweak the end of the trajectory to account for placement
    // inaccuracies (we want to land where we took off from, not where we
    // _should_ have taken off from in a perfect world).
    //
    // This correction is nice to have but is not crucial. If an error happens
    // in the process below, we just bail out and proceed without the correction.
    if (_trajectory_is_circular && !_trajectory_modified_for_landing)
    {
        Location takeoff_location;
        sb_vector3_with_yaw_t end;

        if (!get_current_location(takeoff_location))
        {
            goto exit;
        }

        _show_coordinate_system.convert_global_to_show_coordinate(takeoff_location, end);

        /*
        sb_vector3_with_yaw_t desired_takeoff_location_in_show_coordinates;
        if (sb_trajectory_player_get_position_at(_trajectory_player, 0.0f, &desired_takeoff_location_in_show_coordinates) != SB_SUCCESS)
        {
            goto exit;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "GPS coord: (%d, %d)",
            takeoff_location.lat, takeoff_location.lng);
        gcs().send_text(MAV_SEVERITY_WARNING, "SCS: (%d, %d) %.2f",
            _show_coordinate_system.origin_lat, _show_coordinate_system.origin_lng, _show_coordinate_system.orientation_rad * 180.0f / M_PI);
        gcs().send_text(MAV_SEVERITY_WARNING, "Desired: (%.2f, %.2f)",
            desired_takeoff_location_in_show_coordinates.x, desired_takeoff_location_in_show_coordinates.y);
        gcs().send_text(MAV_SEVERITY_WARNING, "Actual: (%.2f, %.2f)", end.x, end.y);
        gcs().send_text(MAV_SEVERITY_WARNING, "Diff: (%.2f, %.2f) --> %.2fm",
            end.x - desired_takeoff_location_in_show_coordinates.x,
            end.y - desired_takeoff_location_in_show_coordinates.y,
            hypotf(end.x - desired_takeoff_location_in_show_coordinates.x,
            end.y - desired_takeoff_location_in_show_coordinates.y) / 1000.0f
        );
        */

        if (sb_trajectory_replace_end_to_land_at(_trajectory, &_landing_time_sec, end)) {
            goto exit;
        }

        _trajectory_modified_for_landing = true;
    }

exit:
    return true;
}

bool AC_DroneShowManager::_is_at_takeoff_position_xy(float xy_threshold) const
{
    Location takeoff_loc;
    
    if (!_tentative_show_coordinate_system.is_valid())
    {
        // User did not set up the takeoff position yet
        return false;
    }

    if (!get_global_takeoff_position(takeoff_loc))
    {
        // Show coordinate system not set up yet
        return false;
    }

    return _is_close_to_position(
        takeoff_loc,
        xy_threshold > 0 ? xy_threshold : _params.max_xy_placement_error_m,
        0
    );
}
