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

bool AC_DroneShowManager::replace_end_of_trajectory_to_land_at(
    sb_trajectory_t* trajectory, 
    float* landing_time_sec,
    sb_vector3_with_yaw_t landing_position
) {
    sb_vector3_with_yaw_t pos_at_landing_time, vel_at_landing_time;
    sb_vector3_with_yaw_t c1, c2, zero;
    sb_trajectory_player_t player;
    sb_trajectory_builder_t builder;
    float duration_sec;

    // Calculate the position and velocity of the drone at the time when the
    // landing should start
    // TODO(ntamas): Is there a way to get this faster, where we calculate the
    // trajectory stats?
    if (sb_trajectory_player_init(&player, trajectory) != SB_SUCCESS)
    {
        return false;
    }
    if (sb_trajectory_player_get_position_at(&player, *landing_time_sec, &pos_at_landing_time))
    {
        return false;
    }
    if (sb_trajectory_player_get_velocity_at(&player, *landing_time_sec, &vel_at_landing_time))
    {
        return false;
    }
    sb_trajectory_player_destroy(&player);    
    
    /*
    gcs().send_text(MAV_SEVERITY_WARNING,
        "Pos and vel at landing: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)",
        pos_at_landing_time.x, pos_at_landing_time.y, pos_at_landing_time.z,
        vel_at_landing_time.x, vel_at_landing_time.y, vel_at_landing_time.z
    );
    */

    // TODO: query landing velocity from parameters
    const uint32_t landing_velocity_mm_sec = 500;
    duration_sec = pos_at_landing_time.z < 0 ? 0 : (pos_at_landing_time.z / static_cast<float>(landing_velocity_mm_sec));

    // Limit the landing duration to one minute because we are going to
    // append a single Bezier segment and the trajectory format has its
    // limits on the segment length
    if (duration_sec > 60) {
        duration_sec = 60;
    }
    
    // Calculate the cubic Bezier curve that will send the drone back to its
    // takeoff position from the point where it crosses the takeoff altitude
    // threshold from above
    zero.x = zero.y = zero.z = zero.yaw = 0;
    sb_get_cubic_bezier_from_velocity_constraints(
        /* start = */ pos_at_landing_time,
        /* start_vel = */ vel_at_landing_time,
        /* end = */ landing_position,
        /* end_vel = */ zero,
        /* duration_sec = */ duration_sec,
        &c1, &c2
    );

    // Ensure that we own the trajectory and we can modify it at will
    // (i.e. it is not a view into the already loaded show file)
    if (sb_buffer_ensure_owned(&trajectory->buffer) != SB_SUCCESS) {
        return false;
    }

    // Also ensure that we will have extra space at the end of the buffer
    // to add a final Bezier segment. 32 bytes will be enough.
    if (sb_buffer_extend_with_zeros(&trajectory->buffer, 32) != SB_SUCCESS) {
        return false;
    }

    // Shorten the trajectory so that it ends at the time when we cross
    // the takeoff altitude from above
    if (sb_trajectory_cut_at(trajectory, *landing_time_sec) != SB_SUCCESS) {
        return false;
    }

    // Initialize a trajectory builder so we can add the final segment
    if (sb_trajectory_builder_init_from_trajectory(&builder, trajectory, 0) != SB_SUCCESS) {
        return false;
    }

    /*
    gcs().send_text(MAV_SEVERITY_WARNING,
        "c1: (%.2f, %.2f, %.2f)",
        c1.x, c1.y, c1.z
    );
    gcs().send_text(MAV_SEVERITY_WARNING,
        "c2: (%.2f, %.2f, %.2f)",
        c2.x, c2.y, c2.z
    );
    gcs().send_text(MAV_SEVERITY_WARNING,
        "end: (%.2f, %.2f, %.2f)",
        landing_position.x, landing_position.y, landing_position.z
    );
    */

    // Ensure that we own the trajectory and we can modify it at will
    // (i.e. it is not a view into the already loaded show file)
    if (sb_buffer_ensure_owned(&trajectory->buffer) != SB_SUCCESS) {
        return false;
    }

    // Also ensure that we will have extra space at the end of the buffer
    // to add a final Bezier segment. 32 bytes will be enough.
    if (sb_buffer_extend_with_zeros(&trajectory->buffer, 32) != SB_SUCCESS) {
        return false;
    }

    // Shorten the trajectory so that it ends at the time when we cross
    // the takeoff altitude from above
    if (sb_trajectory_cut_at(trajectory, *landing_time_sec) != SB_SUCCESS) {
        return false;
    }

    // Initialize a trajectory builder so we can add the final segment
    if (sb_trajectory_builder_init_from_trajectory(&builder, trajectory, 0) != SB_SUCCESS) {
        return false;
    }

    /*
    gcs().send_text(MAV_SEVERITY_WARNING,
        "c1: (%.2f, %.2f, %.2f)",
        c1.x, c1.y, c1.z
    );
    gcs().send_text(MAV_SEVERITY_WARNING,
        "c2: (%.2f, %.2f, %.2f)",
        c2.x, c2.y, c2.z
    );
    gcs().send_text(MAV_SEVERITY_WARNING,
        "end: (%.2f, %.2f, %.2f)",
        landing_position.x, landing_position.y, landing_position.z
    );
    */

    // Add the final segment
    /*
    gcs().send_text(MAV_SEVERITY_WARNING, "before: %.2f sec", sb_trajectory_get_total_duration_sec(trajectory));
    */
    if (sb_trajectory_builder_append_cubic_bezier(
        &builder, c1, c2, landing_position,
        static_cast<uint32_t>(duration_sec * 1000.0f) /* [s] --> [ms] */
    )) {
        sb_trajectory_builder_destroy(&builder);
        return false;
    }
    /*
    gcs().send_text(MAV_SEVERITY_WARNING, "after: %.2f sec", sb_trajectory_get_total_duration_sec(trajectory));
    */

    // Update the size of the trajectory buffer
    trajectory->buffer.end = builder.buffer.end;
    sb_trajectory_builder_destroy(&builder);
    
    *landing_time_sec += duration_sec;
    
    return true;
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

        if (replace_end_of_trajectory_to_land_at(_trajectory, &_landing_time_sec, end)) {
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
