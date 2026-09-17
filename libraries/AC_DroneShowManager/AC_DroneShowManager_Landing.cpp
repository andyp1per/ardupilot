#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

PostAction AC_DroneShowManager::get_action_at_end_of_show() const
{
    switch (_params.post_action) {
        case PostAction_Land:
            return PostAction_Land;

        case PostAction_Loiter:
            return PostAction_Loiter;

        case PostAction_RTL:
            return PostAction_RTL;

        case PostAction_RTLOrLand:
            return (
                _is_at_takeoff_position_xy(2 * DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS) &&
                _trajectory_is_circular
            ) ? PostAction_RTL : PostAction_Land;

        default:
            // Legacy behaviour when we did not have a parameter for the
            // post-show action
            return PostAction_Land;
    }
}

float AC_DroneShowManager::get_landing_speed_m_sec() const {
    float value = 0.0f;

    if (AP_Param::get("LAND_SPEED", value)) {
        if (value >= 0.0f && isfinite(value)) {
            return value / 100.0f; // Convert from cm/s to m/s
        }
    }

    return DEFAULT_LANDING_SPEED_METERS_PER_SEC;
}

float AC_DroneShowManager::propose_landing_handover_altitude_mm() const {
    float takeoff_altitude_mm = get_takeoff_altitude_mm();
    float proposal = get_landing_altitude_mm();

    if (proposal >= takeoff_altitude_mm) {
        // fallback
        proposal = takeoff_altitude_mm * 0.5f;
        if (proposal > 1000.0f) {
            proposal = 1000.0f;
        }
    }

    // sanity check
    return proposal < 0 ? 0 : proposal;
}

float AC_DroneShowManager::get_landing_hold_altitude_m() const
{
    float value = _params.landing_hold_altitude_m;
    return (value > 0 && isfinite(value)) ? value : 0.0f;
}

float AC_DroneShowManager::get_landing_hold_xy_error_m() const
{
    float value = _params.landing_hold_xy_error_m;
    return (value > 0 && isfinite(value)) ? value : DEFAULT_LANDING_HOLD_XY_ERROR_METERS;
}

float AC_DroneShowManager::get_landing_hold_timeout_sec() const
{
    float value = _params.landing_hold_timeout_sec;
    return (value >= 0 && isfinite(value)) ? value : DEFAULT_LANDING_HOLD_TIMEOUT_SEC;
}

float AC_DroneShowManager::get_landing_hold_min_time_sec() const
{
    float value = _params.landing_hold_min_time_sec;
    return (value >= 0 && isfinite(value)) ? value : DEFAULT_LANDING_HOLD_MIN_TIME_SEC;
}

float AC_DroneShowManager::get_landing_hold_abort_error_m() const
{
    float value = _params.landing_hold_abort_error_m;
    return (value > 0 && isfinite(value)) ? value : 0.0f;
}

float AC_DroneShowManager::get_landing_total_time_sec() const
{
    float value = _params.landing_total_time_sec;
    return (value > 0 && isfinite(value)) ? value : 0.0f;
}

float AC_DroneShowManager::get_landing_hold_descent_speed_m_sec() const
{
    float value = _params.landing_hold_descent_speed_m_sec;
    return (value > 0 && isfinite(value)) ? value : 0.0f;
}
