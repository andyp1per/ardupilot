#include "Copter.h"

#if MODE_CRUISE_ENABLED

/*
 * Init and run calls for cruise flight mode
 */

bool ModeCruise::init(bool ignore_checks)
{
    if (g2.cruise_speed_max_ms < g2.cruise_speed_ms) {
        g2.cruise_speed_max_ms.set(g2.cruise_speed_ms);
    }
    return true;
}

bool ModeCruise::requires_GPS() const
{
    // requires GPS when not cruising
    return !ahrs.get_fly_forward();
}

void ModeCruise::run()
{
    bool need_coord = !loiter_nav->loiter_option_is_set(AC_Loiter::LoiterOption::COORDINATED_TURN_ENABLED);

    if (need_coord) {
        loiter_nav->set_loiter_option(AC_Loiter::LoiterOption::COORDINATED_TURN_ENABLED);
    }
    // run angle controller
    update_fly_forward();
    ModeLoiter::run();

    if (need_coord) {
        loiter_nav->reset_loiter_option(AC_Loiter::LoiterOption::COORDINATED_TURN_ENABLED);
    }

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_airspeed_calibration_ms > 1000) {
        last_airspeed_calibration_ms = now_ms;
        airspeed_ratio_update();

        // use average of min and max airspeed as default airspeed fusion with high variance
        ahrs.writeDefaultAirSpeed((float)((g2.cruise_speed_min_ms + g2.cruise_speed_max_ms)/2),
                                  (float)((g2.cruise_speed_max_ms - g2.cruise_speed_min_ms)/2));
    }
}

void ModeCruise::update_fly_forward(void)
{
    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(aspeed);

    // first check if cruising has to be aborted
    if (!ahrs.using_airspeed_sensor() || !have_airspeed || aspeed < g2.cruise_speed_min_ms || aspeed > g2.cruise_speed_max_ms) {
        if (ahrs.get_fly_forward()) {
            gcs().send_text(MAV_SEVERITY_INFO,"Cruising disabled at %fm/s", aspeed);
            ahrs.set_fly_forward(false);
        }
        return;
    }

    // start estimating wind
    if (aspeed >= g2.cruise_speed_min_ms && !ahrs.get_wind_estimation_enabled()) {
        //ahrs.set_wind_estimation_enabled(true);
    }

    // can we start cruising
    if (aspeed >= g2.cruise_speed_ms && !ahrs.get_fly_forward()) {
        gcs().send_text(MAV_SEVERITY_INFO,"Cruising enabled at %fm/s", aspeed);
        ahrs.set_fly_forward(true);
        return;
    }
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/*
  once a second update the airspeed calibration ratio
 */
void ModeCruise::airspeed_ratio_update(void)
{
    if (!hal.util->get_soft_armed() ||
        !ahrs.get_fly_forward() ||
        !copter.get_likely_flying() ||
        !AP::airspeed()->enabled() ||
        AP::gps().status() < AP_GPS::GPS_OK_FIX_3D ||
        AP::gps().ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (AP::airspeed()->get_airspeed() < g2.cruise_speed_ms && 
        AP::gps().ground_speed() < (uint32_t)g2.cruise_speed_ms.get()) {
        // don't calibrate when flying below the cruising speed airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    const Vector3f &vg = AP::gps().velocity();
    AP::airspeed()->update_calibration(vg, g2.cruise_speed_max_ms.get());
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

#endif
