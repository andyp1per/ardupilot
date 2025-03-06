#include "Rover.h"

#if AP_FENCE_ENABLED

// async fence checking io callback at 1Khz
void Rover::fence_run_checks()
{
    uint32_t now = AP_HAL::millis();

    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 40U)) { // 25Hz update rate
        return;
    }

    if (fence_breaches.have_new_breaches) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    if (new_breaches) {
        fence_breaches.new_breaches = new_breaches;
        fence_breaches.have_new_breaches = true; // new breaches latched so main loop will now pick it up
    } else if (orig_breaches && fence.get_breaches() == 0) {
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

// fence_check - ask fence library to check for breaches and initiate the response
void Rover::fence_check()
{
    // only take action if there is a new breach
    if (!fence_breaches.have_new_breaches) {
        return;
    }

    // return immediately if motors are not armed
    if (!arming.is_armed()) {
        fence_breaches.have_new_breaches = false;
        return;
    }

    // if the user wants some kind of response and motors are armed
    if ((FailsafeAction)fence.get_action() != FailsafeAction::None) {
        // if within 100m of the fence, it will take the action specified by the FENCE_ACTION parameter
        if (fence.get_breach_distance(fence_breaches.new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
            switch ((FailsafeAction)fence.get_action()) {
            case FailsafeAction::None:
                break;
            case FailsafeAction::SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::RTL:
                if (set_mode(mode_rtl, ModeReason::FENCE_BREACHED)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::Hold:
                set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                break;
            case FailsafeAction::SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                    set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                }
                break;
            case FailsafeAction::Terminate:
                arming.disarm(AP_Arming::Method::FENCEBREACH);
                break;
            }
        } else {
            // if more than 100m outside the fence just force to HOLD
            set_mode(mode_hold, ModeReason::FENCE_BREACHED);
        }
    }
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    fence_breaches.have_new_breaches = false;
}

#endif // AP_FENCE_ENABLED
