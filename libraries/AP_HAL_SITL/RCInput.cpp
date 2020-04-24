#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCInput.h"
#include "Scheduler.h"
#include <SITL/SITL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

using namespace HALSITL;

// thread update at 100Hz
#define RC_INPUT_PERIOD_USEC 10000

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
    AP::RC().init();
    _start_input_thread();
}

bool RCInput::new_input()
{
    WITH_SEMAPHORE(_sem);

    bool ret = rc_input_count != last_rc_input_count;
    if (ret) {
        last_rc_input_count.store(rc_input_count);
        using_rc_protocol = true;
    }

    if (!using_rc_protocol && _sitlState->new_rc_input) {
        _sitlState->new_rc_input = false;
        return true;
    }
    return ret;
}

void RCInput::set_num_channels(uint8_t num)
{
    _num_channels = num;
}

const char *RCInput::protocol() const {
    if (using_rc_protocol) {
        return AP::RC().protocol_name();
    }

    return "SITL";
}

uint16_t RCInput::read(uint8_t ch)
{
    if (ch >= num_channels()) {
        return 0;
    }
    if (using_rc_protocol) {
        WITH_SEMAPHORE(_sem);

        return _pwm_values[ch];
    }
    return _sitlState->pwm_input[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > num_channels()) {
        len = num_channels();
    }
    for (uint8_t i=0; i < len; i++) {
        periods[i] = read(i);
    }
    return len;
}

uint8_t RCInput::num_channels()
{
    if (using_rc_protocol) {
        return _num_channels;
    }
    SITL::SITL *_sitl = AP::sitl();
    if (_sitl) {
        return MIN(_sitl->rc_chancount.get(), SITL_RC_INPUT_CHANNELS);
    }
    return SITL_RC_INPUT_CHANNELS;
}

void RCInput::_run_periodic()
{
    uint64_t next_run_usec = AP_HAL::micros64() + RC_INPUT_PERIOD_USEC;

    while (!_thread_should_exit) {
        uint64_t dt = next_run_usec - AP_HAL::micros64();
        if (dt > RC_INPUT_PERIOD_USEC) {
            // we've lost sync - restart
            next_run_usec = AP_HAL::micros64();
        } else {
            Scheduler::from(hal.scheduler)->delay_microseconds(dt);
        }
        next_run_usec += RC_INPUT_PERIOD_USEC;

        _timer_tick();
    }

    _thread_started = false;
    _thread_should_exit = false;
}

// start the input thread
void RCInput::_start_input_thread(void)
{
    WITH_SEMAPHORE(_sem);

    if (_thread_started) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&HALSITL::RCInput::_run_periodic, void), "apm_rcin", 512, AP_HAL::Scheduler::PRIORITY_RCIN, 1)) {
        AP_HAL::panic("Failed to start RCInput update thread");
    }

    _thread_started = true;
}

// update pwm_values from RCProtocol, potentially in another thread
void RCInput::_timer_tick(void)
{
    if (AP::RC().new_input()) {
        WITH_SEMAPHORE(_sem);

        uint8_t n = AP::RC().num_channels();
        for (uint8_t i=0; i<n; i++) {
            _pwm_values[i] = AP::RC().read(i);
        }
        set_num_channels(n);
        rc_input_count++;
    }
}

#endif
