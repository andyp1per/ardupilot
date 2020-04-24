
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SITL_RC_INPUT_CHANNELS 16
#include <atomic>

#include "AP_HAL_SITL.h"

class HALSITL::RCInput : public AP_HAL::RCInput {
public:
    explicit RCInput(SITL_State *sitlState): _sitlState(sitlState) {}
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    void set_num_channels(uint8_t num);
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    const char *protocol() const override;
    void _run_periodic();

private:
    HAL_Semaphore _sem;

    void _start_input_thread(void);
    void _timer_tick();

    std::atomic<unsigned int> rc_input_count;
    std::atomic<unsigned int> last_rc_input_count;

    uint16_t _pwm_values[SITL_RC_INPUT_CHANNELS];
    std::atomic<uint8_t>  _num_channels;
    bool _thread_started = false;
    bool _thread_should_exit = false;

    SITL_State *_sitlState;
    bool using_rc_protocol;
};

#endif

