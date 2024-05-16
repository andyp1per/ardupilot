#pragma once

#include <hal.h>
#include "AP_HAL_ChibiOS.h"
#if CH_CFG_USE_CONDVARS == TRUE
#include <AP_HAL/ConcurrentObjectBuffer.h>

template <class T>
class ChibiOS::ConcurrentObjectBuffer : public AP_HAL::ConcurrentObjectBuffer<T> {
public:
    ConcurrentObjectBuffer(uint32_t _size = 0) : AP_HAL::ConcurrentObjectBuffer<T>(_size) {
        chMtxObjectInit(&_mutex);
        chCondObjectInit(&_cond);
    }
    ~ConcurrentObjectBuffer(void) {
    }

    void lock() override {
        chMtxLock(&_mutex);
    }
    void unlock() override {
        chMtxUnlock(&_mutex);
    }
    void wait() override {
        chCondWait(&_cond);
    }
    void signal() override {
        chCondSignal(&_cond);
    }

    mutex_t _mutex;
    condition_variable_t _cond;
};
#endif
