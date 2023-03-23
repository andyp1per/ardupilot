#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

void init();

void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;

uint16_t micros16();
uint32_t micros();
uint32_t millis();
uint16_t millis16();
uint64_t micros64();
uint64_t millis64();

uint32_t native_micros();
uint32_t native_millis();
uint16_t native_millis16();
uint64_t native_micros64();
uint64_t native_millis64();

void dump_stack_trace();
void dump_core_file();

template <typename T>
inline bool timeout_expired(const T past_time, const T now, const T timeout)
{
    const T dt = now - past_time;
    return (dt > timeout);
}

template <typename T>
inline T timeout_remaining(const T past_time, const T now, const T timeout)
{
    const T dt = now - past_time;
    return (dt > timeout) ? T(0) : (timeout - dt);
}

inline bool timeout_expired_us(uint32_t past_time_us, uint32_t timeout_us)
{
    return timeout_expired(past_time_us, micros(), timeout_us);
}

inline uint32_t timeout_remaining_us(uint32_t past_time_us, uint32_t timeout_us)
{
    return timeout_remaining(past_time_us, micros(), timeout_us);
}

inline bool timeout_expired_us(uint64_t past_time_us, uint64_t timeout_us)
{
    return timeout_expired(past_time_us, micros64(), timeout_us);
}

inline uint32_t timeout_remaining_us(uint64_t past_time_us, uint64_t timeout_us)
{
    return timeout_remaining(past_time_us, micros64(), timeout_us);
}

inline bool timeout_expired_ms(uint32_t past_time_ms, uint32_t timeout_ms)
{
    return timeout_expired(past_time_ms, millis(), timeout_ms);
}

inline uint32_t timeout_remaining_ms(uint32_t past_time_ms, uint32_t timeout_ms)
{
    return timeout_remaining(past_time_ms, millis(), timeout_ms);
}

} // namespace AP_HAL
