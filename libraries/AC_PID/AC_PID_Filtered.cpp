/// @file	AC_PID_Filtered_Filtered.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_Filtered.h"

const AP_Param::GroupInfo AC_PID_Filtered::var_info[] = {
    // parameters from parent PID
    AP_NESTEDGROUPINFO(AC_PID, 0),

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("NTCH", 1, AC_PID_Filtered, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: Frequency
    // @Description: Notch center frequency in Hz
    // @Range: 10 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTHZ", 2, AC_PID_Filtered, center_freq_hz, 80),

    // @Param: BW
    // @DisplayName: Bandwidth
    // @Description: Notch bandwidth in Hz
    // @Range: 5 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTBW", 3, AC_PID_Filtered, bandwidth_hz, 20),

    // @Param: ATT
    // @DisplayName: Attenuation
    // @Description: Notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("NTAT", 4, AC_PID_Filtered, attenuation_dB, 15),
    
    AP_GROUPEND
};

// Constructor
AC_PID_Filtered::AC_PID_Filtered(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff) :
    AC_PID(initial_p, initial_i, initial_d, initial_imax, initial_filt_hz, dt, initial_ff),
    _raw_input(0.0f),
    _raw_derivative(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    _pid_notch_filter.init(1.0/dt, center_freq_hz, bandwidth_hz, attenuation_dB);
}

// set_dt - set time step in seconds
void AC_PID_Filtered::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    AC_PID::set_dt(dt);
    _pid_filter.set_cutoff_frequency(1.0/dt, _filt_hz);
    _pid_notch_filter.init(1.0/dt, center_freq_hz, bandwidth_hz, attenuation_dB);
}

// filt_hz - set input filter hz
void AC_PID_Filtered::filt_hz(float hz)
{
    AC_PID::filt_hz(hz);
    _pid_filter.set_cutoff_frequency(1.0/_dt, _filt_hz);
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_Filtered::set_input_filter_all(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _pid_filter.reset();
        _flags._reset_filter = false;
        _input = input;
        _raw_input = input;
        _derivative = 0.0f;
        _raw_derivative = 0.0f;        
    }

    // update filter and calculate derivative
    const float next_input = _pid_filter.apply(input);
    if (_dt > 0.0f) {
        _derivative = (next_input - _input) / _dt;
        _raw_derivative = (input - _raw_input) / _dt;        
    }
    _input = next_input;
    _raw_input = input;
}

// set_input_filter_d - set input to PID controller
// only input to the D portion of the controller is filtered
// this should be called before any other calls to get_p, get_i or get_d
void AC_PID_Filtered::set_input_filter_d(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
        _derivative = 0.0f;
        _raw_input = 0.0f;
        _raw_derivative = 0.0f;        
    }

    // update filter and calculate derivative
    if (_dt > 0.0f) {
        _raw_derivative = (input - _input) / _dt;
        _derivative = _pid_filter.apply(_raw_derivative);
    }

    _raw_input = input;
    _input = input;
}



