#pragma once

/// @file	AC_HELI_PID.h
/// @brief	Helicopter Specific Rate PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>
#include <Filter/NotchFilter.h>
#include <AP_Logger/AP_Logger.h>
#include "AC_PID.h"

#define AC_PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_FILT_HZ_MIN      0.01f   // minimum input filter frequency

/// @class	AC_PID_Fiteredd
/// @brief	PID control class with D-term notch filter
class AC_PID_Filtered : public AC_PID {
public:

    /// Constructor for PID
    AC_PID_Filtered(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff=0);

   // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input_filter_all - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_all(float input);

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(float input);

    // set accessors
    void        filt_hz(const float v);

    float       get_derivative() const { return _derivative; }
    float       get_raw_derivative() const { return _raw_derivative; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

    AP_Int8     enable;
    AP_Float    center_freq_hz;
    AP_Float    bandwidth_hz;
    AP_Float    attenuation_dB;

protected:
    // internal variables
    float           _raw_input;             // last filtered output value

private:
    LowPassFilterFloat _pid_filter;
    NotchFilterFloat _pid_notch_filter;
};
