/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Code by Andy Piper
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#define XYZ_AXIS_COUNT 3

class AP_GyroFFT
{
public:
    AP_GyroFFT();

    // Do not allow copies
    AP_GyroFFT(const AP_GyroFFT &other) = delete;
    AP_GyroFFT &operator=(const AP_GyroFFT&) = delete;

    void init(uint32_t target_looptime, AP_InertialSensor& ins);

    // cycle through the FFT steps
    void update();

    // get the detected noise frequency filtered at 1/3 the update rate
    Vector3f get_noise_center_freq_hz() const { return _center_freq_hz_filtered; }
    Vector3f get_noise_ref_energy() const { return _ref_energy; }
    float get_weighted_noise_center_freq_hz() const;
    Vector3f get_raw_noise_center_freq_hz() const { return _center_freq_hz; }
    Vector3f get_center_freq_energy() const { return _center_freq_energy; }
    Vector3<uint8_t> get_center_freq_bin() const { return _center_freq_bin; }

    // a function called by the main thread at the main loop rate:
    void sample_gyros();
    bool calibration_check();
    AP_Int8 _track_mode;

    static const struct AP_Param::GroupInfo var_info[];
    static AP_GyroFFT *get_singleton() { return _singleton; }

private:
    void calculate_noise(uint16_t bin_max);
    void update_ref_energy();
    //  interpolate between frequency bins using various methods
    float calculate_simple_center_freq(uint8_t bin_max);
    float calculate_jains_estimator_center_freq(uint8_t k);
    float tau(float x);
    float self_test_bin_frequencies();
    float self_test(float frequency);

    AP_HAL::DSP::FFTWindowState* _state;

    // downsampled gyro data circular buffer for frequency analysis
    uint16_t _circular_buffer_idx;
    uint8_t _sample_count;
    // number of sampeles needed before a new frame can be processed
    uint16_t _samples_per_frame;
    float* _downsampled_gyro_data[XYZ_AXIS_COUNT];
    Vector3f _oversampled_gyro_accum;

    // update state machine step information
    uint8_t _update_axis;
    // the number of cycles required to have a proper noise reference
    uint8_t _noise_cycles;
    uint32_t _output_count;
    Vector3f _center_freq_energy;
    // Smoothing filter on the output
    LowPassFilter2pFloat _center_freq_filter[XYZ_AXIS_COUNT];
    // noise base of the gyros
    Vector3f _ref_energy;
    // detected noise frequency
    Vector3f _center_freq_hz;
    Vector3f _center_freq_hz_filtered;
    float _multiplier;

    uint16_t _fft_sampling_rate_hz;
    uint8_t _fft_start_bin;
    uint8_t _missed_cycles;
    uint8_t _noise_needs_calibration : 3;
    Vector3<uint8_t> _center_freq_bin;
    AP_Int16 _fft_min_hz;
    AP_Int16 _fft_max_hz;
    AP_Int16 _window_size;
    AP_Float _window_overlap;
    AP_Int8 _enable;
    AP_Int8 _sample_mode;
    AP_InertialSensor* _ins;

    static AP_GyroFFT *_singleton;
};

namespace AP {
    AP_GyroFFT *fft();
};
