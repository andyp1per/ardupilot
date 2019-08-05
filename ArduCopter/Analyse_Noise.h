/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_Math/AP_Math.h"
#include "AP_InertialSensor/AP_InertialSensor.h"
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#include <arm_math.h>
#endif

// max for F3 targets
#define FFT_WINDOW_SIZE 64
#define XYZ_AXIS_COUNT 3

class Analyse_Noise
{
public:
    Analyse_Noise();

    void init(uint32_t target_looptime, AP_InertialSensor& ins);
    void push_sample(const Vector3f& sample);
    void analyse();
    void analyse_update();
    void analyse_init(uint32_t target_looptime_us);
    Vector3f get_noise_center_freq_hz() const { return _center_freq_hz; }
    uint8_t get_arm_max_bin() const { return _arm_max_bin; }
    uint8_t get_max_bin() const { return _max_bin; }
    float get_max_bin_energy() const { return _max_bin_energy; }

    // a function called by the main thread at the main loop rate:
    void sample_gyros();

    static const struct AP_Param::GroupInfo var_info[];

private:
    float calculate_weighted_center_freq(uint8_t bin_start, uint8_t bin_max);
    float calculate_simple_center_freq(uint8_t bin_start, uint8_t bin_max);
    float calculate_quinns_second_estimator_center_freq(uint8_t bin_max);
    float tau(float x);
    // downsampled gyro data circular buffer for frequency analysis
    uint8_t _circular_buffer_idx;
    float _downsampled_gyro_data[XYZ_AXIS_COUNT][FFT_WINDOW_SIZE];

    // update state machine step information
    uint8_t _update_ticks;
    uint8_t _update_step;
    uint8_t _update_axis;
    uint8_t _arm_max_bin;
    uint8_t _max_bin;
    float _max_bin_energy;
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_instance_f32 _fft_instance;
#endif
    float _fft_data[FFT_WINDOW_SIZE];
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    float _rfft_data[FFT_WINDOW_SIZE];
#endif

    Vector3f _center_freq_hz;
    Vector3f _prev_center_freq_hz;
    LowPassFilter2pFloat _detected_frequency_filter[XYZ_AXIS_COUNT];

    AP_Int16 _fft_sampling_rate_hz;
    float _fft_resolution;
    uint8_t _fft_start_bin;
    uint16_t _dyn_notch_max_ctr_hz;
    AP_Int16 _dyn_notch_min_hz;
    AP_Int8 _enable;
    AP_InertialSensor* _ins;

    // Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
    float _hanning_window[FFT_WINDOW_SIZE];
};
