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
#define FFT_WINDOW_SIZE 32
#define XYZ_AXIS_COUNT 3
#define TICK_CYCLE 10

class Analyse_Noise
{
public:
    Analyse_Noise();

    void init(uint32_t target_looptime, AP_InertialSensor& ins);
    void analyse();
    void analyse_update(const GyroWindow& window, uint8_t buffer_index);
    void analyse_init();
    Vector3f get_noise_center_freq_hz() const { return _center_freq_hz_filtered; }
    Vector3f get_center_freq_energy() const { return _center_freq_energy; }
    Vector3<uint8_t> get_center_freq_bin() const { return _center_freq_bin; }

    // a function called by the main thread at the main loop rate:
    void sample_gyros();
    bool calibration_check();

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum {
        STEP_HANNING,
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_COUNT
    };

    float calculate_simple_center_freq(uint8_t bin_max);
    float calculate_jains_estimator_center_freq(uint8_t k);
    float calculate_quinns_second_estimator_center_freq(uint8_t bin_max);
    float tau(float x);
    float self_test_bin_frequencies();
    float self_test(float frequency);

    // downsampled gyro data circular buffer for frequency analysis
    uint16_t _circular_buffer_idx;
    uint8_t _sample_count;
    float _downsampled_gyro_data[XYZ_AXIS_COUNT][FFT_WINDOW_SIZE];
    Vector3f _oversampled_gyro_accum;

    // update state machine step information
    uint8_t _update_ticks;
    uint8_t _update_step = STEP_HANNING;
    uint8_t _update_axis;
    uint32_t _output_count;
    Vector3f _center_freq_energy;
    Vector3f _ref_energy;
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_instance_f32 _fft_instance;
#endif
    float _fft_data[FFT_WINDOW_SIZE];
    float _rfft_data[FFT_WINDOW_SIZE];

    Vector3f _center_freq_hz_filtered;
    Vector3f _center_freq_hz;
    float _multiplier;
    LowPassFilter2pFloat _detected_frequency_filter[XYZ_AXIS_COUNT];

    //AP_Int16 _fft_sampling_rate_hz;
    uint16_t _fft_sampling_rate_hz;
    float _fft_resolution;
    uint8_t _fft_start_bin;
    Vector3<uint8_t> _center_freq_bin;
    AP_Int16 _fft_min_hz;
    AP_Int16 _fft_max_hz;
    AP_Int8 _enable;
    AP_Int8 _sample_mode;
    AP_InertialSensor* _ins;

    // Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
    float _hanning_window[FFT_WINDOW_SIZE];

    class StepTimer {
    public:
        uint32_t _timer_total;
        uint32_t _timer_avg;
        uint8_t _time_ticks;

        void time(uint32_t start) {
            _timer_total += (AP_HAL::micros() - start);
            _time_ticks = (_time_ticks + 1) % TICK_CYCLE;
            if (_time_ticks == 0) {
                _timer_avg = _timer_total / TICK_CYCLE;
                _timer_total = 0;
            }
        }
    };

    StepTimer _arm_cfft_f32_timer;
    StepTimer _bitreversal_timer;
    StepTimer _stage_rfft_f32_timer;
    StepTimer _calc_freq_timer;
};
