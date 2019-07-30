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

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 */
#include <stdint.h>
#include "Analyse_Noise.h"

// table of user settable parameters
const AP_Param::GroupInfo Analyse_Noise::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, Analyse_Noise, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: Sampling Frequency
    // @Description: Sampling Frequency in Hz
    // @Range: 10 2000
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SAMPLE_FREQ", 2, Analyse_Noise, _fft_sampling_rate_hz, 1000),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Lower bound of notch center frequency in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 3, Analyse_Noise, _dyn_notch_min_hz, 80),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// Eg [0,31), [31,62), [62, 93) etc
// for gyro loop >= 4KHz, sample rate 2000 defines FFT range to 1000Hz, 16 bins each 62.5 Hz wide
// NB  FFT_WINDOW_SIZE is set to 32 in gyroanalyse.h
#define FFT_BIN_COUNT (FFT_WINDOW_SIZE / 2)
// smoothing frequency for FFT centre frequency
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS (XYZ_AXIS_COUNT * 4)

Analyse_Noise::Analyse_Noise()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Analyse_Noise::init(uint32_t target_looptime_us, AP_InertialSensor& ins)
{
    _ins = &ins;
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    const int gyro_loop_rate_hz = lrintf((1.0f / target_looptime_us) * 1e6f);

    _fft_sampling_rate_hz = MIN(gyro_loop_rate_hz, _fft_sampling_rate_hz);
    _fft_resolution = (float)_fft_sampling_rate_hz / FFT_WINDOW_SIZE;
    _fft_start_bin = _dyn_notch_min_hz / lrintf(_fft_resolution);
    _dyn_notch_max_ctr_hz = _fft_sampling_rate_hz / 2; //Nyquist

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2 * M_PI * i / (FFT_WINDOW_SIZE - 1)));
    }

    analyse_init(target_looptime_us);
}

// a function called by the main thread at the main loop rate:
void Analyse_Noise::sample_gyros()
{
    if (!_enable) {
        return;
    }
    push_sample(_ins->get_filtered_gyro());
}

void Analyse_Noise::analyse_init(uint32_t target_looptime_us)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_init_f32(&_fft_instance, FFT_WINDOW_SIZE);
#endif
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        _center_freq_hz[axis] = _dyn_notch_max_ctr_hz;
        _prev_center_freq_hz[axis] = _dyn_notch_max_ctr_hz;
        _detected_frequency_filter[axis].set_cutoff_frequency(_fft_sampling_rate_hz, DYN_NOTCH_SMOOTH_FREQ_HZ);
    }
}

void Analyse_Noise::push_sample(const Vector3f& sample)
{
    //  fast sampling means that the raw gyro values have already been averaged over 8 samples
    _downsampled_gyro_data[0][_circular_buffer_idx] = sample.x;
    _downsampled_gyro_data[1][_circular_buffer_idx] = sample.y;
    _downsampled_gyro_data[2][_circular_buffer_idx] = sample.z;
    _circular_buffer_idx = (_circular_buffer_idx + 1) % FFT_WINDOW_SIZE;

    // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
    _update_ticks = DYN_NOTCH_CALC_TICKS;
}

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void Analyse_Noise::analyse()
{
    if (!_enable) {
        return;
    }

    // calculate FFT and update filters
    if (_update_ticks > 0) {
        analyse_update();
        --_update_ticks;
    }
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
    void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
    void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);
}
#endif

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
void Analyse_Noise::analyse_update()
{
    enum {
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_HANNING,
        STEP_COUNT
    };
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_cfft_instance_f32 *Sint = &(_fft_instance.Sint);
#endif

    switch (_update_step) {
    case STEP_ARM_CFFT_F32: {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        switch (FFT_BIN_COUNT) {
        case 16:
            // 16us
            arm_cfft_radix8by2_f32(Sint, _fft_data);
            break;
        case 32:
            // 35us
            arm_cfft_radix8by4_f32(Sint, _fft_data);
            break;
        case 64:
            // 70us
            arm_radix8_butterfly_f32(_fft_data, FFT_BIN_COUNT, Sint->pTwiddle, 1);
            break;
        }
#endif
        break;
    }
    case STEP_BITREVERSAL: {
        // 6us
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        arm_bitreversal_32((uint32_t *)_fft_data, Sint->bitRevLength, Sint->pBitRevTable);
#endif
        _update_step++;
        FALLTHROUGH;
    }
    case STEP_STAGE_RFFT_F32: {
        // 14us
        // this does not work in place => _fft_data AND _rfft_data needed
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        stage_rfft_f32(&_fft_instance, _fft_data, _rfft_data);
#endif
        break;
    }
    case STEP_ARM_CMPLX_MAG_F32: {
        // 8us
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        arm_cmplx_mag_f32(_rfft_data, _fft_data, FFT_BIN_COUNT);
#endif
        _update_step++;
        FALLTHROUGH;
    }
    case STEP_CALC_FREQUENCIES: {
        bool fft_increased = false;
        float data_max = 0;
        uint8_t bin_start = 0;
        uint8_t bin_max = 0;
        //for bins after initial decline, identify start bin and max bin
        for (uint8_t i = _fft_start_bin; i < FFT_BIN_COUNT; i++) {
            if (fft_increased || (_fft_data[i] > _fft_data[i - 1])) {
                if (!fft_increased) {
                    bin_start = i; // first up-step bin
                    fft_increased = true;
                }
                if (_fft_data[i] > data_max) {
                    data_max = _fft_data[i];
                    bin_max = i; // tallest bin
                }
            }
        }
        float weighted_center_freq_hz = calculate_weighted_center_freq(bin_start, bin_max);

        if (weighted_center_freq_hz <=0) {
            weighted_center_freq_hz = _prev_center_freq_hz[_update_axis];
        }
        weighted_center_freq_hz = MAX(weighted_center_freq_hz, (float)_dyn_notch_min_hz);
        weighted_center_freq_hz = _detected_frequency_filter[_update_axis].apply(weighted_center_freq_hz);
        _prev_center_freq_hz[_update_axis] = _center_freq_hz[_update_axis];
        _center_freq_hz[_update_axis] = weighted_center_freq_hz;

        break;
    }
    case STEP_UPDATE_FILTERS: {
        // 7us
        // calculate cutoffFreq and notch Q, update notch filter  =1.8+((A2-150)*0.004)
        if (!is_equal(_prev_center_freq_hz[_update_axis], _center_freq_hz[_update_axis])) {
            //
        }

        _update_axis = (_update_axis + 1) % XYZ_AXIS_COUNT;
        _update_step++;
        FALLTHROUGH;
    }
    case STEP_HANNING: {
        // 5us
        // apply hanning window to gyro samples and store result in _fft_data
        // hanning starts and ends with 0, could be skipped for minor speed improvement
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        const uint8_t ringBufIdx = FFT_WINDOW_SIZE - _circular_buffer_idx;
        arm_mult_f32(&_downsampled_gyro_data[_update_axis][_circular_buffer_idx], &_hanning_window[0], &_fft_data[0], ringBufIdx);
        if (_circular_buffer_idx > 0) {
            arm_mult_f32(&_downsampled_gyro_data[_update_axis][0], &_hanning_window[ringBufIdx], &_fft_data[ringBufIdx], _circular_buffer_idx);
        }
#endif
    }
    }

    _update_step = (_update_step + 1) % STEP_COUNT;
}

float Analyse_Noise::calculate_weighted_center_freq(uint8_t bin_start, uint8_t bin_max)
{
    // accumulate fft_sum and fft_weighted_sum from peak bin, and shoulder bins either side of peak
    float cubed_data = _fft_data[bin_max] * _fft_data[bin_max] * _fft_data[bin_max];
    float fft_sum = cubed_data;
    float fft_weighted_sum = cubed_data * (bin_max + 1);
    // accumulate upper shoulder
    for (int i = bin_max; i < FFT_BIN_COUNT - 1; i++) {
        if (_fft_data[i] > _fft_data[i + 1]) {
            cubed_data = _fft_data[i] * _fft_data[i] * _fft_data[i];
            fft_sum += cubed_data;
            fft_weighted_sum += cubed_data * (i + 1);
        }
        else {
            break;
        }
    }
    // accumulate lower shoulder
    for (int i = bin_max; i > bin_start + 1; i--) {
        if (_fft_data[i] > _fft_data[i - 1]) {
            cubed_data = _fft_data[i] * _fft_data[i] * _fft_data[i];
            fft_sum += cubed_data;
            fft_weighted_sum += cubed_data * (i + 1);
        }
        else {
            break;
        }
    }
    // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
    float weighted_center_freq_hz = 0;
    float fft_mean_index = 0;
    // idx was shifted by 1 to start at 1, not 0
    if (fft_sum > 0) {
        fft_mean_index = (fft_weighted_sum / fft_sum) - 1;
        // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
        weighted_center_freq_hz = fft_mean_index * _fft_resolution;
    }
    return weighted_center_freq_hz;
}
