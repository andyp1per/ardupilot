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
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>
#include "Analyse_Noise.h"

// table of user settable parameters
const AP_Param::GroupInfo Analyse_Noise::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, Analyse_Noise, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Lower bound of FFT frequency detection in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 2, Analyse_Noise, _fft_min_hz, 80),

    // @Param: MINHZ
    // @DisplayName: Maximum Frequency
    // @Description: Upper bound of FFT frequency detection  in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MAXHZ", 3, Analyse_Noise, _fft_max_hz, 200),

    // @Param: SAMPLE_MODE
    // @DisplayName: Sample Mode
    // @Description: Sampling mode (and therefore rate). 0: Gyro rate sampling, 1: Fast loop rate sampling, 2: Fast loop rate / 2 sampling, 3: Fast loop rate / 3 sampling
    // @Range: 0 4
    // @User: Advanced
    // @RequiresReboot: True
    AP_GROUPINFO("SAMPLE_MODE", 4, Analyse_Noise, _sample_mode, 0),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency (Nyquist) of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// The first bin is used to store the DC and Nyquist values combined.
// Eg [DC/Nyquist], [16,47), [47,78), [78,109) etc
// For a loop rate of 800Hz, 16 bins each 25Hz wide
// Eg X[0]=[DC/Nyquist], X[1]=[12,37), X[2]=[37,62), X[3]=[62,87), X[4]=[87,112)
// So middle frequency is X[n] * 25 and the range is X[n] * 25 - 12 < f < X[n] * 25 + 12
// NB  FFT_WINDOW_SIZE is set to 32 in Analyse_Noise.h
#define FFT_BIN_COUNT (FFT_WINDOW_SIZE / 2)
// smoothing frequency for FFT centre frequency
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS (XYZ_AXIS_COUNT * 2)

#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f

#define TIMER_START(timer) uint32_t timer##now = AP_HAL::micros()
#define TIMER_END(timer) timer.time(timer##now)

Analyse_Noise::Analyse_Noise()
{
    _multiplier = 2000.0f / radians(2000); // scale from radian gyro output back to degrees
    AP_Param::setup_object_defaults(this, var_info);
}

void Analyse_Noise::init(uint32_t target_looptime_us, AP_InertialSensor& ins)
{
    _ins = &ins;

    if (_sample_mode == 0) {
        _fft_sampling_rate_hz = _ins->get_raw_gyro_rate_hz();
    }
    else {
        const uint16_t loop_rate_hz = 1000*1000UL / target_looptime_us;
        _fft_sampling_rate_hz = loop_rate_hz / _sample_mode;
    }
    _fft_resolution = (float)_fft_sampling_rate_hz / FFT_WINDOW_SIZE;
    _fft_start_bin = MAX(lrintf((float)_fft_min_hz.get() / _fft_resolution), 1);

    for (uint16_t i = 0; i < FFT_WINDOW_SIZE; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2 * M_PI * i / (FFT_WINDOW_SIZE - 1)));
    }

    analyse_init();
}

// A function called by the main thread at the main loop rate:
void Analyse_Noise::sample_gyros()
{
    if (!_enable) {
        return;
    }

    if (_sample_mode > 0) {
        _oversampled_gyro_accum += _ins->get_filtered_gyro();
        _sample_count++;

        if (_sample_count == _sample_mode) {
            _sample_count = 0;

            // calculate mean value of accumulated samples
            Vector3f sample = _oversampled_gyro_accum / _sample_mode;
            // fast sampling means that the raw gyro values have already been averaged over 8 samples
            _downsampled_gyro_data[0][_circular_buffer_idx] = sample.x * _multiplier;
            _downsampled_gyro_data[1][_circular_buffer_idx] = sample.y * _multiplier;
            _downsampled_gyro_data[2][_circular_buffer_idx] = sample.z * _multiplier;

            _circular_buffer_idx = (_circular_buffer_idx + 1) % FFT_WINDOW_SIZE;
            _oversampled_gyro_accum.zero();
            // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
            _update_ticks = DYN_NOTCH_CALC_TICKS;
        }
    }
    else {
        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        _update_ticks = DYN_NOTCH_CALC_TICKS;
    }

}

void Analyse_Noise::analyse_init()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_init_f32(&_fft_instance, FFT_WINDOW_SIZE);
#endif
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        _center_freq_hz_filtered[axis] = _fft_min_hz;
        _center_freq_hz[axis] = _fft_min_hz;
        _detected_frequency_filter[axis].set_cutoff_frequency(_fft_sampling_rate_hz, DYN_NOTCH_SMOOTH_FREQ_HZ);
    }
}

// Analyse gyro data
void Analyse_Noise::analyse()
{
    if (!_enable) {
        return;
    }

    // calculate FFT and update filters
    if (_update_ticks > 0) {
        if (_sample_mode > 0) {
            analyse_update(_downsampled_gyro_data[_update_axis], _circular_buffer_idx);
        }
        else {
            analyse_update(_ins->get_filtered_gyro_window(_update_axis), _ins->get_filtered_gyro_window_index());
        }
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

// Analyse last gyro data from the last FFT_WINDOW_SIZE 
// This is a state-machine version of arm_rfft_fast_f32() such that each step can be processed separately as required.
void Analyse_Noise::analyse_update(const GyroWindow& window, uint8_t buffer_index)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_cfft_instance_f32 *Sint = &(_fft_instance.Sint);
    Sint->fftLen = _fft_instance.fftLenRFFT / 2;
#endif

    switch (_update_step) {
    case STEP_HANNING: {
        // 5us
        // apply hanning window to gyro samples and store result in _fft_data
        // hanning starts and ends with 0, could be skipped for minor speed improvement
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        const uint16_t ringBufIdx = FFT_WINDOW_SIZE - buffer_index;
        arm_mult_f32(&window[buffer_index], &_hanning_window[0], &_fft_data[0], ringBufIdx);
        if (buffer_index > 0) {
            arm_mult_f32(&window[0], &_hanning_window[ringBufIdx], &_fft_data[ringBufIdx], buffer_index);
        }
#endif
        _update_step++;
        FALLTHROUGH;
        //break;
    }
    case STEP_ARM_CFFT_F32: {
        TIMER_START(_arm_cfft_f32_timer);

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        switch (FFT_BIN_COUNT) {
        case 16:
            // 16us (BF)
        case 128:
            arm_cfft_radix8by2_f32(Sint, _fft_data);
            break;
        case 32:
            // 35us (BF)
            // 5us F7
        case 256:
            // 37us F7
            arm_cfft_radix8by4_f32(Sint, _fft_data);
            break;
        case 64:
            // 70us BF
            // 10us F7
        case 512:
            // 77us F7
            arm_radix8_butterfly_f32(_fft_data, FFT_BIN_COUNT, Sint->pTwiddle, 1);
            break;
        }
#endif
        TIMER_END(_arm_cfft_f32_timer);

        break;
    }
    case STEP_BITREVERSAL: {
        TIMER_START(_bitreversal_timer);
        // 6us (BF)
        // 2us 32 F7
        // 3us 64 F7
        // 10us 256 F7
        // 18us 512 F7
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        arm_bitreversal_32((uint32_t *)_fft_data, Sint->bitRevLength, Sint->pBitRevTable);
#endif
        TIMER_END(_bitreversal_timer);

        _update_step++;
        FALLTHROUGH;
        //break;
    }
    case STEP_STAGE_RFFT_F32: {
        TIMER_START(_stage_rfft_f32_timer);
        // 14us (BF)
        // 2us 32 F7
        // 5us 64 F7
        // 21us 256 F7
        // 45us 512 F7
        // this does not work in place => _fft_data AND _rfft_data needed
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        stage_rfft_f32(&_fft_instance, _fft_data, _rfft_data);
#endif
        TIMER_END(_stage_rfft_f32_timer);

        _update_step++;
        FALLTHROUGH;
        //break;
    }
    case STEP_ARM_CMPLX_MAG_F32: {
        // 8us (BF)
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        // General case for the magnitudes - see https://stackoverflow.com/questions/42299932/dsp-libraries-rfft-strange-results
        // The frequency of each of those frequency components are given by k*fs/N
        arm_cmplx_mag_f32(&_rfft_data[2], &_fft_data[1], FFT_BIN_COUNT-1);
        _fft_data[0] = fabs(_rfft_data[0]); // DC
        _fft_data[FFT_BIN_COUNT] = fabs(_rfft_data[1]); // Nyquist
        _fft_data[FFT_BIN_COUNT + 1] = 0; // So that interpolation works correctly
#endif
        _update_step++;
        FALLTHROUGH;
        //break;
    }
    case STEP_CALC_FREQUENCIES: {
        // 3us 32 F7
        // 5us 64 F7
        // 10us 256 F7
        // 29us 512 F7
        TIMER_START(_calc_freq_timer);

        float maxValue = 0;
        uint32_t maxBin = 0;
        arm_max_f32(&_fft_data[_fft_start_bin], (FFT_BIN_COUNT + 1) - _fft_start_bin, &maxValue, &maxBin);
        maxBin += _fft_start_bin;
        _center_freq_energy[_update_axis] = maxValue;
        _center_freq_bin[_update_axis] = maxBin;

        float weighted_center_freq_hz = 0;

        if (_fft_data[maxBin] > _ref_energy[_update_axis]) {
            weighted_center_freq_hz = calculate_jains_estimator_center_freq(maxBin);
        }
        else {
            weighted_center_freq_hz = _fft_min_hz;
        }
        weighted_center_freq_hz = MAX(weighted_center_freq_hz, (float)_fft_min_hz);
        _center_freq_hz[_update_axis] = weighted_center_freq_hz;
        weighted_center_freq_hz = _detected_frequency_filter[_update_axis].apply(weighted_center_freq_hz);
        _center_freq_hz_filtered[_update_axis] = weighted_center_freq_hz;

        TIMER_END(_calc_freq_timer);
#ifdef DEBUG_FFT
        _output_count++;
        if (_output_count % 300 == 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "FFT: t1:%luus, t2:%luus, t3:%luus, t4:%luus",
                _arm_cfft_f32_timer._timer_avg, _bitreversal_timer._timer_avg, _stage_rfft_f32_timer._timer_avg, _calc_freq_timer._timer_avg);
            gcs().send_text(MAV_SEVERITY_WARNING, "FFT: e:%.1f, b:%lu, f:%.1f, r:%.1f",
              maxValue, maxBin, _center_freq_hz[_update_axis], _ref_energy[_update_axis]);
            gcs().send_text(MAV_SEVERITY_WARNING, "[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
              _fft_data[0],_fft_data[1],_fft_data[2],_fft_data[3],_fft_data[4],_fft_data[5],_fft_data[6],_fft_data[7],_fft_data[8],_fft_data[9],_fft_data[10],_fft_data[11],_fft_data[12],_fft_data[13],_fft_data[14],_fft_data[15]);
        }
#endif

        _update_step++;
        FALLTHROUGH;
    }
    case STEP_UPDATE_FILTERS: {
        _update_axis = (_update_axis + 1) % XYZ_AXIS_COUNT;
        break;
    }
    }

    _update_step = (_update_step + 1) % STEP_COUNT;
}

// Interpolate center frequency using simple center of bin
float Analyse_Noise::calculate_simple_center_freq(uint8_t bin_max)
{
    float weighted_center_freq_hz = 0;
    // The frequency of each of those frequency components are given by k*fs/N, so first bin is DC.
    if (_fft_data[bin_max] > 0) {
        // the index points at the center frequency of each bin so index 1 is actually 1 * 800 / 64 = 12.5Hz
        weighted_center_freq_hz = bin_max * _fft_resolution;
    }
    return weighted_center_freq_hz;
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float Analyse_Noise::calculate_jains_estimator_center_freq(uint8_t k)
{
    if (k == 0 || k == FFT_BIN_COUNT) {
        return k * _fft_resolution;
    }

    const float y1 = _fft_data[k - 1];
    const float y2 = _fft_data[k];
    const float y3 = _fft_data[k + 1];

    if (y1  > y3) {
        const float a = y2 / y1;
        const float d = a / (1 + a);
        return (k - 1.0f + d) * _fft_resolution;
    }
    else {
        const float a = y3 / y2;
        const float d = a / (1 + a);
        return (k + d) * _fft_resolution;
    }
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float Analyse_Noise::calculate_quinns_second_estimator_center_freq(uint8_t k)
{
    if (k == 0 || k == FFT_BIN_COUNT) {
        return k * _fft_resolution;
    }

    const float ap = _fft_data[k + 1] / _fft_data[k];
    const float dp = -ap / (1.0f - ap);

    const float am = _fft_data[k - 1] / _fft_data[k];
    const float dm = am / (1.0f - am);

    float d = (dp + dm) / 2.0f + tau(dp * dp) - tau(dm * dm);

    // When the side lobes are very close in magnitude to center d increases dramatically
    d = constrain_float(d, -0.5f, 0.5f);
    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return (k + d) * _fft_resolution;
}

// Helper function used for Quinn's frequency estimation
float Analyse_Noise::tau(float x)
{
    float p1 = logf(3.0f * powf(x, 2.0f) + 6.0f * x + 1.0f);
    float part1 = x + 1.0f - SQRT_2_3;
    float part2 = x + 1.0f + SQRT_2_3;
    float p2 = logf(part1 / part2);
    return (0.25f * p1 - (SQRT_6 / 24.0f) * p2);
}

// self-test the FT analyzer - can only be done while samples are not being taken
bool Analyse_Noise::calibration_check() {
    // Baseline double the gyro noise, this avoids detecting noise as a signal
    if (_ref_energy.is_zero()) {
        _ref_energy = _center_freq_energy * 2.0f;
    }

    if (_fft_max_hz > _fft_sampling_rate_hz / 2) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FFT: config MAXHZ %hdHz > %hdHz", _fft_max_hz.get(), _fft_sampling_rate_hz / 2);
        return false;
    }

    if (DataFlash_Class::instance()->log_while_disarmed()) {
        return true;
    }

    const Vector3f ref_energy = _ref_energy;

    float max_divergence = self_test_bin_frequencies();

    _ref_energy = ref_energy;

    if (max_divergence > _fft_resolution / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test FAILED, max error %fHz", max_divergence);
    }
    return max_divergence <= _fft_resolution / 2;
}

// perform FFT analysis on the range of frequencies supported by the analyzer
float Analyse_Noise::self_test_bin_frequencies() {
    float max_divergence = 0;

    for (uint8_t bin = _fft_start_bin; bin < FFT_BIN_COUNT; bin++) {
        max_divergence = MAX(max_divergence, self_test(bin * _fft_resolution)); // test bin centers
        max_divergence = MAX(max_divergence, self_test(bin * _fft_resolution - _fft_resolution / 4)); // test bin off-centers
    }
    return max_divergence;
}

// perform FFT analysis of a single sine wave at the selected frequency
float Analyse_Noise::self_test(float frequency) {
    static GyroWindow test_window;
    for(uint16_t i = 0; i < FFT_WINDOW_SIZE; i++) {
        test_window[i]= sinf(2.0f * M_PI * frequency * i / _fft_sampling_rate_hz) * 2000.0f;
    }

    _update_step = STEP_HANNING;
    _update_axis = 0;
    _ref_energy.zero();

    for (uint8_t s = 0; s < (DYN_NOTCH_CALC_TICKS / XYZ_AXIS_COUNT); s++) {
        analyse_update(test_window, 0);
    }

    if (_update_step != STEP_HANNING) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed, ended on step %d", _update_step);
    }

    float max_divergence = 0;
    // make sure the selected frequencies are in the right bin
    max_divergence = MAX(max_divergence, fabs(frequency - _center_freq_hz[0]));
    if (_center_freq_hz[0] < (frequency - _fft_resolution / 2) || _center_freq_hz[0] > (frequency + _fft_resolution / 2)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed: wanted %f, had %f", frequency, _center_freq_hz[0]);
    }

    return max_divergence;
}
