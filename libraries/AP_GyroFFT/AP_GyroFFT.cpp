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

   Code by Andy Piper with help from betaflight
 */

#include "AP_GyroFFT.h"
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

#define FFT_DEFAULT_WINDOW_SIZE 32

// table of user settable parameters
const AP_Param::GroupInfo AP_GyroFFT::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable Gyro FFT analyser
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_GyroFFT, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Lower bound of FFT frequency detection in Hz. Takes effect on reboot.
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MINHZ", 2, AP_GyroFFT, _fft_min_hz, 80),

    // @Param: MINHZ
    // @DisplayName: Maximum Frequency
    // @Description: Upper bound of FFT frequency detection in Hz. Takes effect on reboot.
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAXHZ", 3, AP_GyroFFT, _fft_max_hz, 200),

    // @Param: SAMPLE_MODE
    // @DisplayName: Sample Mode
    // @Description: Sampling mode (and therefore rate). 0: Gyro rate sampling, 1: Fast loop rate sampling, 2: Fast loop rate / 2 sampling, 3: Fast loop rate / 3 sampling. Takes effect on reboot.
    // @Range: 0 4
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SAMPLE_MODE", 4, AP_GyroFFT, _sample_mode, 0),

    // @Param: WINDOW_SIZE
    // @DisplayName: FFT window size
    // @Description: Size of window to be used in FFT calculations. Takes effect on reboot. Must be a power of 2 and between 32 and 1024. Larger windows give greater frequency resolution but consume more CPU time and may not be appropriate for all vehicles.
    // @Range: 32 1024
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("WINDOW_SIZE", 5, AP_GyroFFT, _window_size, FFT_DEFAULT_WINDOW_SIZE),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency (Nyquist) of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// The first bin is used to store the DC and Nyquist values combined.
// Eg [DC/Nyquist], [16,47), [47,78), [78,109) etc
// For a loop rate of 800Hz, 16 bins each 25Hz wide
// Eg X[0]=[DC/Nyquist], X[1]=[12,37), X[2]=[37,62), X[3]=[62,87), X[4]=[87,112)
// So middle frequency is X[n] * 25 and the range is X[n] * 25 - 12 < f < X[n] * 25 + 12

// smoothing frequency for FFT centre frequency
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50
#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f
//#define DEBUG_FFT

AP_GyroFFT::AP_GyroFFT()
{
    _multiplier = 2000.0f / radians(2000); // scale from radian gyro output back to degrees
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GyroFFT must be singleton");
    }
    _singleton = this;
}

void AP_GyroFFT::init(uint32_t target_looptime_us, AP_InertialSensor& ins)
{
    _ins = &ins;

    // check that we have enough memory for the window size requested
    const uint32_t total_allocation = (4 + XYZ_AXIS_COUNT * INS_MAX_INSTANCES) * _window_size * sizeof(float);
    if (total_allocation > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "HAL: requested alloc %u bytes for DSP (free=%u)", (unsigned int)total_allocation, (unsigned int)hal.util->available_memory());
        _window_size = FFT_DEFAULT_WINDOW_SIZE;
    }
    // check that we support the window size requested and it is a power of 2
    _window_size = 1 << lrintf(log2f(_window_size.get()));
    _window_size = constrain_int16(_window_size, 32, 1024);

    // determine the FFT sample rate based on the gyro rate, loop rate and configuration
    if (_sample_mode == 0) {
        _fft_sampling_rate_hz = _ins->get_raw_gyro_rate_hz();
    }
    else {
        const uint16_t loop_rate_hz = 1000*1000UL / target_looptime_us;
        _fft_sampling_rate_hz = loop_rate_hz / _sample_mode;
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            _downsampled_gyro_data[axis] = new float[_window_size];
        }
    }
    // make the INS window match the window size
    ins.set_gyro_window_size(_window_size);
    // initialise the HAL subsystem
    _state = hal.dsp->fft_init(_window_size, _fft_sampling_rate_hz);

    _fft_start_bin = MAX(lrintf((float)_fft_min_hz.get() / _state->_bin_resolution), 1);

    // establish suitable defaults
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        _center_freq_hz_filtered[axis] = _fft_min_hz;
        _center_freq_hz[axis] = _fft_min_hz;
        _detected_frequency_filter[axis].set_cutoff_frequency(_fft_sampling_rate_hz, DYN_NOTCH_SMOOTH_FREQ_HZ);
    }
}

// A function called by the main thread at the main loop rate:
void AP_GyroFFT::sample_gyros()
{
    if (!_enable) {
        return;
    }

    // for loop rate sampling accumulate and averae gyro samples
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

            _circular_buffer_idx = (_circular_buffer_idx + 1) % _state->_window_size;
            _oversampled_gyro_accum.zero();
        }
    }
}

// Analyse gyro data
void AP_GyroFFT::update()
{
    if (!_enable) {
        return;
    }

    // calculate FFT and update filters
    uint16_t bin_max = 0;
    if (_sample_mode) {
        // loop rate sampling
        bin_max = hal.dsp->fft_analyse(_state, _downsampled_gyro_data[_update_axis], _circular_buffer_idx, _fft_start_bin);
    }
    else {
        // gyro rate sampling
        bin_max = hal.dsp->fft_analyse(_state, _ins->get_filtered_gyro_window(_update_axis), _ins->get_filtered_gyro_window_index(), _fft_start_bin);
    }
    if (bin_max > 0) {
        calculate_noise(bin_max);
    }
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
void AP_GyroFFT::calculate_noise(uint16_t max_bin)
{
    _center_freq_energy[_update_axis] = _state->_freq_bins[max_bin];
    _center_freq_bin[_update_axis] = max_bin;

    float weighted_center_freq_hz = 0;

    // if the bin energy is above the noise threshold then record it
    if (_state->_freq_bins[max_bin] > _ref_energy[_update_axis]) {
        weighted_center_freq_hz = calculate_jains_estimator_center_freq(max_bin);
    }
    else {
        weighted_center_freq_hz = _fft_min_hz;
    }
    weighted_center_freq_hz = MAX(weighted_center_freq_hz, (float)_fft_min_hz);
    _center_freq_hz[_update_axis] = weighted_center_freq_hz;
    weighted_center_freq_hz = _detected_frequency_filter[_update_axis].apply(weighted_center_freq_hz);
    _center_freq_hz_filtered[_update_axis] = weighted_center_freq_hz;

#ifdef DEBUG_FFT
    _output_count++;
    // output at approx 1hz
   if (_output_count % (400 / _state->_update_steps) == 0)
   {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: e:%.1f, b:%u, f:%.1f, r:%.1f",
                        _state->_freq_bins[max_bin], max_bin, _center_freq_hz[_update_axis], _ref_energy[_update_axis]);
        gcs().send_text(MAV_SEVERITY_WARNING, "[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
                        _state->_freq_bins[0], _state->_freq_bins[1], _state->_freq_bins[2], _state->_freq_bins[3], _state->_freq_bins[4], _state->_freq_bins[5], _state->_freq_bins[6], _state->_freq_bins[7],
                        _state->_freq_bins[8], _state->_freq_bins[9], _state->_freq_bins[10], _state->_freq_bins[11], _state->_freq_bins[12], _state->_freq_bins[13], _state->_freq_bins[14], _state->_freq_bins[15]);
   }
#endif
    _update_axis = (_update_axis + 1) % XYZ_AXIS_COUNT;
}

// Interpolate center frequency using simple center of bin
float AP_GyroFFT::calculate_simple_center_freq(uint8_t bin_max)
{
    float weighted_center_freq_hz = 0;
    // The frequency of each of those frequency components are given by k*fs/N, so first bin is DC.
    if (_state->_freq_bins[bin_max] > 0) {
        // the index points at the center frequency of each bin so index 1 is actually 1 * 800 / 64 = 12.5Hz
        weighted_center_freq_hz = bin_max * _state->_bin_resolution;
    }
    return weighted_center_freq_hz;
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float AP_GyroFFT::calculate_jains_estimator_center_freq(uint8_t k)
{
    if (k == 0 || k == _state->_bin_count) {
        return k * _state->_bin_resolution;
    }

    const float y1 = _state->_freq_bins[k - 1];
    const float y2 = _state->_freq_bins[k];
    const float y3 = _state->_freq_bins[k + 1];

    if (y1  > y3) {
        const float a = y2 / y1;
        const float d = a / (1 + a);
        return (k - 1.0f + d) * _state->_bin_resolution;
    }
    else {
        const float a = y3 / y2;
        const float d = a / (1 + a);
        return (k + d) * _state->_bin_resolution;
    }
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float AP_GyroFFT::calculate_quinns_second_estimator_center_freq(uint8_t k)
{
    if (k == 0 || k == _state->_bin_count) {
        return k * _state->_bin_resolution;
    }

    const float ap = _state->_freq_bins[k + 1] / _state->_freq_bins[k];
    const float dp = -ap / (1.0f - ap);

    const float am = _state->_freq_bins[k - 1] / _state->_freq_bins[k];
    const float dm = am / (1.0f - am);

    float d = (dp + dm) / 2.0f + tau(dp * dp) - tau(dm * dm);

    // When the side lobes are very close in magnitude to center d increases dramatically
    d = constrain_float(d, -0.5f, 0.5f);
    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return (k + d) * _state->_bin_resolution;
}

// Helper function used for Quinn's frequency estimation
float AP_GyroFFT::tau(float x)
{
    float p1 = logf(3.0f * powf(x, 2.0f) + 6.0f * x + 1.0f);
    float part1 = x + 1.0f - SQRT_2_3;
    float part2 = x + 1.0f + SQRT_2_3;
    float p2 = logf(part1 / part2);
    return (0.25f * p1 - (SQRT_6 / 24.0f) * p2);
}

// self-test the FFT analyser - can only be done while samples are not being taken
bool AP_GyroFFT::calibration_check() {
    if (!_enable) {
        return true;
    }

    // Baseline double the gyro noise, this avoids detecting noise as a signal
    if (_ref_energy.is_zero()) {
        _ref_energy = _center_freq_energy * 2.0f;
    }

    if (_fft_max_hz > _fft_sampling_rate_hz / 2) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FFT: config MAXHZ %hdHz > %dHz", _fft_max_hz.get(), _fft_sampling_rate_hz / 2);
        return false;
    }

    // larger windows make the the self-test run too long, triggering the watchdog
    if (DataFlash_Class::instance()->log_while_disarmed() || _window_size > FFT_DEFAULT_WINDOW_SIZE) {
        return true;
    }

    const Vector3f ref_energy = _ref_energy;

    float max_divergence = self_test_bin_frequencies();

    _ref_energy = ref_energy;

    if (max_divergence > _state->_bin_resolution / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test FAILED, max error %fHz", max_divergence);
    }
    return max_divergence <= _state->_bin_resolution / 2;
}

// perform FFT analysis on the range of frequencies supported by the analyser
float AP_GyroFFT::self_test_bin_frequencies() {
    float max_divergence = 0;

    for (uint8_t bin = _fft_start_bin; bin < _state->_bin_count; bin++) {
        max_divergence = MAX(max_divergence, self_test(bin * _state->_bin_resolution)); // test bin centers
        max_divergence = MAX(max_divergence, self_test(bin * _state->_bin_resolution - _state->_bin_resolution / 4)); // test bin off-centers
    }
    return max_divergence;
}

// perform FFT analysis of a single sine wave at the selected frequency
float AP_GyroFFT::self_test(float frequency) {
    static GyroWindow test_window = new float[_state->_window_size];
    for(uint16_t i = 0; i < _state->_window_size; i++) {
        test_window[i]= sinf(2.0f * M_PI * frequency * i / _fft_sampling_rate_hz) * 2000.0f;
    }

    _update_axis = 0;
    _state->reset();
    _ref_energy.zero();

    uint16_t max_bin = 0;
    uint8_t maxsteps = 8;
    while ((max_bin = hal.dsp->fft_analyse(_state, test_window, 0, _fft_start_bin)) == 0 && maxsteps-- > 0) {
        // NOOP
    }
    calculate_noise(max_bin);

    if (maxsteps == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed, failed to find frequency %f", frequency);
    }

    float max_divergence = 0;
    // make sure the selected frequencies are in the right bin
    max_divergence = MAX(max_divergence, fabs(frequency - _center_freq_hz[0]));
    if (_center_freq_hz[0] < (frequency - _state->_bin_resolution / 2) || _center_freq_hz[0] > (frequency + _state->_bin_resolution / 2)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed: wanted %f, had %f", frequency, _center_freq_hz[0]);
    }

    return max_divergence;
}

// singleton instance
AP_GyroFFT *AP_GyroFFT::_singleton;

namespace AP {

AP_GyroFFT *fft()
{
    return AP_GyroFFT::get_singleton();
}

}
