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

#define FFT_DEFAULT_WINDOW_SIZE     32
#define FFT_DEFAULT_WINDOW_OVERLAP  0.5f
#define FFT_THR_REF_DEFAULT         0.35f   // the estimated throttle reference, 0 ~ 1

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
    // @Description: Size of window to be used in FFT calculations. Takes effect on reboot. Must be a power of 2 and between 32 and 256. Larger windows give greater frequency resolution but poorer time resolution, consume more CPU time and may not be appropriate for all vehicles. Time and frequency resolution are given by the sample-rate / window-size. Windows of 256 are only really recommended for F7 class boards.
    // @Range: 32 256
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("WINDOW_SIZE", 5, AP_GyroFFT, _window_size, FFT_DEFAULT_WINDOW_SIZE),

    // @Param: WINDOW_OLAP
    // @DisplayName: FFT window overlap
    // @Description: Percentage of window to be overlapped before another frame is process. Takes effect on reboot. A good default is 50% overlap. Higher overlap results in more processed frames but not necessarily more temporal resolution. Lower overlap results in lost information at the frame edges.
    // @Range: 0 0.9
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("WINDOW_OLAP", 6, AP_GyroFFT, _window_overlap, FFT_DEFAULT_WINDOW_OVERLAP),

    // @Param: FREQ_HOVER
    // @DisplayName: FFT learned hover frequency
    // @Description: The learned hover noise frequency
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("FREQ_HOVER", 7, AP_GyroFFT, _freq_hover, 80.0f),

    // @Param: THR_REF
    // @DisplayName: FFT learned thrust reference
    // @Description: FFT learned thrust reference for the hover frequency and FFT minimum frequency.
    // @Range: 0.01 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_REF", 8, AP_GyroFFT, _throttle_ref, FFT_THR_REF_DEFAULT),

    // @Param: TRACK_MODE
    // @DisplayName: FFT frequency tracking type
    // @Description: FFT frequency tracking type
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("TRACK_MODE", 9, AP_GyroFFT, _track_mode, 0),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency (Nyquist) of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// The first bin is used to store the DC and Nyquist values combined.
// Eg [DC/Nyquist], [16,47), [47,78), [78,109) etc
// For a loop rate of 800Hz, 16 bins each 25Hz wide
// Eg X[0]=[DC/Nyquist], X[1]=[12,37), X[2]=[37,62), X[3]=[62,87), X[4]=[87,112)
// So middle frequency is X[n] * 25 and the range is X[n] * 25 - 12 < f < X[n] * 25 + 12

#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f
// Maximum tolerated number of cycles with missing signal
#define FFT_MAX_MISSED_UPDATES 3

AP_GyroFFT::AP_GyroFFT()
{
    _multiplier = 2000.0f / radians(2000); // scale from radian gyro output back to degrees
    _noise_needs_calibration = 0x07;
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GyroFFT must be singleton");
    }
    _singleton = this;
}

void AP_GyroFFT::init(uint32_t target_looptime_us, AP_InertialSensor& ins)
{
    _ins = &ins;

    // check that we support the window size requested and it is a power of 2
    _window_size = 1 << lrintf(log2f(_window_size.get()));
    _window_size = constrain_int16(_window_size, 32, 256);

    // check that we have enough memory for the window size requested
    const uint32_t total_allocation = (3 + XYZ_AXIS_COUNT * INS_MAX_INSTANCES) * _window_size * sizeof(float);
    if (total_allocation > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AP_GyroFFT: req alloc %u bytes (free=%u)", (unsigned int)total_allocation, (unsigned int)hal.util->available_memory());
        _window_size = FFT_DEFAULT_WINDOW_SIZE;
    }
    // save any changes that were made
    _window_size.save();

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

    // number of samples needed before a new frame can be processed
    _window_overlap = constrain_float(_window_overlap, 0.0f, 0.9f);
    _window_overlap.save();
    _samples_per_frame = (1.0f - _window_overlap) * _window_size;
    _samples_per_frame = 1 << lrintf(log2f(_samples_per_frame));
 
    // make the INS window match the window size
    ins.set_gyro_window_size(_window_size);

    // initialise the HAL DSP subsystem
    _state = hal.dsp->fft_init(_window_size, _fft_sampling_rate_hz);

    _fft_start_bin = MAX(lrintf((float)_fft_min_hz.get() / _state->_bin_resolution), 1);

    // The update rate for the output
    const float output_rate = _fft_sampling_rate_hz / _window_overlap;
    // establish suitable defaults
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        _center_freq_hz[axis] = _fft_min_hz;
        _center_freq_hz_filtered[axis] = _fft_min_hz;
        // Calculate low-pass filter characteristics based on overlap size
        _center_freq_filter[axis].set_cutoff_frequency(output_rate, output_rate / 3.0f);
    }

    // the number of cycles required to have a proper noise reference
    _noise_cycles = (_window_size / _samples_per_frame) * XYZ_AXIS_COUNT;
}

// A function called by the main thread at the main loop rate:
void AP_GyroFFT::sample_gyros()
{
    if (!_enable) {
        return;
    }

    // In order to correctly reconstruct the signal from the samples we must observe COLA: https://ccrma.stanford.edu/~jos/sasp/Overlap_Add_OLA_STFT_Processing.html
    if (_sample_mode == 0) {
        _sample_count += ((_ins->get_filtered_gyro_window_index() - _circular_buffer_idx + _window_size) % _window_size);
        _circular_buffer_idx = _ins->get_filtered_gyro_window_index();
    }
    // for loop rate sampling accumulate and average gyro samples
    else {
        _oversampled_gyro_accum += _ins->get_filtered_gyro();
        _sample_count++;

        if ((_sample_count % _sample_mode) == 0) {
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

    if (_sample_count >= _samples_per_frame) {
#if defined(DEBUG_FFT) || defined(DEBUG_FFT_TIMING)
        _output_count++;
#endif
        uint32_t now = AP_HAL::micros();

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
            _sample_count -= _samples_per_frame;
            update_ref_energy();
            calculate_noise(bin_max);
        }

        uint32_t time_taken = AP_HAL::micros() - now;

        if (time_taken > FFT_UPDATE_BUDGET_MICROS) {
            _overrun_cycles++;
            _overrun_total += time_taken;
            _overrun_max = MAX(_overrun_max, time_taken);
        }
#ifdef DEBUG_FFT_TIMING
        if (_output_count % (400 / _state->_update_steps) == 0 && _overrun_total > 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "FFT: oruns:%lu, orunvag:%lu", get_total_overrun_cycles(), get_average_overrun());
        }
#endif
    }
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
void AP_GyroFFT::update_ref_energy() {
    if (!_noise_needs_calibration) {
        return;
    }
    // baseline the noise floor by averaging all the bins, this avoids detecting noise as a signal
    if (_noise_cycles == 0 && is_zero(_ref_energy[_update_axis])) {
        for (uint8_t i = 1; i < _state->_bin_count; i++) {
            _ref_energy[_update_axis] += _state->_freq_bins[i];
        }
        _ref_energy[_update_axis] = _ref_energy[_update_axis] * 100.0f / (_state->_bin_count - 1);
        _noise_needs_calibration &= ~(1 << _update_axis);
    }
    else if (_noise_cycles > 0) {
        _noise_cycles--;
    }
}

// update the hover frequency input filter.  should be called at 100hz when in a stable hover
void AP_GyroFFT::update_freq_hover(float dt, float throttle_out)
{
    // we have chosen to constrain the hover frequency to be within the range reachable by the third order expo polynomial.
    _freq_hover = constrain_float(_freq_hover + (dt / (10.0f + dt)) * (get_weighted_noise_center_freq_hz() - _freq_hover), _fft_min_hz, _fft_max_hz);
    _throttle_ref = constrain_float(_throttle_ref + (dt / (10.0f + dt)) * (throttle_out * powf((float)_fft_min_hz.get() / _freq_hover, 2.0f) - _throttle_ref), 0.01f, 0.9f);
}

// save parameters as part of disarming
void AP_GyroFFT::save_params_on_disarm()
{
    _freq_hover.save();
    _throttle_ref.save();
}

// Return an average center frequency weighted by bin energy
float AP_GyroFFT::get_weighted_noise_center_freq_hz() const
{
    if (!_center_freq_energy.is_nan()
        && !is_zero(_center_freq_energy.x)
        && !is_zero(_center_freq_energy.y)) {
        return (_center_freq_hz_filtered.x * _center_freq_energy.x + _center_freq_hz_filtered.y * _center_freq_energy.y) / (_center_freq_energy.x + _center_freq_energy.y);
    }
    else {
        return (_center_freq_hz_filtered.x + _center_freq_hz_filtered.y) / 2.0f;
    }
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
void AP_GyroFFT::calculate_noise(uint16_t max_bin)
{
    _center_freq_bin[_update_axis] = max_bin;

    float weighted_center_freq_hz = 0;

    // if the bin energy is above the noise threshold then we have a signal
    if (!isnan(_state->_freq_bins[max_bin]) && _state->_freq_bins[max_bin] > _ref_energy[_update_axis]) {
        _center_freq_energy[_update_axis] =_state->_freq_bins[max_bin];
        weighted_center_freq_hz = MAX(_state->_max_bin_freq, (float)_fft_min_hz);
        _center_freq_hz[_update_axis] = weighted_center_freq_hz;
        _missed_cycles = 0;
    }
    // if we failed to find a signal, carry on using the previous reading
    else if (_missed_cycles++ < FFT_MAX_MISSED_UPDATES) {
        weighted_center_freq_hz = _center_freq_hz[_update_axis];
        // Keep the previous center frequency and energy
    }
    // we failed to find a signal for more than FFT_MAX_MISSED_UPDATES cycles
    else {
        weighted_center_freq_hz = _fft_min_hz;
        _center_freq_hz[_update_axis] = _fft_min_hz;
        _center_freq_energy[_update_axis] = 0.0f;
    }

    _center_freq_hz_filtered[_update_axis] = _center_freq_filter[_update_axis].apply(weighted_center_freq_hz);

#ifdef DEBUG_FFT
    // output at approx 1hz
   if (_output_count % (400 / _state->_update_steps) == 0)
   {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: e:%.1f, b:%u, f:%.1f, fr:%.1f, r:%.1f, d:%.1f",
                        _state->_freq_bins[max_bin], max_bin, _center_freq_hz_filtered[_update_axis], _center_freq_hz[_update_axis], _ref_energy[_update_axis], _state->_max_bin_freq);
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

// self-test the FFT analyser - can only be done while samples are not being taken
bool AP_GyroFFT::calibration_check() {
    if (!_enable) {
        return true;
    }

    if (_fft_max_hz > _fft_sampling_rate_hz / 2) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FFT: config MAXHZ %hdHz > %dHz", _fft_max_hz.get(), _fft_sampling_rate_hz / 2);
        return false;
    }

    // larger windows make the the self-test run too long, triggering the watchdog
    if (DataFlash_Class::instance()->log_while_disarmed()
        || _window_size > FFT_DEFAULT_WINDOW_SIZE * 2
        || _noise_needs_calibration) {
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

    if (maxsteps == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed, failed to find frequency %f", frequency);
    }

    calculate_noise(max_bin);

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
