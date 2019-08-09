/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper
 */

#include <AP_Math/AP_Math.h>
#include "AP_HAL.h"
#include "DSP.h"

using namespace AP_HAL;

extern const AP_HAL::HAL &hal;

#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f

DSP::FFTWindowState::FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps)
    : _update_steps(update_steps),
    _window_size(window_size),
    _bin_count(window_size / 2),
    _bin_resolution((float)sample_rate / (float)window_size)
{
    // includes DC ad Nyquist components and needs to be large enough for intermediate steps
    _freq_bins = new float[window_size];
    _hanning_window = new float[window_size];

    if (_freq_bins == nullptr || _hanning_window == nullptr) {
        delete[] _freq_bins;
        delete[] _hanning_window;
        _freq_bins = nullptr;
        _hanning_window = nullptr;
        return;
    }

    // create the Hanning window
    // https://holometer.fnal.gov/GH_FFT.pdf - equation 19
    for (uint16_t i = 0; i < window_size; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2.0f * M_PI * i / ((float)window_size - 1)));
        _window_scale += _hanning_window[i];
    }
    // Calculate the inverse of the Effective Noise Bandwidth
    _window_scale = 2.0f / sq(_window_scale);
}

DSP::FFTWindowState::~FFTWindowState()
{
    delete[] _freq_bins;
    delete[] _hanning_window;
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float DSP::calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k_max) const
{
    if (k_max <= 1 || k_max >= fft->_bin_count) {
        return 0.0f;
    }

    const uint16_t k_m1 = (k_max - 1) * 2;
    const uint16_t k_p1 = (k_max + 1) * 2;
    const uint16_t k = k_max * 2;

    const float divider = complex_fft[k] * complex_fft[k] + complex_fft[k+1] * complex_fft[k+1];
    const float ap = (complex_fft[k_p1] * complex_fft[k] + complex_fft[k_p1 + 1] * complex_fft[k+1]) / divider;
    const float am = (complex_fft[k_m1] * complex_fft[k] + complex_fft[k_m1 + 1] * complex_fft[k + 1]) / divider;

    // sanity check
    if (is_zero(ap) || is_zero(am)) {
        return 0.0f;
    }

    const float dp = -ap / (1.0f - ap);
    const float dm = am / (1.0f - am);

    float d = (dp + dm) * 0.5f + tau(dp * dp) - tau(dm * dm);

    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return constrain_float(d, -0.5f, 0.5f);
}

static const float TAU_FACTOR = SQRT_6 / 24.0f;

// Helper function used for Quinn's frequency estimation
float DSP::tau(const float x) const
{
    float p1 = logf(3.0f * sq(x) + 6.0f * x + 1.0f);
    float part1 = x + 1.0f - SQRT_2_3;
    float part2 = x + 1.0f + SQRT_2_3;
    float p2 = logf(part1 / part2);
    return (0.25f * p1 - TAU_FACTOR * p2);
}
