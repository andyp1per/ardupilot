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
 * Code by Andy Piper and the betaflight team
 */

#include "AP_HAL_SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "DSP.h"
#include <cmath>

using namespace HALSITL;

// The algorithms originally came from betaflight but are now substantially modified based on theory and experiment.
// https://holometer.fnal.gov/GH_FFT.pdf "Spectrum and spectral density estimation by the Discrete Fourier transform (DFT),
// including a comprehensive list of window functions and some new flat-top windows." - Heinzel et. al is a great reference
// for understanding the underlying theory although we do not use spectral density here since time resolution is equally
// important as frequency resolution. Referred to as [Heinz] throughout the code.

// initialize the FFT state machine
AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate)
{
    DSP::FFTWindowStateSITL* fft = new DSP::FFTWindowStateSITL(window_size, sample_rate);
    if (fft->_hanning_window == nullptr || fft->_rfft_data == nullptr || fft->_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }
    return fft;
}

// start an FFT analysis
void DSP::fft_start(AP_HAL::DSP::FFTWindowState* state, const float* samples, uint16_t buffer_index)
{
    step_hanning((FFTWindowStateSITL*)state, samples, buffer_index);
}

// perform remaining steps of an FFT analysis
uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, uint16_t start_bin, uint16_t end_bin)
{
    FFTWindowStateSITL* fft = (FFTWindowStateSITL*)state;
    step_fft(fft);
    step_cmplx_mag_f32(fft, start_bin, end_bin);
    return step_calc_frequencies(fft, start_bin, end_bin);
}

// create an instance of the FFT state machine
DSP::FFTWindowStateSITL::FFTWindowStateSITL(uint16_t window_size, uint16_t sample_rate)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate)
{
    if (_freq_bins == nullptr || _hanning_window == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate window for DSP");
        return;
    }

    // allocate workspace, +2 includes the nyquist component necessary for the interoplator to work acorss the whole range
    _rfft_data = new float[_window_size + 2];
    if (_rfft_data == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate window for DSP");
        return;
    }
}

DSP::FFTWindowStateSITL::~FFTWindowStateSITL()
{
    delete[] _rfft_data;
}

// step 1: filter the incoming samples through a Hanning window
void DSP::step_hanning(FFTWindowStateSITL* fft, const float* samples, uint16_t buffer_index)
{
    // 5us
    // apply hanning window to gyro samples and store result in _freq_bins
    // hanning starts and ends with 0, could be skipped for minor speed improvement
    const uint16_t ring_buf_idx = fft->_window_size - buffer_index;
    mult_f32(&samples[buffer_index], &fft->_hanning_window[0], &fft->_freq_bins[0], ring_buf_idx);
    if (buffer_index > 0) {
        mult_f32(&samples[0], &fft->_hanning_window[ring_buf_idx], &fft->_freq_bins[ring_buf_idx], buffer_index);
    }
}

// step 2: performm an in-place FFT on the windowed data
void DSP::step_fft(FFTWindowStateSITL* fft)
{
    complexf* buf = new complexf[fft->_window_size];

    for (uint32_t i = 0; i < fft->_window_size; i++) {
        buf[i] = complexf(fft->_freq_bins[i], 0);
    }

    calculate_fft(buf, fft->_window_size);

    for (uint32_t i = 0; i < fft->_bin_count; i++) {
        fft->_freq_bins[i] = std::norm(buf[i]);
    }

    // components at the nyquist frequency are real only
    for (uint32_t i = 0, j=0; i <= fft->_bin_count; i++, j+=2) {
        fft->_rfft_data[j] = buf[i].real();
        fft->_rfft_data[j+1] = buf[i].imag();
    }

    delete[] buf;
}

// step 3: find the magnitudes of the complex data
void DSP::step_cmplx_mag_f32(FFTWindowStateSITL* fft, uint16_t start_bin, uint16_t end_bin)
{
    // find the maximum power in the range we are interested in
    float max_value = 0;
    uint16_t bin_range = (end_bin - start_bin) + 1;
    max_f32(&fft->_freq_bins[start_bin], bin_range, &max_value, &fft->_max_energy_bin);
    fft->_max_energy_bin += start_bin;
    // scale the power to account for the input window
    scale_f32(fft->_freq_bins, fft->_window_scale, fft->_freq_bins, fft->_bin_count);
}

// step 4: find the bin with the highest energy and interpolate the required frequency
uint16_t DSP::step_calc_frequencies(FFTWindowStateSITL* fft, uint16_t start_bin, uint16_t end_bin)
{
    if (is_zero(fft->_freq_bins[fft->_max_energy_bin])) {
        fft->_max_bin_freq = start_bin * fft->_bin_resolution;
    } else {
        // It turns out that Jain is pretty good and works with only magnitudes, but Candan is significantly better
        // if you have access to the complex values and Quinn is a little better still. Quinn is computationally
        // more expensive, but compared to the overall FFT cost seems worth it.
        fft->_max_bin_freq = (fft->_max_energy_bin + calculate_quinns_second_estimator(fft, fft->_rfft_data, fft->_max_energy_bin)) * fft->_bin_resolution;
    }

    return fft->_max_energy_bin;
}

void DSP::mult_f32(const float* v1, const float* v2, float* vout, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        vout[i] = v1[i] * v2[i];
    }
}

void DSP::max_f32(const float* vin, uint32_t len, float* maxValue, uint32_t* maxIndex)
{
    *maxValue = vin[0];
    *maxIndex = 0;
    for (uint32_t i = 1; i < len; i++) {
        if (vin[i] > *maxValue) {
            *maxValue = vin[i];
            *maxIndex = i;
        }
    }
}

void DSP::scale_f32(const float* vin, float scale, float* vout, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        vout[i] = vin[i] * scale;
    }
}

// simple integer log2
static uint32_t fft_log2(uint32_t n)
{
    uint32_t k = n, i = 0;
    while (k) {
        k >>= 1;
        i++;
    }
    return i - 1;
}

// bitreverse the input
static uint32_t fft_bitreverse(uint32_t length, uint32_t n)
{
    uint32_t p = 0;
    for (uint32_t j = 1; j <= fft_log2(length); j++) {
        if (n & (1 << (fft_log2(length) - j))) {
            p |= 1 << (j - 1);
        }
    }
    return p;
}

// reorder all elements of the input according to their bitreversed index
static void fft_order(complexf *f1, uint32_t length, complexf* temp)
{
    for (uint32_t i = 0; i < length; i++) {
        temp[i] = f1[fft_bitreverse(length, i)];
    }
    for (uint32_t j = 0; j < length; j++) {
        f1[j] = temp[j];
    }
}

// calculate the in-place FFT of the input using the Cooley–Tukey algorithm
void DSP::calculate_fft(complexf *f, uint32_t length)
{
    complexf *temp = new complexf[length];
    fft_order(f, length, temp);

    temp[1] = std::polar(1.0f, -2.0f * M_PI / length);
    temp[0] = 1;
    for (uint32_t i = 2; i < length * 0.5f; i++) {
        temp[i] = std::pow(temp[1], i);
    }
    uint32_t n = 1;
    uint32_t a = length * 0.5f;
    for (uint32_t j = 0; j < log2(length); j++) {
        for (uint32_t i = 0; i < length; i++) {
            if (!(i & n)) {
                complexf t1 = f[i];
                complexf t2 = temp[(i * a) % (n * a)] * f[i + n];
                f[i] = t1 + t2;
                f[i + n] = t1 - t2;
            }
        }
        n *= 2;
        a = a * 0.5f;
    }
    delete[] temp;
}
