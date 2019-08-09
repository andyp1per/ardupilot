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
/*
  interface to DSP device
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::DSP {
public:
    typedef float* FFTSampleWindow;
    class FFTWindowState {
    public:
        // number of cycles to complete a FFT
        const uint8_t _update_steps;
        // frequency width of a FFT bin
        const float _bin_resolution;
        // number of FFT bins
        const uint16_t _bin_count;
        // size of the FFT window
        const uint16_t _window_size;
        // FFT data
        float* _freq_bins;
        // Estimate of FFT peak frequency
        float _max_bin_freq;
        // Hanning window for incoming samples, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float* _hanning_window;
        // Use in calculating the PS of the signal [Heinz] equations (20) & (21)
        float _window_scale;

        virtual void reset() = 0;
        virtual ~FFTWindowState();
        FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps);
    };
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) = 0;
    // perform one step of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin) = 0;

protected:
        // quinn's frequency interpolator
    float calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k) const;
    float tau(const float x) const;

private:
};
