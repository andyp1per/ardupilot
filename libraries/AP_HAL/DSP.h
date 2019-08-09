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
        const uint8_t _update_steps;
        const float _bin_resolution;
        const uint16_t _bin_count;
        const uint16_t _window_size;
        float* _freq_bins;
        ~FFTWindowState();
        FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps);
    };
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) = 0;
    virtual uint16_t fft_analyse(FFTWindowState* state, const FFTSampleWindow* samples, uint16_t buffer_index, uint16_t start_bin) = 0;
private:
};
