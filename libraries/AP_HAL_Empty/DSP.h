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
#pragma once

#include "AP_HAL_Empty.h"

class Empty::DSP : public AP_HAL::DSP {
public:
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) override { return nullptr; };
    virtual uint16_t fft_analyse(FFTWindowState* state, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin) override { return 0; };
};
