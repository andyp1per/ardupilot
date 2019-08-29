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

DSP::FFTWindowState::FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps)
    : _update_steps(update_steps),
    _window_size(window_size),
    _bin_count(window_size / 2),
    _bin_resolution((float)sample_rate / (float)window_size)
{
    // includes DC ad Nyquist components and needs to be large enough for intermediate steps
    _freq_bins = new float[window_size];
}

DSP::FFTWindowState::~FFTWindowState()
{
    delete[] _freq_bins;
}
