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

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include <arm_math.h>

class ChibiOS::DSP : public AP_HAL::DSP {
public:
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) override;
    virtual uint16_t fft_analyse(FFTWindowState* state, const FFTSampleWindow* samples, uint16_t buffer_index, uint16_t start_bin) override;

    enum {
        STEP_HANNING,
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_COUNT
    };

public:
    class FFTWindowStateARM : public AP_HAL::DSP::FFTWindowState {
        friend class ChibiOS::DSP;

    protected:
        ~FFTWindowStateARM();
        FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps);

    private:
       // update state machine step information
        uint8_t _update_step = STEP_HANNING;
        arm_rfft_fast_instance_f32 _fft_instance;
        float* _fft_data;
        float* _rfft_data;

        // Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float* _hanning_window;
    };

};