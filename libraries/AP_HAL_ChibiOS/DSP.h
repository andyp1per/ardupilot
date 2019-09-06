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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"

#if HAL_WITH_DSP

#include <arm_math.h>

#define DEBUG_FFT   0

// ChibiOS implementation of FFT analysis to run on STM32 processors
class ChibiOS::DSP : public AP_HAL::DSP {
public:
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) override;
    // perform one step of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin) override;

    // the stages of the FFT state machine
    enum {
        STEP_HANNING,
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_COUNT
    };

    // STM32-based FFT state
    class FFTWindowStateARM : public AP_HAL::DSP::FFTWindowState {
        friend class ChibiOS::DSP;

    protected:
        FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps);
        ~FFTWindowStateARM();
        // reset the state to the first step
        virtual void reset() override { _update_step = STEP_HANNING; }

    private:
       // current update state machine step
        uint8_t _update_step = STEP_HANNING;
        // bin with maximum energy
        uint32_t _max_energy_bin;
        // underlying CMSIS data structure for FFT analysis
        arm_rfft_fast_instance_f32 _fft_instance;
        // intermediate real FFT data
        float* _rfft_data;
        // Hanning window for incoming samples, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float* _hanning_window;
        // Use in calculating the PS of the signal [Heinz] equations (20) & (21)
        float _window_scale;
    };

private:
    // 2 step FFT analysis for short windows and fast processors
    uint16_t fft_analyse_window_2step(FFTWindowStateARM* fft, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin);
    // 3 step FFT analysis for longer windows and slower processors
    uint16_t fft_analyse_window_3step(FFTWindowStateARM* fft, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin);
    // 6 step FFT analysis for the largest windows and slowest processors
    uint16_t fft_analyse_window_6step(FFTWindowStateARM* fft, const float* samples, uint16_t buffer_index, uint16_t start_bin, uint16_t end_bin);
    // following are the six independent steps for calculating an FFT
    void step_hanning(FFTWindowStateARM* fft, const float* samples, uint16_t buffer_index);
    void step_arm_cfft_f32(FFTWindowStateARM* fft);
    void step_bitreversal(FFTWindowStateARM* fft);
    void step_stage_rfft_f32(FFTWindowStateARM* fft);
    void step_arm_cmplx_mag_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin);
    uint16_t step_calc_frequencies(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin);
    // candan's frequency interpolator
    float calculate_candans_estimator(FFTWindowStateARM* fft, uint8_t k);
    // quinn's frequency interpolator
    float calculate_quinns_second_estimator(FFTWindowStateARM* fft, uint8_t k);
    float tau(float x);

#if DEBUG_FFT
    class StepTimer {
    public:
        uint32_t _timer_total;
        uint32_t _timer_avg;
        uint8_t _time_ticks;

        void time(uint32_t start);
    };

    uint32_t  _output_count;
    StepTimer _arm_cfft_f32_timer;
    StepTimer _bitreversal_timer;
    StepTimer _stage_rfft_f32_timer;
    StepTimer _arm_cmplx_mag_f32_timer;
    StepTimer _step_calc_frequencies;
#endif
};

#endif