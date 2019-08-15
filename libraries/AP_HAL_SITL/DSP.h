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
#include "AP_HAL_SITL_Namespace.h"

#include <complex>

typedef std::complex<float> complexf;

// ChibiOS implementation of FFT analysis to run on STM32 processors
class HALSITL::DSP : public AP_HAL::DSP {
public:
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) override;
    // start an FFT analysis
    virtual void fft_start(FFTWindowState* state, const float* samples, uint16_t buffer_index) override;
    // perform remaining steps of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin) override;

    // STM32-based FFT state
    class FFTWindowStateSITL : public AP_HAL::DSP::FFTWindowState {
        friend class HALSITL::DSP;

    protected:
        FFTWindowStateSITL(uint16_t window_size, uint16_t sample_rate);
        ~FFTWindowStateSITL();

    private:
        // bin with maximum energy
        uint32_t _max_energy_bin;
        // intermediate real FFT data
        float* _rfft_data;
    };

private:
    void step_hanning(FFTWindowStateSITL* fft, const float* samples, uint16_t buffer_index);
    void step_fft(FFTWindowStateSITL* fft);
    void step_cmplx_mag_f32(FFTWindowStateSITL* fft, uint16_t start_bin, uint16_t end_bin);
    uint16_t step_calc_frequencies(FFTWindowStateSITL* fft, uint16_t start_bin, uint16_t end_bin);
    void mult_f32(const float* v1, const float* v2, float* vout, uint32_t len);
    void max_f32(const float* vin, uint32_t len, float* maxValue, uint32_t* maxIndex);
    void scale_f32(const float* vin, float scale, float* vout, uint32_t len);
    void calculate_fft(complexf* f, uint32_t length);
};
