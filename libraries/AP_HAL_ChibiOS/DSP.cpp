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
#include <AP_HAL/AP_HAL.h>
#include "DSP.h"
#include "AP_HAL_ChibiOS.h"

using namespace ChibiOS;



AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate) 
{
    return new DSP::FFTWindowStateARM(window_size, sample_rate, STEP_COUNT);
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
    void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
    void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);
}
#endif

uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, const AP_HAL::DSP::FFTSampleWindow* samples, uint16_t buffer_index, uint16_t start_bin)
{
    FFTWindowStateARM* fft = (FFTWindowStateARM*)state;
    uint32_t bin_max = 0;

    arm_cfft_instance_f32 *Sint = &(fft->_fft_instance.Sint);
    Sint->fftLen = fft->_fft_instance.fftLenRFFT / 2;

    switch (fft->_update_step) {
    case STEP_HANNING: {
        // 5us
        // apply hanning window to gyro samples and store result in _fft_data
        // hanning starts and ends with 0, could be skipped for minor speed improvement
        const uint8_t ringBufIdx = fft->_window_size - buffer_index;
        arm_mult_f32((float32_t*)&samples[buffer_index], &fft->_hanning_window[0], &fft->_fft_data[0], ringBufIdx);
        if (buffer_index > 0) {
            arm_mult_f32((float32_t*)&samples[0], &fft->_hanning_window[ringBufIdx], &fft->_fft_data[ringBufIdx], buffer_index);
        }
        fft->_update_step++;
        FALLTHROUGH;
    }
    case STEP_ARM_CFFT_F32: {
        switch (fft->_bin_count) {
        case 16:
        case 128:
            // 16us
            arm_cfft_radix8by2_f32(Sint, fft->_fft_data);
            break;
        case 32:
        case 256:
            // 35us
            arm_cfft_radix8by4_f32(Sint, fft->_fft_data);
            break;
        case 64:
        case 512:
            // 70us
            arm_radix8_butterfly_f32(fft->_fft_data, fft->_bin_count, Sint->pTwiddle, 1);
            break;
        }
        break;
    }
    case STEP_BITREVERSAL: {
        // 6us
        arm_bitreversal_32((uint32_t *)fft->_fft_data, Sint->bitRevLength, Sint->pBitRevTable);
        fft->_update_step++;
        FALLTHROUGH;
    }
    case STEP_STAGE_RFFT_F32: {
        // 14us
        // this does not work in place => _fft_data AND _rfft_data needed
        stage_rfft_f32(&fft->_fft_instance, fft->_fft_data, fft->_rfft_data);
        fft->_update_step++;
        FALLTHROUGH;
        // break;
    }
    case STEP_ARM_CMPLX_MAG_F32: {
        // 8us
        // General case for the magnitudes - see https://stackoverflow.com/questions/42299932/dsp-libraries-rfft-strange-results
        // The frequency of each of those frequency components are given by k*fs/N
        arm_cmplx_mag_f32(fft->_rfft_data + 2, fft->_fft_data + 1, fft->_bin_count-1);
        fft->_fft_data[0] = fft->_rfft_data[0]; // DC
        fft->_fft_data[fft->_bin_count] = fft->_rfft_data[1]; // Nyquist
        fft->_fft_data[fft->_bin_count + 1] = 0; // So that interpolation works correctly
        fft->_update_step++;
        FALLTHROUGH;
    }
    case STEP_CALC_FREQUENCIES: {
        float maxValue = 0;
        arm_max_f32(fft->_fft_data + start_bin, (fft->_bin_count + 1) - start_bin, &maxValue, &bin_max);
        bin_max += start_bin;

        fft->_update_step++;
        FALLTHROUGH;
        // break;
    }
    }
    fft->_update_step = (fft->_update_step + 1) % STEP_COUNT;

    return bin_max;
}
    
DSP::FFTWindowStateARM::FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t update_steps)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate, update_steps)
{
    arm_rfft_fast_init_f32(&_fft_instance, _window_size);

    _fft_data = new float[_window_size];
    _rfft_data = new float[_window_size];
    _hanning_window = new float[_window_size];

    for (uint16_t i = 0; i < _window_size; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2 * M_PI * i / (_window_size - 1)));
    }
}

DSP::FFTWindowStateARM::~FFTWindowStateARM()
{
    delete[] _fft_data;
    delete[] _rfft_data;
    delete[] _hanning_window;
}
