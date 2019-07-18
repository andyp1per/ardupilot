/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 */
#include <stdint.h>
#include "autotune_fft.h"

// table of user settable parameters
const AP_Param::GroupInfo autotune_fft::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, autotune_fft, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: Sampling Frequency
    // @Description: Sampling Frequency in Hz
    // @Range: 10 2000
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SAMPLE_FREQ", 2, autotune_fft, fftSamplingRateHz, 1000),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Lower bound of notch center frequency in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 3, autotune_fft, dynNotchMinHz, 80),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// Eg [0,31), [31,62), [62, 93) etc
// for gyro loop >= 4KHz, sample rate 2000 defines FFT range to 1000Hz, 16 bins each 62.5 Hz wide
// NB  FFT_WINDOW_SIZE is set to 32 in gyroanalyse.h
#define FFT_BIN_COUNT (FFT_WINDOW_SIZE / 2)
// smoothing frequency for FFT centre frequency
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS (XYZ_AXIS_COUNT * 4)

#define DYN_NOTCH_OSD_MIN_THROTTLE 20

autotune_fft::autotune_fft()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void autotune_fft::init(uint32_t targetLooptimeUs, AP_InertialSensor& ins)
{
    _ins = &ins;
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);

    fftSamplingRateHz = MIN((gyroLoopRateHz / 3), fftSamplingRateHz);

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE;

    fftStartBin = dynNotchMinHz / lrintf(fftResolution);

    dynNotchMaxCtrHz = fftSamplingRateHz / 2; //Nyquist

    for (int i = 0; i < FFT_WINDOW_SIZE; i++)
    {
        hanningWindow[i] = (0.5f - 0.5f * cosf(2 * M_PI * i / (FFT_WINDOW_SIZE - 1)));
    }

    if (_enable) {
        analyse_init(targetLooptimeUs);
    }
}

    // a function called by the main thread at the main loop rate:
void autotune_fft::periodic() 
{
    if (!_enable) {
        return;
    }
    push_sample(_ins->get_raw_gyro());
}

void autotune_fft::analyse_init(uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    // *** can this next line be removed ??? ***
    //data_analyse_init(targetLooptimeUs);

    const uint16_t samplingFrequency = 1000000 / targetLooptimeUs;
    maxSampleCount = samplingFrequency / fftSamplingRateHz;
    maxSampleCountRcp = 1.f / maxSampleCount;
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);
#endif
    //    recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
    //    at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
    //    for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    //const float looptime = MAX(1000000u / fftSamplingRateHz, targetLooptimeUs * DYN_NOTCH_CALC_TICKS);
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        // any init value
        centerFreq[axis] = dynNotchMaxCtrHz;
        prevCenterFreq[axis] = dynNotchMaxCtrHz;
    }
}

void autotune_fft::push_sample(const Vector3f& sample)
{
    oversampledGyroAccumulator += sample;
}

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void autotune_fft::analyse()
{
    if (!_enable) {
        return;
    }
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    sampleCount++;

    // this runs at 1kHz
    if (sampleCount == maxSampleCount)
    {
        sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
        {
            Vector3f sample = oversampledGyroAccumulator * maxSampleCountRcp;
            downsampledGyroData[circularBufferIdx] = sample;
            oversampledGyroAccumulator.zero();
        }

        circularBufferIdx = (circularBufferIdx + 1) % FFT_WINDOW_SIZE;

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and update filters
    if (updateTicks > 0)
    {
        analyse_update();
        --updateTicks;
    }
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

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
void autotune_fft::analyse_update()
{
    enum
    {
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_HANNING,
        STEP_COUNT
    };
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_cfft_instance_f32 *Sint = &(fftInstance.Sint);
#endif
    //uint32_t startTime = 0;

    switch (updateStep)
    {
    case STEP_ARM_CFFT_F32:
    {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        switch (FFT_BIN_COUNT)
        {
        case 16:
            // 16us
            arm_cfft_radix8by2_f32(Sint, fftData);
            break;
        case 32:
            // 35us
            arm_cfft_radix8by4_f32(Sint, fftData);
            break;
        case 64:
            // 70us
            arm_radix8_butterfly_f32(fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
            break;
        }
#endif
        break;
    }
    case STEP_BITREVERSAL:
    {
        // 6us
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        arm_bitreversal_32((uint32_t *)fftData, Sint->bitRevLength, Sint->pBitRevTable);
#endif
        updateStep++;
        FALLTHROUGH;
    }
    case STEP_STAGE_RFFT_F32:
    {
        // 14us
        // this does not work in place => fftData AND rfftData needed
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        stage_rfft_f32(&fftInstance, fftData, rfftData);
#endif
        break;
    }
    case STEP_ARM_CMPLX_MAG_F32:
    {
        // 8us
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        arm_cmplx_mag_f32(rfftData, fftData, FFT_BIN_COUNT);
#endif
        updateStep++;
        FALLTHROUGH;
    }
    case STEP_CALC_FREQUENCIES:
    {
        bool fftIncreased = false;
        float dataMax = 0;
        uint8_t binStart = 0;
        uint8_t binMax = 0;
        //for bins after initial decline, identify start bin and max bin
        for (int i = fftStartBin; i < FFT_BIN_COUNT; i++)
        {
            if (fftIncreased || (fftData[i] > fftData[i - 1]))
            {
                if (!fftIncreased)
                {
                    binStart = i; // first up-step bin
                    fftIncreased = true;
                }
                if (fftData[i] > dataMax)
                {
                    dataMax = fftData[i];
                    binMax = i; // tallest bin
                }
            }
        }
        // accumulate fftSum and fftWeightedSum from peak bin, and shoulder bins either side of peak
        float cubedData = fftData[binMax] * fftData[binMax] * fftData[binMax];
        float fftSum = cubedData;
        float fftWeightedSum = cubedData * (binMax + 1);
        // accumulate upper shoulder
        for (int i = binMax; i < FFT_BIN_COUNT - 1; i++)
        {
            if (fftData[i] > fftData[i + 1])
            {
                cubedData = fftData[i] * fftData[i] * fftData[i];
                fftSum += cubedData;
                fftWeightedSum += cubedData * (i + 1);
            }
            else
            {
                break;
            }
        }
        // accumulate lower shoulder

        for (int i = binMax; i > binStart + 1; i--)
        {
            if (fftData[i] > fftData[i - 1])
            {
                cubedData = fftData[i] * fftData[i] * fftData[i];
                fftSum += cubedData;
                fftWeightedSum += cubedData * (i + 1);
            }
            else
            {
                break;
            }
        }
        // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
        float weightedCenterFreq = dynNotchMaxCtrHz;
        float fftMeanIndex = 0;
        // idx was shifted by 1 to start at 1, not 0
        if (fftSum > 0)
        {
            fftMeanIndex = (fftWeightedSum / fftSum) - 1;
            // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
            weightedCenterFreq = fftMeanIndex * fftResolution;
        }
        else
        {
            weightedCenterFreq = prevCenterFreq[updateAxis];
        }
        weightedCenterFreq = MAX(weightedCenterFreq, (float)dynNotchMinHz);
        //centerFreq = biquadFilterApply(&detectedFrequencyFilter[updateAxis], centerFreq);
        prevCenterFreq[updateAxis] = centerFreq[updateAxis];
        centerFreq[updateAxis] = weightedCenterFreq;

        //if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
        //    dynNotchMaxFFT = MAX(dynNotchMaxFFT, centerFreq[updateAxis]);
        //}

        break;
    }
    case STEP_UPDATE_FILTERS:
    {
        // 7us
        // calculate cutoffFreq and notch Q, update notch filter  =1.8+((A2-150)*0.004)
        if (prevCenterFreq[updateAxis] != centerFreq[updateAxis])
        {
            //
        }

        updateAxis = (updateAxis + 1) % XYZ_AXIS_COUNT;
        updateStep++;
        FALLTHROUGH;
    }
    case STEP_HANNING:
    {
        // 5us
        // apply hanning window to gyro samples and store result in fftData
        // hanning starts and ends with 0, could be skipped for minor speed improvement
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        const uint8_t ringBufIdx = FFT_WINDOW_SIZE - circularBufferIdx;
        arm_mult_f32(&downsampledGyroData[updateAxis][circularBufferIdx], &hanningWindow[0], &fftData[0], ringBufIdx);
        if (circularBufferIdx > 0)
        {
            arm_mult_f32(&downsampledGyroData[updateAxis][0], &hanningWindow[ringBufIdx], &fftData[ringBufIdx], circularBufferIdx);
        }
#endif
    }
    }

    updateStep = (updateStep + 1) % STEP_COUNT;
}
