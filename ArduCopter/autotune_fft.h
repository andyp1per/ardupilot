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

#pragma once

#include "AP_Math/AP_Math.h"
#include "AP_InertialSensor/AP_InertialSensor.h"
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#include <arm_math.h>
#endif

// max for F3 targets
#define FFT_WINDOW_SIZE 32
#define XYZ_AXIS_COUNT 3
#define DYN_NOTCH_RANGE_HZ_HIGH 2000
#define DYN_NOTCH_RANGE_HZ_MEDIUM 1333
#define DYN_NOTCH_RANGE_HZ_LOW 1000

class autotune_fft
{
public:
    autotune_fft();

    enum
    {
        DYN_NOTCH_RANGE_HIGH = 0,
        DYN_NOTCH_RANGE_MEDIUM,
        DYN_NOTCH_RANGE_LOW,
        DYN_NOTCH_RANGE_AUTO
    };

    void init(uint32_t targetLooptime, AP_InertialSensor& ins);
    void push_sample(const Vector3f& sample);
    void analyse();
    void analyse_update();
    void analyse_init(uint32_t targetLooptimeUs);
    Vector3f get_motor_peak() const { return Vector3f(centerFreq[0], centerFreq[1], centerFreq[2]);}

    // a function called by the main thread at the main loop rate:
    void periodic();

    static const struct AP_Param::GroupInfo var_info[];

private:
    // accumulator for oversampled data => no aliasing and less noise
    uint8_t sampleCount;
    uint8_t maxSampleCount;
    float maxSampleCountRcp;

    // downsampled gyro data circular buffer for frequency analysis
    uint8_t circularBufferIdx;
    Vector3f downsampledGyroData[FFT_WINDOW_SIZE];

    // update state machine step information
    uint8_t updateTicks;
    uint8_t updateStep;
    uint8_t updateAxis;
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    arm_rfft_fast_instance_f32 fftInstance;
#endif
    float fftData[FFT_WINDOW_SIZE];
    float rfftData[FFT_WINDOW_SIZE];

    uint16_t centerFreq[XYZ_AXIS_COUNT];
    uint16_t prevCenterFreq[XYZ_AXIS_COUNT];
    LowPassFilter2pFloat detectedFrequencyFilter[XYZ_AXIS_COUNT];

    AP_Int16 fftSamplingRateHz;
    float fftResolution;
    uint8_t fftStartBin;
    uint16_t dynNotchMaxCtrHz;
    uint8_t dynamicFilterRange;
    AP_Int16 dynNotchMinHz;
    uint16_t dynNotchMaxFFT;
    AP_Int8 _enable;
    AP_InertialSensor* _ins;

    // Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
    float hanningWindow[FFT_WINDOW_SIZE];
};
