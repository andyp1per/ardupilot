/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "HarmonicNotchFilter.h"

// table of user settable parameters
const AP_Param::GroupInfo HarmonicNotchFilterParams::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable harmonic notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, HarmonicNotchFilterParams, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: BASEHZ
    // @DisplayName: Base Frequency
    // @Description: Notch base center frequency in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("BASEHZ", 2, HarmonicNotchFilterParams, _center_freq_hz, 80),

    // @Param: BW
    // @DisplayName: Bandwidth
    // @Description: Harmonic notch bandwidth in Hz
    // @Range: 5 100
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("BW", 3, HarmonicNotchFilterParams, _bandwidth_hz, 20),

    // @Param: ATT
    // @DisplayName: Attenuation
    // @Description: Harmonic notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("ATT", 4, HarmonicNotchFilterParams, _attenuation_dB, 15),

    // @Param: HMNCS
    // @DisplayName: Harmonics
    // @Description: Number of harmonic frequencies to add to the filter. This option takes effect on the next reboot.
    // @Range: 0 2
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HMNCS", 5, HarmonicNotchFilterParams, _harmonics, 1),

    AP_GROUPEND
};


/*
  initialise filter
 */
template <class T>
void HarmonicNotchFilter<T>::init(float _sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    if (filters == nullptr) {
        return;
    }

    sample_freq_hz = _sample_freq_hz;

    // adjust the center frequency to be in the allowable range
    center_freq_hz = constrain_float(center_freq_hz, bandwidth_hz * 0.52f, sample_freq_hz * 0.48f);

    NotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);

    for (int i=0; i<harmonics+1; i++) {
        filters[i].internal_init(sample_freq_hz, center_freq_hz * (i+1), A, Q);
    }
    initialised = true;
}

/*
  initialise filter
 */
template <class T>
void HarmonicNotchFilter<T>::create(uint8_t _harmonics)
{
    filters = new NotchFilter<T>[_harmonics+1];
    harmonics = _harmonics;
}

template <class T>
void HarmonicNotchFilter<T>::update(float center_freq_hz)
{
    if (!initialised) {
        return;
    }

    // adjust the center frequency to be in the allowable range
    center_freq_hz = constrain_float(center_freq_hz, 1.0f, sample_freq_hz * 0.48f);

    for (int i=0; i<harmonics+1; i++) {
        filters[i].internal_init(sample_freq_hz, center_freq_hz * (i+1), A, Q);
    }
}

template <class T>
T HarmonicNotchFilter<T>::apply(const T &sample)
{
    if (!initialised) {
        return sample;
    }

    T output = filters[0].apply(sample);
    for (int i=1; i<harmonics+1; i++) {
        output = filters[i].apply(output);
    }
    return output;
}

template <class T>
void HarmonicNotchFilter<T>::reset()
{
    if (!initialised) {
        return;
    }

    for (int i=0; i<harmonics+1; i++) {
        filters[i].reset();
    }
}


/*
  a notch filter with enable and filter parameters - constructor
 */
HarmonicNotchFilterParams::HarmonicNotchFilterParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);    
}

/* 
   instantiate template classes
 */
template class HarmonicNotchFilter<float>;
template class HarmonicNotchFilter<Vector3f>;
