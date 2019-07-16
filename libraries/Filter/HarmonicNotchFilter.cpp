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

    // @Param: FREQ
    // @DisplayName: Base Frequency
    // @Description: Notch base center frequency in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FREQ", 2, HarmonicNotchFilterParams, _center_freq_hz, 80),

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
    // @Description: Bitmask of harmonic frequencies to add to the filter fundamental frequency. This option takes effect on the next reboot.
    // @Bitmask: 0:2nd harmonic,1:3rd harmonic,2:4th harmonic,3:5th hamronic,0:6th harmonic,0:7th harmonic,0:8th harmonic
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HMNCS", 5, HarmonicNotchFilterParams, _harmonics, 1),

    // @Param: REF
    // @DisplayName: Reference value
    // @Description: Reference value associated with the specified frequency to facilitate frequency scaling
    // @User: Advanced
    // @Range: 0.1 0.9
    // @RebootRequired: True
    AP_GROUPINFO("REF", 6, HarmonicNotchFilterParams, _reference, 0.1f),

    AP_GROUPEND
};


/*
  initialise filter
 */
template <class T>
void HarmonicNotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    if (_filters == nullptr) {
        return;
    }

    _sample_freq_hz = sample_freq_hz;

    // adjust the center frequency to be in the allowable range
    center_freq_hz = constrain_float(center_freq_hz, bandwidth_hz * 0.52f, sample_freq_hz * 0.48f);

    NotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, _A, _Q);

    for (uint8_t i=0; i<HNF_MAX_FILTERS; i++) {
        if ((1U<<i) & _harmonics) {
            _filters[i].internal_init(sample_freq_hz, center_freq_hz * (i+1), _A, _Q);
        }
    }
    _initialised = true;
}

/*
  initialise filter
 */
template <class T>
void HarmonicNotchFilter<T>::create(uint8_t harmonics)
{
    _num_filters = 1;
    for (uint8_t i = 0; i < HNF_MAX_FILTERS-1; i++) {
        if ((1U<<i) & harmonics) {
            _num_filters++;
        }
    }
    _filters = new NotchFilter<T>[_num_filters];
    _harmonics = harmonics;
}

template <class T>
void HarmonicNotchFilter<T>::update(float center_freq_hz)
{
    if (!_initialised) {
        return;
    }

    // adjust the center frequency to be in the allowable range
    center_freq_hz = constrain_float(center_freq_hz, 1.0f, _sample_freq_hz * 0.48f);

    for (uint8_t i=0, filt=0; i<HNF_MAX_FILTERS; i++) {
        if (i==0 || (1U<<i) & _harmonics) {
            _filters[filt++].internal_init(_sample_freq_hz, center_freq_hz * (i+1), _A, _Q);
        }
    }
}

template <class T>
T HarmonicNotchFilter<T>::apply(const T &sample)
{
    if (!_initialised) {
        return sample;
    }

    T output = _filters[0].apply(sample);
    for (uint8_t i=1; i<_num_filters; i++) {
        output = _filters[i].apply(output);
    }
    return output;
}

template <class T>
void HarmonicNotchFilter<T>::reset()
{
    if (!_initialised) {
        return;
    }

    for (uint8_t i=0; i<_num_filters+1; i++) {
        _filters[i].reset();
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
