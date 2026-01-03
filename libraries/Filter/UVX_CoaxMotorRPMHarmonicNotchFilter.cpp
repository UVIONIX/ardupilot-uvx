
/// TODO: License description

/// @file UVX_CoaxMotorRPMHarmonicNotchFilter.cpp
/// @brief Harmonic notch filter class for a coaxial motors configuration

#include "UVX_CoaxMotorRPMHarmonicNotchFilter.h"

// destructor
UVX_CoaxMotorRPMHarmonicNotchFilter::~UVX_CoaxMotorRPMHarmonicNotchFilter()
{
    free_filters();
}

// free allocated filters
void UVX_CoaxMotorRPMHarmonicNotchFilter::free_filters()
{
    delete[] _filters;
    _filters = nullptr;
    _num_harmonics = 0;
}

/**
 * @brief allocate filters
 * @param harmonic_scalars pointer to a float array with scalars of a propeller rotational frequency at which notch filters will be applied
 * @param bandwidth_scalars pointer to a float array with scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
 * @param num_harmonics number of harmonics as the size of the provided scalar arrays
 * @note if num_harmonics is greater than MAX_HARMONICS_PER_COAX_MOTOR the first MAX_HARMONICS_PER_COAX_MOTOR elements of both arrays will be used
*/
bool UVX_CoaxMotorRPMHarmonicNotchFilter::allocate_filters(const float *harmonic_scalars, const float *bandwidth_scalars, const uint16_t num_harmonics)
{
    // free filters if already allocated
    free_filters();

    // initialize the harmonic and bandwidth scalars
    _num_harmonics = MIN(num_harmonics, MAX_HARMONICS_PER_COAX_MOTOR);
    if (_num_harmonics == 0)
    {
        return false;
    }

    for (uint16_t i = 0; i < _num_harmonics; i++)
    {
        _harmonic_scalars[i] = harmonic_scalars[i];
        _bandwidth_scalars[i] = bandwidth_scalars[i];
    }

    // allocate the filters
    const uint16_t nfilters = 2 * _num_harmonics;
    _filters = NEW_NOTHROW UVX_HarmonicNotchFilterVector3f[nfilters];

    if (_filters == nullptr)
    {
        // allocation has failed
        _num_harmonics = 0;
        return false;
    }

    // initial reset of the filters
    for (uint16_t i = 0; i < nfilters; ++i)
    {
        _filters[i].reset();
        _filters[i].set_min_allowed_notch_frequency_hz(MIN_NOTCH_FREQUENCY_HZ);
    }

    return true;
}

/**
 * run the filter on an input 3D vector sample
 * @param input input 3D vector with values (e.g. gyro or accel measurements)
 * @param up_rpm angular velocity of the upper propeller, [RPM]
 * @param lp_rpm angular velocity of the lower propeller, [RPM]
 * @param dt sampling time, [sec]
*/
Vector3f UVX_CoaxMotorRPMHarmonicNotchFilter::run(const Vector3f &input, const float up_rpm, const float lp_rpm, const float dt)
{
    // exit if filter allocation has failed
    if (_filters == nullptr || !_num_harmonics)
    {
        return input;
    }

    // calculate the base notch frequencies
    float upper_notch_freq_hz = up_rpm / 60.0;
    float lower_notch_freq_hz = lp_rpm / 60.0;

    // run the notch filters
    uint16_t nfilters = 2 * _num_harmonics;
    Vector3f out = input;
    for (uint16_t i = 0; i < nfilters; ++i)
    {
        // get the base frequency as that of the upper or lower motor
        float base_freq = (i < _num_harmonics ? upper_notch_freq_hz : lower_notch_freq_hz);

        // calculate the notch frequency as base frequency times the harmonic scalar
        float nf = base_freq * _harmonic_scalars[i % _num_harmonics];

        // calculate the bandwidth as base frequency times the bandwidth scalar
        float bw = base_freq * _bandwidth_scalars[i % _num_harmonics];

        // run an individual filter instance
        out = _filters[i].apply(out, nf, bw, dt);
    }

    return out;
}
