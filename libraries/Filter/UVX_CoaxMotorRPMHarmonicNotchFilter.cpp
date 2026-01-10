
/// TODO: License description

/// @file UVX_CoaxMotorRPMHarmonicNotchFilter.cpp
/// @brief Harmonic notch filter class for a coaxial motors configuration

#include "UVX_CoaxMotorRPMHarmonicNotchFilter.h"
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo UVX_CoaxMotorRPMNotchFilterParams::var_info[] = {

    // @Param: HCS
    // @DisplayName: UVX coax RPM notch harmonic count
    // @Description: Number of harmonics to use in the UVX coax RPM notch filter (0 disables). The harmonic scalars _H_ and bandwidth scalars _B_ are taken up this number.
    // @Range: 0 8
    // @User: Advanced
    AP_GROUPINFO("HCS", 1, UVX_CoaxMotorRPMNotchFilterParams, num_harmonics, 4),

    // @Param: H1
    // @DisplayName: UVX coax RPM notch first instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the first notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 1 defines a notch frequency of 30 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H1", 2, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[0], 1),

    // @Param: H2
    // @DisplayName: UVX coax RPM notch second instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the second notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 2 defines a notch frequency of 60 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H2", 3, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[1], 2),

    // @Param: H3
    // @DisplayName: UVX coax RPM notch third instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the third notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 3 defines a notch frequency of 90 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H3", 4, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[2], 3),

    // @Param: H4
    // @DisplayName: UVX coax RPM notch fourth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the fourth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 4 defines a notch frequency of 120 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H4", 5, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[3], 4),

    // @Param: H5
    // @DisplayName: UVX coax RPM notch fifth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the fifth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 5 defines a notch frequency of 150 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H5", 6, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[4], 5),

    // @Param: H6
    // @DisplayName: UVX coax RPM notch sixth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the sixth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 6 defines a notch frequency of 180 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H6", 7, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[5], 6),

    // @Param: H7
    // @DisplayName: UVX coax RPM notch seventh instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the seventh notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 7 defines a notch frequency of 210 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H7", 8, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[6], 7),

    // @Param: H8
    // @DisplayName: UVX coax RPM notch eighth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the eighth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 8 defines a notch frequency of 240 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("H8", 9, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[7], 8),

    // @Param: B1
    // @DisplayName: UVX coax RPM notch first instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the first notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.4 defines a bandwidth of 12 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B1", 10, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[0], 0.4),

    // @Param: B2
    // @DisplayName: UVX coax RPM notch second instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the second notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.4 defines a bandwidth of 12 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B2", 11, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[1], 0.4),

    // @Param: B3
    // @DisplayName: UVX coax RPM notch third instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the third notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B3", 12, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[2], 0.2),

    // @Param: B4
    // @DisplayName: UVX coax RPM notch fourth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the fourth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B4", 13, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[3], 0.2),

    // @Param: B5
    // @DisplayName: UVX coax RPM notch fifth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the fifth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B5", 14, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[4], 0.2),

    // @Param: B6
    // @DisplayName: UVX coax RPM notch sixth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the sixth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B6", 15, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[5], 0.2),

    // @Param: B7
    // @DisplayName: UVX coax RPM notch seventh instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the seventh notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B7", 16, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[6], 0.2),

    // @Param: B8
    // @DisplayName: UVX coax RPM notch eighth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the eighth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("B8", 17, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[7], 0.2),

    AP_GROUPEND
};

// parameters class constructor
UVX_CoaxMotorRPMNotchFilterParams::UVX_CoaxMotorRPMNotchFilterParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// harmonic notch filter class destructor
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
 * @param num_harmonics number of harmonics to use up to UVX_MAX_HARMONICS_PER_COAX_MOTOR. Set to zero to disable the filter
 * @return true if allocation was successful, false otherwise
 * @note if num_harmonics is greater than UVX_MAX_HARMONICS_PER_COAX_MOTOR the first UVX_MAX_HARMONICS_PER_COAX_MOTOR elements of both arrays will be used. Care must
 * be taken to ensure that the harmonic_scalars and bandwidth_scalars arrays are with the same length and with dimension equal to or greater than num_harmonics.
*/
bool UVX_CoaxMotorRPMHarmonicNotchFilter::allocate_filters(const float *harmonic_scalars, const float *bandwidth_scalars, const uint16_t num_harmonics)
{
    // free filters if already allocated
    free_filters();

    // initialize the harmonic and bandwidth scalars
    _num_harmonics = MIN(num_harmonics, UVX_MAX_HARMONICS_PER_COAX_MOTOR);
    if (_num_harmonics == 0)
    {
        return true;
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

    // update filter scalars
    update_scalars(harmonic_scalars, bandwidth_scalars);

    // initial reset of the filters
    reset();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully allocated %d notch instances", nfilters);
    return true;
}

/**
 * @brief update filter parameters in runtime
 * @param harmonic_scalars pointer to a float array with scalars of a propeller rotational frequency at which notch filters will be applied
 * @param bandwidth_scalars pointer to a float array with scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
 * @param num_harmonics number of harmonics to use up to UVX_MAX_HARMONICS_PER_COAX_MOTOR. Set to zero to disable the filter
 * @return true if update was successful, false otherwise
 * @note if num_harmonics is greater than UVX_MAX_HARMONICS_PER_COAX_MOTOR the first UVX_MAX_HARMONICS_PER_COAX_MOTOR elements of both arrays will be used. Care must
 * be taken to ensure that the harmonic_scalars and bandwidth_scalars arrays are with the same length and with dimension equal to or greater than num_harmonics. Filters
 * will be re-allocated if the number of harmonics has changed. Otherwise, only the harmonic and bandwidth scalars are updated
*/
bool UVX_CoaxMotorRPMHarmonicNotchFilter::update_params(const float *harmonic_scalars, const float *bandwidth_scalars, const uint16_t num_harmonics)
{
    // check if filters have to be re-allocated
    if (num_harmonics == _num_harmonics)
    {
        // no change in filters count
        if (_num_harmonics == 0)
        {
            return true;
        }

        bool params_changed = false;
        for (uint16_t i = 0; i < _num_harmonics; i++)
        {
            if (fabsf(_harmonic_scalars[i] - harmonic_scalars[i]) > 1e-6f || fabsf(_bandwidth_scalars[i] - bandwidth_scalars[i]) > 1e-6f)
            {
                params_changed = true;
                break;
            }
        }

        if (!params_changed)
        {
            return false;
        }

        // update filter scalars. Do not reset the filters to prevent continuous filters resets during continuous runtime parameter updates
        update_scalars(harmonic_scalars, bandwidth_scalars);
        return true;
    }

    // re-allocate the filters
    return allocate_filters(harmonic_scalars, bandwidth_scalars, num_harmonics);
}

// reset all filters
void UVX_CoaxMotorRPMHarmonicNotchFilter::reset()
{
    // exit if filter allocation has failed or the filter is disabled
    if (_filters == nullptr || !_num_harmonics)
    {
        return;
    }

    uint16_t nfilters = 2 * _num_harmonics;
    for (uint16_t i = 0; i < nfilters; ++i)
    {
        _filters[i].reset();
        _filters[i].set_min_allowed_notch_frequency_hz(MIN_NOTCH_FREQUENCY_HZ);
    }
}

// update the harmonic and bandwidth scalars
void UVX_CoaxMotorRPMHarmonicNotchFilter::update_scalars(const float *harmonic_scalars, const float *bandwidth_scalars)
{
    for (uint16_t i = 0; i < _num_harmonics; i++)
    {
        _harmonic_scalars[i] = harmonic_scalars[i];
        _bandwidth_scalars[i] = bandwidth_scalars[i];
    }
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
    // exit if filter allocation has failed or the filter is disabled
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
