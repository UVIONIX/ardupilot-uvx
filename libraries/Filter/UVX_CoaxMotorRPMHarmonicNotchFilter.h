
/// TODO: License description

/// @file UVX_CoaxMotorRPMHarmonicNotchFilter.h
/// @brief Harmonic notch filter class for a coaxial motors configuration
#pragma once

#include <AP_Param/AP_Param.h>
#include "UVX_HarmonicNotchFilter.h"

// maximum allowed number of harmonics per motor in the coaxial configuration
static constexpr uint16_t UVX_MAX_HARMONICS_PER_COAX_MOTOR = 8;

/**
 * @brief Harmonic notch filter class for a coaxial motors configuration
*/
class UVX_CoaxMotorRPMHarmonicNotchFilter
{
    // minimum allowed notch frequency
    static constexpr float MIN_NOTCH_FREQUENCY_HZ = 10.0f;

public:
    // constructor
    UVX_CoaxMotorRPMHarmonicNotchFilter() = default;

    // destructor
    ~UVX_CoaxMotorRPMHarmonicNotchFilter();

    // do not allow copies
    CLASS_NO_COPY(UVX_CoaxMotorRPMHarmonicNotchFilter);

    /**
     * @brief allocate filters
     * @param harmonic_scalars pointer to a float array with scalars of a propeller rotational frequency at which notch filters will be applied
     * @param bandwidth_scalars pointer to a float array with scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
     * @param num_harmonics number of harmonics to use up to UVX_MAX_HARMONICS_PER_COAX_MOTOR. Set to zero to disable the filter
     * @return true if allocation was successful, false otherwise
     * @note if num_harmonics is greater than UVX_MAX_HARMONICS_PER_COAX_MOTOR the first UVX_MAX_HARMONICS_PER_COAX_MOTOR elements of both arrays will be used. Care must
     * be taken to ensure that the harmonic_scalars and bandwidth_scalars arrays are with the same length and with dimension equal to or greater than num_harmonics.
    */
    bool allocate_filters(const float *harmonic_scalars, const float *bandwidth_scalars, const uint16_t num_harmonics);

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
    bool update_params(const float *harmonic_scalars, const float *bandwidth_scalars, const uint16_t num_harmonics);

    /**
     * run the filter on an input 3D vector sample
     * @param input input 3D vector with values (e.g. gyro or accel measurements)
     * @param up_rpm angular velocity of the upper propeller, [RPM]
     * @param lp_rpm angular velocity of the lower propeller, [RPM]
     * @param dt sampling time, [sec]
    */
    Vector3f run(const Vector3f &input, const float up_rpm, const float lp_rpm, const float dt);

    // reset all filters
    void reset();

private:
    // free allocated filters
    void free_filters();

    // update the harmonic and bandwidth scalars
    void update_scalars(const float *harmonic_scalars, const float *bandwidth_scalars);

    // vector of harmonics of a propeller rotational frequency at which notch filters will be applied
    float _harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR]{};

    // vector of scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
    float _bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR]{};

    // number of harmonics per motor
    uint16_t _num_harmonics{0};

    // individual filter instances container
    UVX_HarmonicNotchFilterVector3f* _filters = nullptr;
};

/**
 * @brief Parameters class of the coaxial motors configuration harmonic notch filter
*/
class UVX_CoaxMotorRPMNotchFilterParams
{
public:
    // constructor
    UVX_CoaxMotorRPMNotchFilterParams(void);

    // table of user settable parameters
    static const AP_Param::GroupInfo var_info[];

    // number of harmonics to use up to UVX_MAX_HARMONICS_PER_COAX_MOTOR
    AP_Int8 num_harmonics;

    // vector of harmonics of a propeller rotational frequency at which notch filters will be applied
    AP_Float harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR];

    // vector of scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
    AP_Float bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR];
};
