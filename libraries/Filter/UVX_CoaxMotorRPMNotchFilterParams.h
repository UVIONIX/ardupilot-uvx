
/// TODO: License description

/// @file UVX_CoaxMotorRPMNotchFilterParams.h
/// @brief Parameters class for the coaxial motors configuration harmonic notch filter
#pragma once

#include <AP_Param/AP_Param.h>

// maximum allowed number of harmonics per motor in the coaxial configuration
static constexpr uint16_t UVX_MAX_HARMONICS_PER_COAX_MOTOR = 8;

/**
 * @brief Struct containing the parameters of the coaxial motors configuration harmonic notch filter
*/
struct UVX_CoaxMotorRPMNotchFilterParams
{
    // number of harmonics to use up to UVX_MAX_HARMONICS_PER_COAX_MOTOR
    AP_Int8 num_harmonics;

    // vector of harmonics of a propeller rotational frequency at which notch filters will be applied
    AP_Float harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR];

    // vector of scalars of a propeller rotational frequency, defining the filter bandwith at the respective frequency
    AP_Float bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR];

    static const AP_Param::GroupInfo var_info[];
};
