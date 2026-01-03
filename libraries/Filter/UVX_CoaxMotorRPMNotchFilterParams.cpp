
/// TODO: License description

/// @file UVX_CoaxMotorRPMNotchFilterParams.cpp
/// @brief Parameters class for the coaxial motors configuration harmonic notch filter

#include "UVX_CoaxMotorRPMNotchFilterParams.h"

const AP_Param::GroupInfo UVX_CoaxMotorRPMNotchFilterParams::var_info[] = {

    // @Param: _HMNCS
    // @DisplayName: UVX coax RPM notch harmonic count
    // @Description: Number of harmonics to use in the UVX coax RPM notch filter (0 disables). The harmonic scalars _H_ and bandwidth scalars _B_ are taken up this number.
    // @Range: 0 8
    // @User: Advanced
    AP_GROUPINFO("_HMNCS", 1, UVX_CoaxMotorRPMNotchFilterParams, num_harmonics, 4),

    // @Param: _H1
    // @DisplayName: UVX coax RPM notch first instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the first notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 1 defines a notch frequency of 30 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H1", 2, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[0], 1.0f),

    // @Param: _H2
    // @DisplayName: UVX coax RPM notch second instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the second notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 2 defines a notch frequency of 60 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H2", 3, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[1], 2.0f),

    // @Param: _H3
    // @DisplayName: UVX coax RPM notch third instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the third notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 3 defines a notch frequency of 90 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H3", 4, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[2], 3.0f),

    // @Param: _H4
    // @DisplayName: UVX coax RPM notch fourth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the fourth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 4 defines a notch frequency of 120 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H4", 5, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[3], 4.0f),

    // @Param: _H5
    // @DisplayName: UVX coax RPM notch fifth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the fifth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 5 defines a notch frequency of 150 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H5", 6, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[4], 5.0f),

    // @Param: _H6
    // @DisplayName: UVX coax RPM notch sixth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the sixth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 6 defines a notch frequency of 180 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H6", 7, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[5], 6.0f),

    // @Param: _H7
    // @DisplayName: UVX coax RPM notch seventh instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the seventh notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 7 defines a notch frequency of 210 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H7", 8, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[6], 7.0f),

    // @Param: _H8
    // @DisplayName: UVX coax RPM notch eighth instance harmonic scalar
    // @Description: Scalar of a propeller rotational frequency for the eighth notch filter instance, defining its notch frequency (e.g. motor_rpm = 1800 and scalar = 8 defines a notch frequency of 240 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_H8", 9, UVX_CoaxMotorRPMNotchFilterParams, harmonic_scalars[7], 8.0f),

    // @Param: _B1
    // @DisplayName: UVX coax RPM notch first instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the first notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.4 defines a bandwidth of 12 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B1", 10, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[0], 0.4f),

    // @Param: _B2
    // @DisplayName: UVX coax RPM notch second instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the second notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.4 defines a bandwidth of 12 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B2", 11, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[1], 0.4f),

    // @Param: _B3
    // @DisplayName: UVX coax RPM notch third instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the third notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B3", 12, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[2], 0.2f),

    // @Param: _B4
    // @DisplayName: UVX coax RPM notch fourth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the fourth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B4", 13, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[3], 0.2f),

    // @Param: _B5
    // @DisplayName: UVX coax RPM notch fifth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the fifth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B5", 14, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[4], 0.2f),

    // @Param: _B6
    // @DisplayName: UVX coax RPM notch sixth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the sixth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B6", 15, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[5], 0.2f),

    // @Param: _B7
    // @DisplayName: UVX coax RPM notch seventh instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the seventh notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B7", 16, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[6], 0.2f),

    // @Param: _B8
    // @DisplayName: UVX coax RPM notch eighth instance bandwidth scalar
    // @Description: Scalar of a propeller rotational frequency for the eighth notch filter instance, defining its bandwidth (e.g. motor_rpm = 1800 and scalar = 0.2 defines a bandwidth of 6 Hz for that instance)
    // @User: Advanced
    AP_GROUPINFO("_B8", 17, UVX_CoaxMotorRPMNotchFilterParams, bandwidth_scalars[7], 0.2f),

    AP_GROUPEND
};
