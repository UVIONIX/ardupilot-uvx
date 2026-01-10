
/// TODO: License description

/// @file	UVX_InertialSensor.cpp
/// @brief	This file defines some custom methods declared in the file AP_InertialSensor.h

#include "AP_InertialSensor.h"
#include <GCS_MAVLink/GCS.h>

#if ENABLE_UVX_COAX_RPM_NOTCH_FILTERS
// allocate the RPM notch filters for the gyro and accel measurements
void AP_InertialSensor::allocate_uvx_coax_rpm_notch_filters()
{
    bool gyro_rpm_notches_ok;
    bool accel_rpm_notches_ok;
    float harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};
    float bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};

    // get the required number of harmonics
    const uint8_t num_harmonics = constrain_int16(_uvx_rpm_notch_params.num_harmonics.get(), 0, UVX_MAX_HARMONICS_PER_COAX_MOTOR);

    // initialize the harmonic and bandwidth scalars
    for (uint8_t i = 0; i < num_harmonics; i++)
    {
        harmonic_scalars[i] = _uvx_rpm_notch_params.harmonic_scalars[i].get();
        bandwidth_scalars[i] = _uvx_rpm_notch_params.bandwidth_scalars[i].get();
    }

    // allocate the filters
    for (uint8_t i = 0; i < get_gyro_count(); i++)
    {
        if (_use(i))
        {
            gyro_rpm_notches_ok = _uvx_gyro_rpm_notches[i].allocate_filters(harmonic_scalars, bandwidth_scalars, num_harmonics);
            accel_rpm_notches_ok = _uvx_accel_rpm_notches[i].allocate_filters(harmonic_scalars, bandwidth_scalars, num_harmonics);

            if (!gyro_rpm_notches_ok)
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to allocate gyro RPM notch for instance %d", i);
            else
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully allocated gyro RPM notch for instance %d", i);
            
            if (!accel_rpm_notches_ok)
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to allocate accel RPM notch for instance %d", i);
            else
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully allocated accel RPM notch for instance %d", i);
        }
    }
}

// update the parameters of the gyro rpm notch filter
void AP_InertialSensor::update_uvx_gyro_rpm_notch_params(uint8_t instance)
{
    // exit if instance is not being used
    if (!_use(instance))
        return;

    // throttle to 1Hz
    const uint32_t now = AP_HAL::millis();

    if (now - _uvx_gyro_rpm_notch_last_params_update_timestamp_ms[instance] < 1000)
    {
        return;
    }

    _uvx_gyro_rpm_notch_last_params_update_timestamp_ms[instance] = now;

    // get the required number of harmonics
    const uint8_t num_harmonics = constrain_int16(_uvx_rpm_notch_params.num_harmonics.get(), 0, UVX_MAX_HARMONICS_PER_COAX_MOTOR);

    // initialize the harmonic and bandwidth scalars
    float harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};
    float bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};
    for (uint8_t i = 0; i < num_harmonics; i++)
    {
        harmonic_scalars[i] = _uvx_rpm_notch_params.harmonic_scalars[i].get();
        bandwidth_scalars[i] = _uvx_rpm_notch_params.bandwidth_scalars[i].get();
    }

    // update gyro notch parameters
    bool ok = _uvx_gyro_rpm_notches[instance].update_params(harmonic_scalars, bandwidth_scalars, num_harmonics);

    if (ok)
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully updated gyro %d UVX notch params", instance);
}

// update the parameters of the accel rpm notch filter
void AP_InertialSensor::update_uvx_accel_rpm_notch_params(uint8_t instance)
{
    // exit if instance is not being used
    if (!_use(instance))
        return;

    // throttle to 1Hz
    const uint32_t now = AP_HAL::millis();

    if (now - _uvx_accel_rpm_notch_last_params_update_timestamp_ms[instance] < 1000)
    {
        return;
    }

    _uvx_accel_rpm_notch_last_params_update_timestamp_ms[instance] = now;

    // get the required number of harmonics
    const uint8_t num_harmonics = constrain_int16(_uvx_rpm_notch_params.num_harmonics.get(), 0, UVX_MAX_HARMONICS_PER_COAX_MOTOR);

    // initialize the harmonic and bandwidth scalars
    float harmonic_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};
    float bandwidth_scalars[UVX_MAX_HARMONICS_PER_COAX_MOTOR] = {};
    for (uint8_t i = 0; i < num_harmonics; i++)
    {
        harmonic_scalars[i] = _uvx_rpm_notch_params.harmonic_scalars[i].get();
        bandwidth_scalars[i] = _uvx_rpm_notch_params.bandwidth_scalars[i].get();
    }

    // update accel notch parameters
    bool ok = _uvx_accel_rpm_notches[instance].update_params(harmonic_scalars, bandwidth_scalars, num_harmonics);

    if (ok)
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully updated accel %d UVX notch params", instance);
}
#endif // ENABLE_UVX_COAX_RPM_NOTCH_FILTERS
