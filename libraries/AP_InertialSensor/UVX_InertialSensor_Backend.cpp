
/// TODO: License description

/// @file	UVX_InertialSensor_Backend.cpp
/// @brief	This file overwrites some methods in the file AP_InertialSensor_Backend.cpp

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_InertialSensor_Backend.h"

const extern AP_HAL::HAL& hal;

#ifdef ENABLE_UVX_COAX_RPM_NOTCH_FILTERS
void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro_raw,
                                                            uint64_t sample_us)
{
    if (has_been_killed(instance)) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_gyro_count[instance], _imu._sample_gyro_start_us[instance],
                        _imu._gyro_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._gyro_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _imu._gyro_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._gyro_last_sample_us[instance]) * 1.0e-6f;
        _imu._gyro_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 40Hz
        if (_imu._gyro_raw_sample_rates[instance] < 40) {
            return;
        }

        dt = 1.0f / _imu._gyro_raw_sample_rates[instance];
        _imu._gyro_last_sample_us[instance] = AP_HAL::micros64();
        sample_us = _imu._gyro_last_sample_us[instance];
    }

    Vector3f gyro = gyro_raw;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        // apply motor RPM-based harmonic notch filters
        gyro = _imu._uvx_gyro_rpm_notches[instance].run(gyro, _imu._motor_upper_rpm, _imu._motor_lower_rpm, dt);
        if (gyro.is_nan() || gyro.is_inf())
        {
            gyro = gyro_raw;
            _imu._uvx_gyro_rpm_notches[instance].reset();
        }
    
        // compute delta angle
        Vector3f delta_angle = (gyro + _imu._last_raw_gyro[instance]) * 0.5f * dt;

        // compute coning correction
        // see page 26 of:
        // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
        // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
        // see also examples/coning.py
        Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                                 _imu._last_delta_angle[instance] * (1.0f / 6.0f));
        delta_coning = delta_coning % delta_angle;
        delta_coning *= 0.5f;

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_angle_acc[instance].zero();
            _imu._delta_angle_acc_dt[instance] = 0;
            dt = 0;
            delta_angle.zero();
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

        // apply gyro filters
        apply_gyro_filters(instance, gyro);

        _imu._new_gyro_data[instance] = true;
    }

    // push gyros if optical flow present
    if (hal.opticalflow) {
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    }

    // 5us
    log_gyro_raw(instance, sample_us, gyro_raw, _imu._gyro_filtered[instance]);
    update_primary();
}

/*
  handle a delta-angle sample from the backend. This assumes FIFO
  style sampling and the sample should not be rotated or corrected for
  offsets.
  This function should be used when the sensor driver can directly
  provide delta-angle values from the sensor.
 */
void AP_InertialSensor_Backend::_notify_new_delta_angle(uint8_t instance, const Vector3f &dangle)
{
    if (has_been_killed(instance)) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_gyro_count[instance], _imu._sample_gyro_start_us[instance],
                        _imu._gyro_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._gyro_last_sample_us[instance];

    // don't accept below 40Hz
    if (_imu._gyro_raw_sample_rates[instance] < 40) {
        return;
    }

    dt = 1.0f / _imu._gyro_raw_sample_rates[instance];
    _imu._gyro_last_sample_us[instance] = AP_HAL::micros64();
    uint64_t sample_us = _imu._gyro_last_sample_us[instance];

    Vector3f gyro_raw = dangle / dt;

    _rotate_and_correct_gyro(instance, gyro_raw);

    Vector3f gyro = gyro_raw;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        // apply motor RPM-based harmonic notch filters
        gyro = _imu._uvx_gyro_rpm_notches[instance].run(gyro, _imu._motor_upper_rpm, _imu._motor_lower_rpm, dt);
        if (gyro.is_nan() || gyro.is_inf())
        {
            gyro = gyro_raw;
            _imu._uvx_gyro_rpm_notches[instance].reset();
        }

        // compute delta angle
        Vector3f delta_angle = gyro * dt;

        // compute coning correction
        // see page 26 of:
        // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
        // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
        // see also examples/coning.py
        Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                                 _imu._last_delta_angle[instance] * (1.0f / 6.0f));
        delta_coning = delta_coning % delta_angle;
        delta_coning *= 0.5f;

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_angle_acc[instance].zero();
            _imu._delta_angle_acc_dt[instance] = 0;
            dt = 0;
            delta_angle.zero();
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

        // apply gyro filters and sample for FFT
        apply_gyro_filters(instance, gyro);

        _imu._new_gyro_data[instance] = true;
    }

    // push gyros if optical flow present
    if (hal.opticalflow) {
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    }

    log_gyro_raw(instance, sample_us, gyro_raw, _imu._gyro_filtered[instance]);
    update_primary();
}

void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel_raw,
                                                             uint64_t sample_us,
                                                             bool fsync_set)
{
    if (has_been_killed(instance)) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_accel_count[instance], _imu._sample_accel_start_us[instance],
                        _imu._accel_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._accel_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _imu._accel_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._accel_last_sample_us[instance]) * 1.0e-6f;
        _imu._accel_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 40Hz
        if (_imu._accel_raw_sample_rates[instance] < 40) {
            return;
        }

        dt = 1.0f / _imu._accel_raw_sample_rates[instance];
        _imu._accel_last_sample_us[instance] = AP_HAL::micros64();
        sample_us = _imu._accel_last_sample_us[instance];
    }

    Vector3f accel = accel_raw;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        // apply motor RPM-based harmonic notch filters
        accel = _imu._uvx_accel_rpm_notches[instance].run(accel, _imu._motor_upper_rpm, _imu._motor_lower_rpm, dt);
        if (accel.is_nan() || accel.is_inf())
        {
            accel = accel_raw;
            _imu._uvx_accel_rpm_notches[instance].reset();
        }

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_velocity_acc[instance].zero();
            _imu._delta_velocity_acc_dt[instance] = 0;
            dt = 0;
        }
        
        // delta velocity
        _imu._delta_velocity_acc[instance] += accel * dt;
        _imu._delta_velocity_acc_dt[instance] += dt;

        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

        _imu._new_accel_data[instance] = true;
    }

    _imu.calc_vibration_and_clipping(instance, accel, dt);

    // 5us
    log_accel_raw(instance, sample_us, accel_raw, _imu._accel_filtered[instance]);
}

/*
  handle a delta-velocity sample from the backend. This assumes FIFO style sampling and
  the sample should not be rotated or corrected for offsets

  This function should be used when the sensor driver can directly
  provide delta-velocity values from the sensor.
 */
void AP_InertialSensor_Backend::_notify_new_delta_velocity(uint8_t instance, const Vector3f &dvel)
{
    if (has_been_killed(instance)) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_accel_count[instance], _imu._sample_accel_start_us[instance],
                        _imu._accel_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._accel_last_sample_us[instance];

    // don't accept below 40Hz
    if (_imu._accel_raw_sample_rates[instance] < 40) {
        return;
    }

    dt = 1.0f / _imu._accel_raw_sample_rates[instance];
    _imu._accel_last_sample_us[instance] = AP_HAL::micros64();
    uint64_t sample_us = _imu._accel_last_sample_us[instance];

    Vector3f accel_raw = dvel / dt;

    _rotate_and_correct_accel(instance, accel_raw);

    Vector3f accel = accel_raw;
    
    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        // apply motor RPM-based harmonic notch filters
        accel = _imu._uvx_accel_rpm_notches[instance].run(accel, _imu._motor_upper_rpm, _imu._motor_lower_rpm, dt);
        if (accel.is_nan() || accel.is_inf())
        {
            accel = accel_raw;
            _imu._uvx_accel_rpm_notches[instance].reset();
        }

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_velocity_acc[instance].zero();
            _imu._delta_velocity_acc_dt[instance] = 0;
            dt = 0;
        }
        
        // delta velocity
        _imu._delta_velocity_acc[instance] += accel * dt;
        _imu._delta_velocity_acc_dt[instance] += dt;

        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

        _imu._new_accel_data[instance] = true;
    }

    _imu.calc_vibration_and_clipping(instance, accel, dt);

    // 5us
    log_accel_raw(instance, sample_us, accel_raw, _imu._accel_filtered[instance]);
}

void AP_InertialSensor_Backend::log_accel_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &raw_accel, const Vector3f &filtered_accel)
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }

#if AP_AHRS_ENABLED
    const bool log_because_primary_accel = _imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::PRIMARY_GYRO_ONLY) && (instance == _imu._primary);
#else
    const bool log_because_primary_accel = false;
#endif

    if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::ALL_GYROS) ||
        log_because_primary_accel ||
        should_log_imu_raw()) {

        if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::PRE_AND_POST_FILTER)) {
            // Both pre and post, offset post instance as batch sampler does
            Write_ACC(instance, sample_us, raw_accel);
            Write_ACC(instance + _imu._accel_count, sample_us, filtered_accel);

        } else if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::POST_FILTER)) {
            // Just post
            Write_ACC(instance, sample_us, filtered_accel);

        } else {
            // Just pre
            Write_ACC(instance, sample_us, raw_accel);

        }
    } else {
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
        if (!_imu.batchsampler.doing_sensor_rate_logging()) {
            _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_ACCEL, sample_us,
                                     !_imu.batchsampler.doing_post_filter_logging() ? raw_accel : filtered_accel);
        }
#endif
    }
#endif
}

#endif // ENABLE_UVX_COAX_RPM_NOTCH_FILTERS
