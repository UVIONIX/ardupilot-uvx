
/// TODO: License description

/// @file UVX_HarmonicNotchFilter.h
/// @brief Harmonic notch filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

// minimum allowed notch frequency
//
constexpr float MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ = 13.0f;

/**
 * A class implementing a notch filter with transfer function W(s) = ( s^2 + w0^2 ) / ( s^2 + 2 * wr * s + w0^2 ), where
 * w0      -> center rejected frequency (notch frequency), [rad/s]
 * 2 * wr  -> bandwidth (from the point -3dB on the left of the notch frequency to the point -3dB on the right of the notch frequency), [rad/s]
 * wr      -> rejection bandwidth (half the bandwidth), [rad/s]
 * The notch frequency and the bandwidth can be updated dynamically in runtime
 */
template <class T>
class UVX_HarmonicNotchFilter
{
public:

    // initialize the filter with the provided parameters
    void init( float sample_freq_hz, float center_freq_hz, float bandwidth_hz );

    // update the sampling period and frequency
    void update_sample_period_and_frequency( float sample_period_sec );

    // set the notch frequency from rpm
    void set_notch_freq_from_rpm( float rpm );

    // get the notch frequency
    float get_notch_freq_Hz() { return _notch_freq_Hz; }

    // set the rejection bandwidth as a function of the notch frequency
    void set_rejection_bandwidth_Hz();

    // set the rejection bandwidth to a specified value
    void set_rejection_bandwidth_Hz(float rej_bandwidth_Hz) { _rej_bandwidth_Hz = rej_bandwidth_Hz; }

    // issue a filter reset
    void reset() { _flag_reset = true; }

    // apply the notch filter on an input sample
    T apply( T input );

    // apply the notch filter on an input sample with variable notch frequency, bandwidth and sample time
    T apply( T input, const float notch_freq_hz, const float bandwidth_hz, const float dt );

private:

    // reset flag
    bool _flag_reset = true;

    // initialized flag
    bool _initialized = false;

    // notch frequency, [Hz]
    float _notch_freq_Hz = 0.0f;

    // rejection bandwidth, [Hz]
    float _rej_bandwidth_Hz = 0.0f;

    // sampling period, [msec]
    float _Ts_msec;

    // sampling frequency, [Hz]
    float _sample_freq_Hz;

    // filter coefficients
    float _c1, _c2, _c3, _c4;

    // state-storage variables
    T _state_1, _state_2, _state_3;

    // update the notch filter coefficients
    void update_coeffs();
};

typedef UVX_HarmonicNotchFilter<Vector3f> UVX_HarmonicNotchFilterVector3f;
typedef UVX_HarmonicNotchFilter<float> UVX_HarmonicNotchFilterFloat;
