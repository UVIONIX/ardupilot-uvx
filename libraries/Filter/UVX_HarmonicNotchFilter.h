
/// TODO: License description

/// @file UVX_HarmonicNotchFilter.h
/// @brief Harmonic notch filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

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
    /**
     * set the minimum allowed notch frequency
     * @note the default minimum notch frequency is 10 Hz. Below that frequency the filter is effectively turned-off and returns the provided input 
    */
    void set_min_allowed_notch_frequency_hz( float min_notch_hz ) { _min_allowed_notch_frequency_hz = min_notch_hz; }

    // get the notch frequency
    float get_notch_freq_hz() { return _notch_freq_hz; }

    // issue a filter reset
    void reset() { _flag_reset = true; }

    // apply the notch filter on an input sample with variable notch frequency, bandwidth and sample time
    T apply( const T &input, const float notch_freq_hz, const float bandwidth_hz, const float dt );

private:
    // update the notch filter coefficients
    void update_coeffs();

    // reset flag
    bool _flag_reset = true;

    // initialized flag
    bool _initialized = false;

    // notch frequency, [Hz]
    float _notch_freq_hz = 0.0f;

    // rejection bandwidth, [Hz]
    float _rej_bandwidth_hz = 0.0f;

    // minimum allowed notch frequency, [Hz]
    float _min_allowed_notch_frequency_hz = 10.0f;

    // sampling period, [msec]
    float _Ts_msec;

    // sampling frequency, [Hz]
    float _sample_freq_hz;

    // filter coefficients
    float _c1, _c2, _c3, _c4;

    // state-storage variables
    T _state_1, _state_2, _state_3;
};

typedef UVX_HarmonicNotchFilter<Vector3f> UVX_HarmonicNotchFilterVector3f;
typedef UVX_HarmonicNotchFilter<float> UVX_HarmonicNotchFilterFloat;
