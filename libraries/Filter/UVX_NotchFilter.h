
/// TODO: License description

/// @file UVX_NotchFilter.h
/// @brief Notch filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

// minimum allowed notch frequency
constexpr float MIN_NOTCH_FILTER_FREQUENCY_HZ = 1.0f;

/**
 * A class implementing a notch filter with transfer function W(s) = ( s^2 + w0^2 ) / ( s^2 + 2 * wr * s + w0^2 ), where
 * w0      -> center rejected frequency (notch frequency), [rad/s]
 * 2 * wr  -> bandwidth (from the point -3dB on the left of the notch frequency to the point -3dB on the right of the notch frequency), [rad/s]
 * wr      -> rejection bandwidth, [rad/s]
 */
template <class T>
class UVX_NotchFilter
{
public:

    // set the notch filter center rejected frequency
    void set_notch_freq_Hz(float nfreq) { _notch_freq_Hz = nfreq; }

    // set the notch filter bandwidth
    void set_bandwidth_Hz(float bw) { _rej_bandwidth_Hz = 0.5f * bw; }

    // issue a filter reset
    void reset() { _flag_reset = true; }

    // apply the notch filter on an input sample
    T apply(T input, float dt);

private:

    // reset flag
    bool _flag_reset = true;

    // notch frequency, [Hz]
    float _notch_freq_Hz;

    // rejection bandwidth, [Hz]
    float _rej_bandwidth_Hz;

    // filter coefficients
    float _c1, _c2, _c3, _c4;

    // state-storage variables
    T _state_1, _state_2, _state_3;

    // update the notch filter coefficients. returns the notch frequency limited up to the Nyquist frequency
    float update_coeffs(float dt);
};

typedef UVX_NotchFilter<Vector2f> UVX_NotchFilterVector2f;
typedef UVX_NotchFilter<Vector3f> UVX_NotchFilterVector3f;
typedef UVX_NotchFilter<float> UVX_NotchFilterFloat;
