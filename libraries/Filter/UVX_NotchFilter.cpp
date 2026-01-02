
/// TODO: License description

/// @file UVX_NotchFilter.cpp
/// @brief Notch filter class
#include "UVX_NotchFilter.h"

// update the notch filter coefficients. returns the notch frequency limited up to the Nyquist frequency
template <class T>
float UVX_NotchFilter<T>::update_coeffs(float dt)
{
    // limit the notch frequency up to the Nyquist frequency
    float notch_freq = constrain_float(_notch_freq_Hz, MIN_NOTCH_FILTER_FREQUENCY_HZ, 0.5f / dt);

    float temp = 2.0f * M_PI * notch_freq * dt;
    float temp1 = notch_freq / _rej_bandwidth_Hz;

    _c4 = sinf(temp) * _rej_bandwidth_Hz;
    _c3 = cosf(temp) * notch_freq;
    _c2 = 0.5f * temp1 * temp1 * _c4;
    _c1 = 1.0f / (notch_freq + _c4);

    // NOTE: Division by zero in the calculation of _c1 is not possible if the notch frequency is smaller than the Nyquist frequency
    return notch_freq;
}

// apply the notch filter on an input sample
template <class T>
T UVX_NotchFilter<T>::apply(T input, float dt)
{
    // reset the notch filter upon request
    if (_flag_reset)
    {
        _state_1 = input;
        _state_2 = input - input;
        _state_3 = input;
        _flag_reset = false;

        return input;
    }

    // update the filter coefficients
    float notch_freq = update_coeffs(dt);

    // evaluate the notch filter
    T temp = input + _state_3;
    T temp1 = _state_1;

    _state_1 = (temp1 * (_c3 + _c4) + _state_2 * _c2 + temp * (0.5f * (notch_freq - _c3))) * _c1;
    _state_2 = (_state_2 * _c3 + (temp - (temp1 * 2.0f) - _state_2) * _c4) * _c1;
    _state_3 = input;

    return (input - _state_2);
}

// instantiate template classes
template class UVX_NotchFilter<Vector2f>;
template class UVX_NotchFilter<Vector3f>;
template class UVX_NotchFilter<float>;
