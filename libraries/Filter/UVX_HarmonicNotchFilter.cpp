
/// TODO: License description

/// @file UVX_HarmonicNotchFilter.cpp
/// @brief Harmonic notch filter class
#include "UVX_HarmonicNotchFilter.h"

// update the notch filter coefficients
template <class T>
void UVX_HarmonicNotchFilter<T>::update_coeffs()
{
    float temp = 0.002f * M_PI * _notch_freq_hz * _Ts_msec;
    float temp1 = _notch_freq_hz / _rej_bandwidth_hz;

    _c4 = sinf( temp ) * _rej_bandwidth_hz;
    _c3 = cosf( temp ) * _notch_freq_hz;
    _c2 = 0.5f * temp1 * temp1 * _c4;
    _c1 = 1.0f / ( _notch_freq_hz + _c4 );

    // NOTE: Division by zero in the calculation of _c1 is not possible if the notch frequency is smaller than the Nyquist frequency
}

// apply the notch filter on an input sample with variable notch frequency, bandwidth and sample time
template <class T>
T UVX_HarmonicNotchFilter<T>::apply( const T &input, const float notch_freq_hz, const float bandwidth_hz, const float dt )
{
    // update the sample frequency and period
    if ( dt > 0.0f )
    {
        _Ts_msec = 1000.0f * dt;
        _sample_freq_hz = 1.0f / dt;
        _initialized = true;
    }

    // issue filter reset if the notch frequency is below the minimum
    if ( notch_freq_hz < _min_allowed_notch_frequency_hz )
    {
        _flag_reset = true;
    }

    // reset the notch filter upon request
    if ( _flag_reset || !_initialized )
    {
        _state_1 = input;
        _state_2 = input - input;
        _state_3 = input;
        _flag_reset = false;
        
        return input;
    }

    // limit the notch frequency between the minimum allowed and up to the Nyquist frequency
    _notch_freq_hz = constrain_float( notch_freq_hz, _min_allowed_notch_frequency_hz, 0.48f * _sample_freq_hz );

    // update the rejection bandwidth
    _rej_bandwidth_hz = 0.5f * bandwidth_hz;
    
    // update the filter coefficients
    update_coeffs();
    
    // evaluate the notch filter
    T temp = input + _state_3;
    T temp1 = _state_1;

    _state_1 = ( temp1 * ( _c3 + _c4 ) + _state_2 * _c2 + temp * ( 0.5f * ( _notch_freq_hz - _c3 ) ) ) * _c1;
    _state_2 = ( _state_2 * _c3 + ( temp - ( temp1 * 2.0f ) - _state_2 ) * _c4 ) * _c1;
    _state_3 = input;
    
    return ( input - _state_2 );
}

// instantiate template classes
template class UVX_HarmonicNotchFilter<Vector3f>;
template class UVX_HarmonicNotchFilter<float>;
