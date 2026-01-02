
/// TODO: License description

/// @file UVX_HarmonicNotchFilter.cpp
/// @brief Harmonic notch filter class
#include "UVX_HarmonicNotchFilter.h"

// initialize the filter with the provided parameters
template <class T>
void UVX_HarmonicNotchFilter<T>::init( float sample_freq_hz, float center_freq_hz, float bandwidth_hz )
{
    // set the filter sampling period
    _Ts_msec = 1000 / sample_freq_hz;

    // save the sampling frequency
    _sample_freq_Hz = sample_freq_hz;

    // set the rejection bandwidth
    _rej_bandwidth_Hz = constrain_float( 0.5f * bandwidth_hz, 0.25f * MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ, 0.25f * 0.48f * sample_freq_hz );

    // set the notch frequency
    _notch_freq_Hz = constrain_float( center_freq_hz, MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ, 0.48f * sample_freq_hz );

    // the filter has been initialized
    _initialized = true;

    // initial reset of the filter
    _flag_reset = true;
}

// update the sampling period and frequency
template <class T>
void UVX_HarmonicNotchFilter<T>::update_sample_period_and_frequency( float sample_period_sec )
{
    if ( sample_period_sec > 0.0f )
    {
        _Ts_msec = 1000.0f * sample_period_sec;
        _sample_freq_Hz = 1.0f / sample_period_sec;
        _initialized = true;
    }
}

// set the notch frequency from rpm
template <class T>
void UVX_HarmonicNotchFilter<T>::set_notch_freq_from_rpm( float rpm )
{
    if ( _initialized )
    {
        // calculate rpm in Hz
        rpm = rpm / 60.0f;

        // limit the notch frequency up to the Nyquist frequency
        if ( rpm > 0.48f * _sample_freq_Hz )
            rpm = 0.48f * _sample_freq_Hz;

        // limit the notch frequency down to the specified minimum frequency
        if ( rpm < MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ )
            _notch_freq_Hz = MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ;
        else
            _notch_freq_Hz = rpm;
    }
}

// set the rejection bandwidth as a function of the notch frequency
template <class T>
void UVX_HarmonicNotchFilter<T>::set_rejection_bandwidth_Hz()
{
    if ( _initialized )
    {
        // currently the filter bandwidth is set to half the notch frequency and hence the rejection bandwidth is a quarter of the notch frequency
        _rej_bandwidth_Hz = _notch_freq_Hz * 0.25f;
    }
}

// update the notch filter coefficients
template <class T>
void UVX_HarmonicNotchFilter<T>::update_coeffs()
{
    float temp = 0.002f * M_PI * _notch_freq_Hz * _Ts_msec;
    float temp1 = _notch_freq_Hz / _rej_bandwidth_Hz;

    _c4 = sinf( temp ) * _rej_bandwidth_Hz;
    _c3 = cosf( temp ) * _notch_freq_Hz;
    _c2 = 0.5f * temp1 * temp1 * _c4;
    _c1 = 1.0f / ( _notch_freq_Hz + _c4 );

    // NOTE: Division by zero in the calculation of _c1 is not possible if the notch frequency is smaller than the Nyquist frequency
}

// apply the notch filter on an input sample
template <class T>
T UVX_HarmonicNotchFilter<T>::apply( T input )
{
    // reset the notch filter upon request
    if ( _flag_reset || !_initialized )
    {
        _state_1 = input;
        _state_2 = input - input;
        _state_3 = input;
        _flag_reset = false;

        return input;
    }

    // update the filter coefficients
    update_coeffs();

    // evaluate the notch filter
    T temp = input + _state_3;
    T temp1 = _state_1;

    _state_1 = ( temp1 * ( _c3 + _c4 ) + _state_2 * _c2 + temp * ( 0.5f * ( _notch_freq_Hz - _c3 ) ) ) * _c1;
    _state_2 = ( _state_2 * _c3 + ( temp - ( temp1 * 2.0f ) - _state_2 ) * _c4 ) * _c1;
    _state_3 = input;
    
    return ( input - _state_2 );
}

// apply the notch filter on an input sample with variable notch frequency, bandwidth and sample time
template <class T>
T UVX_HarmonicNotchFilter<T>::apply( T input, const float notch_freq_hz, const float bandwidth_hz, const float dt )
{
    // reset the notch filter upon request
    if ( _flag_reset || !_initialized )
    {
        _state_1 = input;
        _state_2 = input - input;
        _state_3 = input;
        _flag_reset = false;
        
        return input;
    }

    // update the sample frequency and period
    if ( dt > 0.0f )
    {
        _Ts_msec = 1000.0f * dt;
        _sample_freq_Hz = 1.0f / dt;
    }

    // limit the notch frequency up to the Nyquist frequency
    _notch_freq_Hz = constrain_float( notch_freq_hz, MIN_HMNC_NOTCH_FILTER_FREQUENCY_HZ, 0.48f * _sample_freq_Hz );

    // update the rejection bandwidth
    _rej_bandwidth_Hz = 0.5f * bandwidth_hz;
    
    // update the filter coefficients
    update_coeffs();
    
    // evaluate the notch filter
    T temp = input + _state_3;
    T temp1 = _state_1;

    _state_1 = ( temp1 * ( _c3 + _c4 ) + _state_2 * _c2 + temp * ( 0.5f * ( _notch_freq_Hz - _c3 ) ) ) * _c1;
    _state_2 = ( _state_2 * _c3 + ( temp - ( temp1 * 2.0f ) - _state_2 ) * _c4 ) * _c1;
    _state_3 = input;
    
    return ( input - _state_2 );
}

// instantiate template classes
template class UVX_HarmonicNotchFilter<Vector3f>;
template class UVX_HarmonicNotchFilter<float>;
