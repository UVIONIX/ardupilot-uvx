
/// TODO: License description

/// @file UVX_FirstOrderDifferentiator.cpp
/// @brief First order differentiator filter class
#include "UVX_FirstOrderDifferentiator.h"

// update the differentiator coefficients
template <class T>
void UVX_FirstOrderDifferentiator<T>::update_coeffs()
{
    if (is_zero(_bandwidth_kHz) && is_zero(_time_const_msec))
    {
        // the specified tuning parameters are invalid -> reset the filter
        reset();
    }
    else
    {
        if (!is_zero(_critical_freq_kHz))
        {
            // apply prewarping
            if (!is_zero(_bandwidth_kHz))
            {
                // use the specified filter bandwidth
                float TFms = 1.0f / (2.0f * M_PI * _bandwidth_kHz);
                float temp = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
                _c1 = (2000.0f * M_PI * _critical_freq_kHz) / (2.0f * M_PI * TFms * _critical_freq_kHz + temp);
                _c2 = (0.001f * temp) / (M_PI * _critical_freq_kHz);
            }
            else
            {
                // use the specified filter time constant
                float temp = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
                _c1 = (2000.0f * M_PI * _critical_freq_kHz) / (2.0f * M_PI * _time_const_msec * _critical_freq_kHz + temp);
                _c2 = (0.001f * temp) / (M_PI * _critical_freq_kHz);
            }
        }
        else
        {
            // no prewarping
            if (!is_zero(_bandwidth_kHz))
            {
                // use the specified filter bandwidth
                float TFms = 1.0f / (2.0f * M_PI * _bandwidth_kHz);
                _c1 = 2000.0f / (2.0f * TFms + _Ts_msec);
                _c2 = 0.001f * _Ts_msec;
            }
            else
            {
                // use the specified filter time constant
                _c1 = 2000.0f / (2.0f * _time_const_msec + _Ts_msec);
                _c2 = 0.001f * _Ts_msec;
            }
        }
    }
}

// reset the filter states to the provided values and store the current filter input
template <class T>
void UVX_FirstOrderDifferentiator<T>::reset(T out, T input)
{
    _state_1 = out;
    _state_2 = input;
}

// apply the differentiator on an input sample
template <class T>
T UVX_FirstOrderDifferentiator<T>::apply(T input, float dt)
{
    // update the sample time
    update_sample_time_sec(dt);

    // update the filter coefficients
    update_coeffs();

    if (_flag_reset)
    {
        _state_1 = input - input;
        _state_2 = input;
        _flag_reset = false;
        return _state_1;
    }

    // run the differentiator on the input sample
    _state_1 += ( ( (input - _state_2) - (_state_1 * _c2) ) * _c1 );
    _state_2 = input;

    return _state_1;
}

// instantiate template classes
template class UVX_FirstOrderDifferentiator<float>;
template class UVX_FirstOrderDifferentiator<Vector2f>;
