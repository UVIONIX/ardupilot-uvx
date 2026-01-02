
/// TODO: License description

/// @file UVX_FirstOrderLPF.cpp
/// @brief First order low-pass filter class
#include "UVX_FirstOrderLPF.h"

// update the filter coefficients
template <class T>
void UVX_FirstOrderLPF<T>::update_coeffs()
{
	if ( is_zero( _bandwidth_kHz ) && is_zero( _time_const_msec ) )
	{
		// the specified tuning parameters are invalid -> reset the filter
		reset();
	}
	else
	{
		if (!is_zero(_critical_freq_kHz))
		{
			// apply pre-warping
			if (!is_zero(_bandwidth_kHz))
			{
				// use the specified filter bandwidth
				float TFms = 1.0f / (2.0f * M_PI * _bandwidth_kHz);
				float temp = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
				_c1 = (2.0f * temp) / (2.0f * M_PI * _critical_freq_kHz * TFms + temp);
			}
			else
			{
				// use the specified filter time constant
				float temp = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
				_c1 = (2.0f * temp) / (2.0f * M_PI * _critical_freq_kHz * _time_const_msec + temp);
			}
		}
		else
		{
			// no pre-warping
			if (!is_zero(_bandwidth_kHz))
			{
				// use the specified filter bandwidth
				float TFms = 1.0f / (2.0f * M_PI * _bandwidth_kHz);
				_c1 = (2.0f * _Ts_msec) / (2.0f * TFms + _Ts_msec);
			}
			else
			{
				// use the specified filter time constant
				_c1 = (2.0f * _Ts_msec) / (2.0f * _time_const_msec + _Ts_msec);
			}
		}
	}
}

// apply the filter on an input sample
template <class T>
T UVX_FirstOrderLPF<T>::apply( T input, float dt )
{
	// update the sample time
	update_sample_time_sec(dt);

	// update the filter coefficients
	update_coeffs();

	if ( _flag_reset )
	{
		_state_1 = input;
		_state_2 = input;
		_flag_reset = false;
		return input;
	}

	// run the filter on the input sample
	_state_1 += ( ( ( ( input + _state_2 ) * 0.5f ) - _state_1 ) * _c1 );
	_state_2 = input;

	return _state_1;
}

// instantiate template classes
template class UVX_FirstOrderLPF<float>;
template class UVX_FirstOrderLPF<Vector2f>;
