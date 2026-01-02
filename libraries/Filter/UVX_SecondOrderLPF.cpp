
/// TODO: License description

/// @file UVX_SecondOrderLPF.cpp
/// @brief Second order low-pass filter class
#include "UVX_SecondOrderLPF.h"

// update the filter coefficients
void UVX_SecondOrderLPF::update_coeffs()
{
	if (is_zero(_bandwidth_kHz) && is_zero(_time_const_msec))
	{
		// the specified tuning parameters are invalid -> reset the filter
		reset();
	}
	else
	{
		if (_Ts_msec > 0.0f)
			_sample_freq_Hz = 1000.0f / _Ts_msec;

		if (!is_zero(_critical_freq_kHz))
		{
			// apply pre-warping
			if (!is_zero(_bandwidth_kHz))
			{
				// use the specified values for the damping factor and the filter bandwidth
				float temp1 = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
				float temp2 = _critical_freq_kHz / _bandwidth_kHz;
				float temp3 = 1.0f / (temp2 * temp2 + temp1 * (2.0f * temp2 * _damping + temp1));
				float temp = temp1 * temp3;

				_c1 = 0.001f * temp2 * temp / (M_PI * _bandwidth_kHz);
				_c2 = 2.0f * temp1 * temp;
				_c3 = temp2 * temp2 * temp3 - (2.0f * temp2 * _damping + temp1) * temp;
				_c4 = 4000.0f * M_PI * _critical_freq_kHz * temp;
			}
			else
			{
				// use the specified values for the damping factor and the filter time constant
				float Bandwidth_kHz = 1.0f / (2.0f * M_PI * _time_const_msec);
				float temp1 = tanf(M_PI * _critical_freq_kHz * _Ts_msec);
				float temp2 = _critical_freq_kHz / Bandwidth_kHz;
				float temp3 = 1.0f / (temp2 * temp2 + temp1 * (2.0f * temp2 * _damping + temp1));
				float temp = temp1 * temp3;

				_c1 = 0.001f * temp2 * temp / (M_PI * Bandwidth_kHz);
				_c2 = 2.0f * temp1 * temp;
				_c3 = temp2 * temp2 * temp3 - (2.0f * temp2 * _damping + temp1) * temp;
				_c4 = 4000.0f * M_PI * _critical_freq_kHz * temp;
			}
		}
		else
		{
			// no pre-warping 
			if (!is_zero(_bandwidth_kHz))
			{
				// use the specified values for the damping factor and the filter bandwidth
				float temp1 = M_PI * _Ts_msec * _bandwidth_kHz;
				float temp = 2.0f / (1.0f + temp1 * (temp1 + 2.0f * _damping));

				_c1 = 0.0005f * _Ts_msec * temp;
				_c2 = temp1 * temp1 * temp;
				_c3 = temp - 1.0f;
				_c4 = 2000.0f * M_PI * _bandwidth_kHz * temp1 * temp;
			}
			else
			{
				// use the specified values for the damping factor and the filter time constant
				float Bandwidth_kHz = 1.0f / (2.0f * M_PI * _time_const_msec);
				float temp1 = M_PI * _Ts_msec * Bandwidth_kHz;
				float temp = 2.0f / (1.0f + temp1 * (temp1 + 2.0f * _damping));

				_c1 = 0.0005f * _Ts_msec * temp;
				_c2 = temp1 * temp1 * temp;
				_c3 = temp - 1.0f;
				_c4 = 2000.0f * M_PI * Bandwidth_kHz * temp1 * temp;
			}
		}	
	}
}

// reset the filter states to the provided values and store the current filter input 
void UVX_SecondOrderLPF::reset(float out, float out_dot, float input)
{
    _state_1 = out;
    _state_2 = out_dot;
    _state_3 = input;
}

// apply the filter on an input sample
float UVX_SecondOrderLPF::apply(float input, float dt)
{
	// update the sample time
	update_sample_time_sec(dt);

	// update the filter coefficients
	update_coeffs();

	if (_flag_reset)
	{
		_state_1 = input;
		_state_2 = 0.0f;
		_state_3 = input;
		_flag_reset = false;

		return input;
	}

	if (_flag_bypass)
	{
		_state_1 = input;
		_state_2 = (input - _state_3) * _sample_freq_Hz;
		_state_3 = input;

		return input;
	}

	// run the filter on an input sample
	float temp = 0.5f * (input + _state_3) - _state_1;
	_state_1 += _c1 * _state_2 + _c2 * temp;
	_state_2 = _c3 * _state_2 + _c4 * temp;
	_state_3 = input;

	return _state_1;
}
