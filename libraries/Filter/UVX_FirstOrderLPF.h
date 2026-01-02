
/// TODO: License description

/// @file UVX_FirstOrderLPF.h
/// @brief First order low-pass filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

/**
 * A class implementing a first order low-pass filter with transfer function W(s) = 1 / ( TF * s + 1 ), where
 * TF  -> filter time constant, [s]
 * The filter can be tuned by specifying either the desired filter time constant in seconds or filter bandwidth in Hz. 
 * If a critical frequency in Hz is set a pre-warping is used for calculating the filter coefficients. The discrete-time
 * implementation of the filter is according to the BLT algorithm.
 * The appropriate functions set_crit_freq_Hz, set_bandwidth_Hz or set_time_const_sec should be called during filter
 * initialization or in runtime.
 */
template <class T>
class UVX_FirstOrderLPF
{
public:

	// set critical frequency
	void set_crit_freq_Hz( float crit_freq_Hz ) { _critical_freq_kHz = crit_freq_Hz * 0.001f; }

	// set the filter bandwidth
	void set_bandwidth_Hz( float bandwidth_Hz ) { _bandwidth_kHz = bandwidth_Hz * 0.001f; }

	// set the filter time constant
	void set_time_const_sec( float time_const_sec ) { _time_const_msec = time_const_sec * 1000; }

	// issue a filter reset
	void reset() { _flag_reset = true; }

    // reset the filter states
    void reset(T out, T in) { _state_1 = out; _state_2 = in; }

	// apply the filter on an input sample
	T apply( T input, float dt );

private:

	// reset flag
	bool _flag_reset = true;

	// critical frequency for pre-warping, [kHz]
	float _critical_freq_kHz = 0.0f;

	// filter bandwidth, [kHz]
	float _bandwidth_kHz = 0.0f;

	// filter time constant, [msec]
	float _time_const_msec = 0.0f;

	// sampling period, [msec]
	float _Ts_msec;

	// filter coefficients
	float _c1;

	// state-storage variables
	T _state_1, _state_2;

	// update the sample time
	void update_sample_time_sec( float sample_time_sec ) { _Ts_msec = sample_time_sec * 1000; }

	// update the filter coefficients
	void update_coeffs();
};

typedef UVX_FirstOrderLPF<float> UVX_FirstOrderLPFfloat;
typedef UVX_FirstOrderLPF<Vector2f> UVX_FirstOrderLPFVector2f;
