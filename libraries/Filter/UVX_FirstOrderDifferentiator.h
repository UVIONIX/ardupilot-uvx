
/// TODO: License description

/// @file UVX_FirstOrderDifferentiator.h
/// @brief First order differentiator filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

/**
 * A class implementing a differentiator with a first order low-pass filter with transfer function W(s) = s / ( TF * s + 1 ), where
 * TF  -> filter time constant, [s]
 * The filter can be tuned by specifying either the desired filter time constant in seconds or filter bandwidth in Hz.
 * If a critical frequency in Hz is set a pre-warping is used for calculating the filter coefficients. The discrete-time
 * implementation of the differentiator is according to the BLT algorithm.
 * The appropriate methods set_crit_freq_Hz, set_bandwidth_Hz or set_time_const_sec should be called during filter
 * initialization or in runtime.
 */
template <class T>
class UVX_FirstOrderDifferentiator
{
public:

	// set critical frequency
	void set_crit_freq_Hz(float crit_freq_Hz) { _critical_freq_kHz = crit_freq_Hz * 0.001f; }

	// set the filter bandwidth
	void set_bandwidth_Hz(float bandwidth_Hz) { _bandwidth_kHz = bandwidth_Hz * 0.001f; }

	// set the filter time constant
	void set_time_const_sec(float time_const_sec) { _time_const_msec = time_const_sec * 1000; }

    // get the last output
    T get_last_output() const { return _state_1; }

	// issue a reset
	void reset() { _flag_reset = true; }

    // reset the filter states to the provided values and store the current filter input
    void reset(T out, T input);

	// apply the differentiator on an input sample
	T apply(T input, float dt);

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

	// differentiator coefficients
	float _c1, _c2;

	// state-storage variables
	T _state_1, _state_2;

	// update the sample time
	void update_sample_time_sec(float sample_time_sec) { _Ts_msec = sample_time_sec * 1000; }

	// update the differentiator coefficients
	void update_coeffs();
};

typedef UVX_FirstOrderDifferentiator<float> UVX_FirstOrderDifferentiatorfloat;
typedef UVX_FirstOrderDifferentiator<Vector2f> UVX_FirstOrderDifferentiatorVector2f;
