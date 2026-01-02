
/// TODO: License description

/// @file UVX_SecondOrderLPF.h
/// @brief Second order low-pass filter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

/**
 * A class implementing a second order low-pass filter with transfer function W(s) = 1 / ( TF^2 * s^2 + 2 * damping * TF * s + 1 ), where
 * TF		-> filter time constant, [s]
 * damping  -> filter damping factor
 * The filter can be tuned by specifying either the desired filter time constant in seconds or filter bandwidth in Hz.
 * If a critical frequency in Hz is set a pre-warping is used for calculating the filter coefficients. The discrete-time
 * implementation of the filter is according to the BLT algorithm.
 * The appropriate functions set_crit_freq_Hz, set_bandwidth_Hz, set_time_const_sec or set_damping should be called during filter
 * initialization or in runtime. The default value for the damping factor is set to 1.0. The filter supports bypass mode in which
 * the state variables track the input and its first time derivative. 
 */
class UVX_SecondOrderLPF
{
public:

	// set critical frequency
	void set_crit_freq_Hz(float crit_freq_Hz) { _critical_freq_kHz = crit_freq_Hz * 0.001f; }

	// set the filter bandwidth
	void set_bandwidth_Hz(float bandwidth_Hz) { _bandwidth_kHz = bandwidth_Hz * 0.001f; }

	// set the filter time constant
	void set_time_const_sec(float time_const_sec) { _time_const_msec = time_const_sec * 1000; }

	// set the damping factor
	void set_damping(float damping) { _damping = damping; }

	// issue a filter reset
	void reset() { _flag_reset = true; }

    // reset the filter states to the provided values and store the current filter input 
    void reset(float out, float out_dot, float input);

	// set filter bypass
	void set_bypass() { _flag_bypass = true; }

	// remove filter bypass
	void remove_bypass() { _flag_bypass = false; }

	// apply the filter on an input sample
	float apply(float input, float dt);

    // get the filter last output
    float get_last_output() { return _state_1; }

	// get first derivative of filter output
	float get_output_derivative() { return _state_2; }

    // get the damping factor
    float get_damping() { return _damping; }

    // get the filter time constant in seconds
    float get_time_const_sec() { return _time_const_msec / 1000.0f; }

private:

	// reset flag
	bool _flag_reset = true;

	// bypass flag
	bool _flag_bypass = false;

	// critical frequency for pre-warping, [kHz]
	float _critical_freq_kHz = 0.0f;

	// filter bandwidth, [kHz]
	float _bandwidth_kHz = 0.0f;

	// filter time constant, [msec]
	float _time_const_msec = 0.0f;

	// filter damping factor
	float _damping = 1.0f;

	// sampling period, [msec]
	float _Ts_msec;

	// filter coefficients
	float _c1, _c2, _c3, _c4, _sample_freq_Hz;

	// state-storage variables
	float _state_1, _state_2, _state_3;

	// update the sample time
	void update_sample_time_sec(float sample_time_sec) { _Ts_msec = sample_time_sec * 1000; }

	// update the filter coefficients
	void update_coeffs();
};
