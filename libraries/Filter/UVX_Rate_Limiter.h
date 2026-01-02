
/// TODO: License description

/// @file UVX_Rate_Limiter.h
/// @brief Rate limiter class
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>

/**
 * A class implementing a rate limiter
 */
class UVX_Rate_Limiter
{
public:

    // set max rate in output / s
    void set_max_rate(float max_rate_per_sec) { _max_rate = max_rate_per_sec; }

    // get the max rate in output / s
    float get_max_rate() { return _max_rate; }

    // reset the rate limiter output to the current input
    void reset() { _flag_reset = true; }

    // reset the rate limiter output to the specified input
    void reset( float input ) { _output = input; _flag_reset = false; }

    // apply the rate limiter on an input sample
    float apply(float input, float dt);

private:

    // maximum output rate in output / s
    float _max_rate = 0;

    // rate limiter output
    float _output;

    // reset flag
    bool _flag_reset = true;
};
