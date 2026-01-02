
/// TODO: License description

/// @file UVX_Rate_Limiter.cpp
/// @brief Rate limiter class
#include "UVX_Rate_Limiter.h"

float UVX_Rate_Limiter::apply(float input, float dt)
{
    if ( _flag_reset )
    {
        _output = input;
        _flag_reset = false;
    }
    else if ( !is_zero(dt) )
    {
        float input_rate = ( input - _output ) / dt;

        if ( fabsf( input_rate ) < _max_rate )
        {
            _output = input;
        }
        else
        {
            if ( input_rate < 0.0f )
            {
                _output -= dt * _max_rate;
            }
            else
            {
                _output += dt * _max_rate;
            }
        }
    }

    return _output;
}
