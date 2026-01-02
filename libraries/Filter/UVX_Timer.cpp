
/// TODO: License description

/// @file UVX_Timer.cpp
/// @brief Timer class
#include "UVX_Timer.h"

// update the timer value - return true if current timer value >= timeout
bool UVX_Timer::update(float dt)
{
    _t += dt;

    if ( _t < 0.0f )
    {
        _t = 0.0f;
        return false;
    }

    if (_t < _timeout)
    {
        return false;
    }
    else
    {
        _t = _timeout;
        return true;
    }
}
