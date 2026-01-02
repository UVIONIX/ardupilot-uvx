
/// TODO: License description

/// @file UVX_Timer.h
/// @brief Timer class
#pragma once

class UVX_Timer
{
public:

    // reset the timer
    void reset() { _t = 0.0f; }
    void reset(float t0) { _t = t0; }

    // set timeout
    void set_timeout(float timout) { _timeout = timout; }

    // get the current timer value
    float get() { return _t; }

    // get the current timer value relative to the timeout
    float get_relative() { return (_t / _timeout); }

    // update the timer value - return true if current timer value >= timeout
    bool update(float dt);

private:

    // current timer value
    float _t = 0.0f;

    // timeout value
    float _timeout = 1.0f;
};
