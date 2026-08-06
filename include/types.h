#pragma once
#include <stdint.h>

struct Vec3 {
    float x, y, z;
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }
};


struct PitchMeasurement {
    float angle; // degrees, accel-derived tilt
    float rate;  // deg/s, gyro (sign matches d(angle)/dt)
};


struct WheelState {
    float speed; // mm per second
    int32_t count;
};


struct WheelStates {
    WheelState a;
    WheelState b;
};