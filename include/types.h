#pragma once
#include <stdint.h>

struct Vec3 {
    float x, y, z;
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }
};


struct PitchMeasurement {
    float angle;     // degrees, accel-derived tilt
    float rate;      // deg/s, gyro (sign matches d(angle)/dt)
    float accel_dev; // |accel magnitude - calibrated rest magnitude| in g; >0 means non-gravity accel is corrupting `angle`
};


struct WheelState {
    float speed; // mm per second
    int32_t count;
};


struct WheelStates {
    WheelState a;
    WheelState b;
};