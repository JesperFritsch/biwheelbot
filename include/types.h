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

struct WheelSpeed {
    float a;
    float b;
    float avg_speed() const { return (a + b) * 0.5f; }
};

struct WheelPosition {
    int32_t a;
    int32_t b;
    int32_t avg_pos() const { return (a + b) / 2; }
};

struct WheelState {
    WheelSpeed speed;
    WheelPosition position;
};

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

