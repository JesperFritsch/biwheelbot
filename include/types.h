#pragma once
#include <stdint.h>

constexpr float WHEEL_CIRCUM_MM = 226.0f;
constexpr float COUNTS_PER_REV = 425.0f;
constexpr float MM_PER_COUNT = WHEEL_CIRCUM_MM / COUNTS_PER_REV;

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
    int32_t pos_diff() const { return a - b; }
    float avg_pos_mm() const { return avg_pos() * MM_PER_COUNT; }
    float pos_diff_mm() const { return pos_diff() * MM_PER_COUNT; }
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

struct DriveCmd { 
    int8_t linear; // drive 
    int8_t angular; // turn
    uint8_t flags; 
    uint8_t seq;
};

// Identifies one tunable PID block. Every layer that carries gains around --
// the control loop's defaults table, the BLE characteristic table, the hooks --
// is indexed by this, so adding a loop means adding one row to each and the
// compiler finds the ones you missed.
enum GainId : uint8_t {
    GAIN_BALANCE,   // angle -> duty
    GAIN_SPEED,     // speed -> angle
    GAIN_POS,       // position -> speed
    GAIN_TURN,      // wheel difference -> turn duty
    GAIN_COUNT
};

