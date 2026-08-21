#pragma once
#include <stdint.h>
#include "utils.h"
#include "types.h"

class PIDController {
public:
    // `gains` is held by reference so a mid-run retune takes effect on the next
    // update() -- it must outlive the controller, and must not be written
    // concurrently. control.cpp satisfies both with a control-thread-private
    // copy that is refreshed only at a tick boundary.
    PIDController(const PIDGains& gains, float limit=infinity(), float i_cap=infinity(), bool i_snapback=true) :
        gains(gains), 
        integral(0.0f), 
        prev_error(0.0f), 
        limit(limit),
        i_cap(i_cap),
        i_snapback(i_snapback) {}
    float update(float current_value, float target_value, float dt) {
        float error = target_value - current_value;
        integral += error * dt;
        if (integral > i_cap) {
            integral = i_cap;
        }
        else if (integral < -i_cap) {
            integral = -i_cap;
        }
        if (i_snapback && e_is_neg != (error < 0)) {
            integral = 0;
        }
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        float result = gains.kp * error + gains.ki * integral + gains.kd * derivative;
        e_is_neg = error < 0;
        return sym_cap(result, limit);
    } 
private:
    const PIDGains& gains;
    float integral;
    float prev_error;
    float limit;
    float i_cap;
    bool e_is_neg;
    bool i_snapback;
};


struct ControlSnapshot {
    float angle;        // the estimate actually closing the loop
    float rate;
    float kf_angle;     // both estimators, logged every tick regardless of
    float battery_voltage;
    float turning_duty;
    float effort_duty;
    float target_angle;
    float target_speed;
    float motor_a_duty;
    float motor_b_duty;
    bool motors_enabled;
    float t_pos_mm;
    float pos_mm;
    uint32_t overruns;
};

void control_start();
ControlSnapshot control_get_snapshot();

// Retune one PID block from another thread. The write is atomic against the
// control loop: it either sees the whole triple or none of it, never a mix of
// old and new terms. Out-of-range ids are ignored.
void control_set_gains(GainId id, PIDGains gains);
PIDGains control_get_gains(GainId id);
void control_set_drive(DriveCmd cmd);