#pragma once
#include <stdint.h>
#include "utils.h"
#include "types.h"

class PIDController {
public:
    PIDController(const volatile PIDGains& gains, float limit=infinity(), float i_cap=infinity(), bool i_snapback=true) : 
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
    const volatile PIDGains& gains;
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
    float comp_angle;   // which one is selected, so they can be compared
    float battery_voltage;
    float turning_duty;
    float effort_duty;
    float target_angle;
    float target_speed;
    float ab_diff_drift;
    float motor_a_duty;
    float motor_b_duty;
    bool motors_enabled;
    float t_pos_mm;
    float pos_mm;
    uint32_t overruns;
};

void control_start();
ControlSnapshot control_get_snapshot();
void set_balance_gains(PIDGains gains);
PIDGains get_balance_gains();
void set_speed_gains(PIDGains gains);
PIDGains get_speed_gains();
void set_pos_gains(PIDGains gains);
PIDGains get_pos_gains();
void set_turn_gains(PIDGains gains);
PIDGains get_turn_gains();
