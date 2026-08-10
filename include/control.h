#pragma once
#include <stdint.h>
#include "utils.h"
#include "types.h"

class PIDController {
public:
    PIDController(const volatile PIDGains& gains, float limit=infinity()) : gains(gains), integral(0.0f), prev_error(0.0f), limit(limit) {}
    float update(float current_value, float target_value, float dt) {
        float error = target_value - current_value;
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        float result = gains.kp * error + gains.ki * integral + gains.kd * derivative;
        return SYM_CAP(result, limit);
    } 
private:
    const volatile PIDGains& gains;
    float integral;
    float prev_error;
    float limit;
};


struct ControlSnapshot {
    float angle;
    float rate;
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
void set_balance_gains(PIDGains gains);
void set_speed_gains(PIDGains gains);
PIDGains get_balance_gains();
PIDGains get_speed_gains();
void set_pos_gains(PIDGains gains);
PIDGains get_pos_gains();