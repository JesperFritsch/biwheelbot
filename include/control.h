#include <stdint.h>

struct ControlSnapshot {
    float angle;
    float rate;
    float battery_voltage;
    float motor_a_duty;
    float motor_b_duty;
    bool motors_enabled;
    uint32_t overruns;
};

void control_start();
ControlSnapshot control_get_snapshot();