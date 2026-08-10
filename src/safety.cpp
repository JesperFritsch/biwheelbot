#include "safety.h"

const float SAFETY_ANGLE_MAX = 75.0f; // degrees
const float SAFETY_ANGLE_RESTORE = 5.0f; // degrees
const float SAFETY_ANGLE_RATE_RESTORE = 5.0f; // deg/s
const float SAFETY_RESTORE_DELAY_MS = 300; // ms
const float SAFETY_BATT_MIN = 6.0f; // volts


MotorState safety_update(float angle_deg, float angle_rate, float battery_voltage) {
    static MotorState state = MotorState::OFF;
    static uint32_t t_on = 0;

    if (state == MotorState::OFF) {
        if (fabsf(angle_deg) < SAFETY_ANGLE_RESTORE && fabsf(angle_rate) < SAFETY_ANGLE_RATE_RESTORE) {
            if (t_on == 0) t_on = millis();
            else if (millis() - t_on > SAFETY_RESTORE_DELAY_MS) {
                state = MotorState::ON;
                }
            } else {
                t_on = 0;
            }
        } else {
            if (fabsf(angle_deg) > SAFETY_ANGLE_MAX) {
                state = MotorState::OFF;
            }
        }
        if (battery_voltage < SAFETY_BATT_MIN) {
        state = MotorState::OFF;
        }
        
    return state;
}