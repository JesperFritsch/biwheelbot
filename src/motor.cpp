
#include "motor.h"
#include <mbed.h>
#include "Arduino.h"

#define enB 9
#define inB1 7
#define inB2 8
#define enA 10
#define inA1 11
#define inA2 12



mbed::PwmOut pwm_a(digitalPinToPinName(enA));
mbed::PwmOut pwm_b(digitalPinToPinName(enB));

bool motors_enabled = false;

void init_motors() {
    pwm_a.period_us(300);
    pwm_b.period_us(300);
}


void motor_set_a(float duty) {
    // duty is -1 - 1;
    if (duty > 0) {
        digitalWrite(inA1, HIGH);
        digitalWrite(inA2, LOW);
    }
    else {
        digitalWrite(inA1, LOW);
        digitalWrite(inA2, HIGH);
    }
    if (!motors_enabled) {
        duty = 0;
    }
    pwm_a.write(fabsf(duty));
}


void motor_set_b(float duty) {
    // duty is -1 - 1;
    if (duty > 0) {
        digitalWrite(inB1, HIGH);
        digitalWrite(inB2, LOW);
    }
    else {
        digitalWrite(inB1, LOW);
        digitalWrite(inB2, HIGH);
    }
    if (!motors_enabled) {
        duty = 0;
    }
    pwm_b.write(fabsf(duty));
}

void set_motors_enabled(bool enabled) {
    motors_enabled = enabled;
    motor_set_a(0.0);
    motor_set_b(0.0);
}
