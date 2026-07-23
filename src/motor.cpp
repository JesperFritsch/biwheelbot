
#include "motor.h"
#include "Arduino.h"

#define enB 9
#define inB1 7
#define inB2 8
#define enA 10
#define inA1 11
#define inA2 12


void motor_set_a(float duty) {
    // duty is -1 - 1;
    int pwm = 0;
    if (duty > 0) {
        digitalWrite(inA1, HIGH);
        digitalWrite(inA2, LOW);
        pwm = duty * 0xff;
    }
    else {
        digitalWrite(inA1, LOW);
        digitalWrite(inA2, HIGH);
        pwm = -duty * 0xff;
    }
    analogWrite(enA, pwm);
}

void motor_set_b(float duty) {
    // duty is -1 - 1;
    int pwm = 0;
    if (duty > 0) {
        digitalWrite(inB1, HIGH);
        digitalWrite(inB2, LOW);
        pwm = duty * 0xff;
    }
    else {
        digitalWrite(inB1, LOW);
        digitalWrite(inB2, HIGH);
        pwm = -duty * 0xff;
    }
    analogWrite(enB, pwm);
}
