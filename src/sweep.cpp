// --- sweep.cpp ---

#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"

#define SETTLE_MS   400
#define WINDOW_MS   1000
#define CRAWL_START 0.150f
#define CRAWL_STEP  0.005f
#define CRAWL_STEPS 41      // 0.150 .. 0.350

static void measure(bool motor_a, float duty, const char* tag) {
    if (motor_a) motor_set_a(duty); else motor_set_b(duty);
    delay(SETTLE_MS);

    WheelStates w0 = sensor_wheels();
    int32_t c0 = motor_a ? w0.a.count : w0.b.count;
    uint32_t t0 = micros();

    delay(WINDOW_MS);

    WheelStates w1 = sensor_wheels();
    int32_t c1 = motor_a ? w1.a.count : w1.b.count;
    uint32_t t1 = micros();

    float v_batt = sensor_read_battery();
    float dt = (t1 - t0) / 1e6f;
    int32_t ticks = c1 - c0;
    float speed = ticks * MM_PER_COUNT / dt;
    float v_eff = duty * v_batt;

    char buf[160];
    sprintf(buf, "%s,%c,%.4f,%.3f,%.4f,%ld,%.4f,%.2f",
            tag,
            motor_a ? 'A' : 'B',
            duty, v_batt, v_eff,
            (long)ticks, dt, speed);
    Serial.println(buf);
}

static void stop_and_rest(bool motor_a) {
    if (motor_a) motor_set_a(0.0f); else motor_set_b(0.0f);
    delay(1500);
}

static void coarse_sweep(bool motor_a, float sign, const char* dir) {
    char tag[24];
    sprintf(tag, "coarse_%s_up", dir);
    for (int i = 1; i <= 20; i++) measure(motor_a, sign * 0.05f * i, tag);

    sprintf(tag, "coarse_%s_dn", dir);
    for (int i = 20; i >= 1; i--) measure(motor_a, sign * 0.05f * i, tag);

    stop_and_rest(motor_a);
}

static void crawl_sweep(bool motor_a, float sign, const char* dir) {
    char tag[24];
    sprintf(tag, "crawl_%s_up", dir);
    for (int i = 0; i < CRAWL_STEPS; i++)
        measure(motor_a, sign * (CRAWL_START + CRAWL_STEP * i), tag);

    sprintf(tag, "crawl_%s_dn", dir);
    for (int i = CRAWL_STEPS - 1; i >= 0; i--)
        measure(motor_a, sign * (CRAWL_START + CRAWL_STEP * i), tag);

    stop_and_rest(motor_a);
}

void run_full_sweep() {
    Serial.println("tag,motor,duty,v_batt,v_eff,ticks,dt_s,mmps");

    for (int m = 0; m < 2; m++) {
        bool motor_a = (m == 0);
        for (int d = 0; d < 2; d++) {
            float sign = (d == 0) ? 1.0f : -1.0f;
            const char* dir = (d == 0) ? "fwd" : "rev";
            // coarse_sweep(motor_a, sign, dir);
            crawl_sweep(motor_a, sign, dir);
        }
    }

    motor_set_a(0.0f);
    motor_set_b(0.0f);
    Serial.println("SWEEP_DONE");
}