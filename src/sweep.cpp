// --- sweep.cpp ---
//
// Characterises duty -> speed for both motors and both directions, to build the
// duty linearisation (ff_duty). Two thresholds matter and they are different
// numbers:
//
//   static breakaway - the duty needed to start a stopped wheel
//   kinetic dropout  - the duty at which an already-turning wheel stops
//
// The dropout is the one ff_duty wants, since while balancing the wheels are
// essentially always moving. It is always below the breakaway, so every descent
// here runs until the wheel actually stops rather than to a fixed floor -- an
// earlier version used a fixed floor and missed the dropout on 6 of 8 runs.

#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"

#define SETTLE_MS    300
#define WINDOW_MS    500

#define PROBE_MS     120     // dwell per breakaway probe
#define PROBE_TICKS  3       // counts within PROBE_MS that mean "turning"
#define PROBE_MAX    0.60f   // give up past here; something is wrong
#define STOP_TICKS   5       // counts within WINDOW_MS below which it has stopped

#define COARSE_STEPS 15
#define CRAWL_STEP   0.01f
#define CRAWL_SPAN   0.10f  // either side of the coarse dropout

static int32_t measure(bool motor_a, float duty, const char* tag) {
    if (motor_a) motor_set_a(duty); else motor_set_b(duty);
    delay(SETTLE_MS);

    WheelState w0 = sensor_get_wheels();
    int32_t c0 = motor_a ? w0.position.a : w0.position.b;
    uint32_t t0 = micros();

    delay(WINDOW_MS);

    WheelState w1 = sensor_get_wheels();
    int32_t c1 = motor_a ? w1.position.a : w1.position.b;
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

    return ticks;
}

static void stop_and_rest(bool motor_a) {
    if (motor_a) motor_set_a(0.0f); else motor_set_b(0.0f);
    delay(1500);
}

// Ramp up from rest until the wheel turns, and report that duty.
//
// One continuous ramp, never stopping: that is the condition static breakaway
// is defined under. Binary search would be faster but meaningless, since the
// threshold depends on whether the wheel is already moving, so every probe
// would need its own rest first. Coarse below 0.10 where nothing has ever
// moved, fine above it -- that is most of the time saved.
static float find_deadzone(bool motor_a, float sign) {
    for (float d = 0.0f; d <= PROBE_MAX; d += (d < 0.10f ? 0.02f : 0.005f)) {
        if (motor_a) motor_set_a(sign * d); else motor_set_b(sign * d);

        WheelState w0 = sensor_get_wheels();
        int32_t c0 = motor_a ? w0.position.a : w0.position.b;
        delay(PROBE_MS);
        WheelState w1 = sensor_get_wheels();
        int32_t c1 = motor_a ? w1.position.a : w1.position.b;

        if (abs(c1 - c0) >= PROBE_TICKS) return d;
    }
    return PROBE_MAX;
}

// Step down from `hi` until the wheel stops, and return the duty it stopped at.
// No lower bound other than zero: the dropout is what we came for, and guessing
// a floor is exactly how it gets missed.
static float descend_to_stop(bool motor_a, float sign, float hi, float step,
                             const char* tag) {
    float d = hi;
    for (; d > 0.0f; d -= step) {
        if (abs(measure(motor_a, sign * d, tag)) < STOP_TICKS) return d;
    }
    return 0.0f;
}

// Breakaway to full duty and back. The ascent starts at the breakaway so no
// point is wasted below it; the descent keeps going until it stops.
static float coarse_sweep(bool motor_a, float sign, const char* dir, float d0) {
    char tag[24];
    float step = (1.0f - d0) / COARSE_STEPS;

    sprintf(tag, "coarse_%s_up", dir);
    for (int i = 0; i <= COARSE_STEPS; i++) measure(motor_a, sign * (d0 + step * i), tag);

    sprintf(tag, "coarse_%s_dn", dir);
    float stop = descend_to_stop(motor_a, sign, 1.0f, step, tag);

    stop_and_rest(motor_a);
    return stop;
}

// Fine resolution around the dropout the coarse pass located, so both
// thresholds land inside the window at 0.005 rather than the coarse ~0.04.
// Centred on the measured value rather than a guess, because the two motors and
// the two directions do not share one.
static void crawl_sweep(bool motor_a, float sign, const char* dir, float centre) {
    char tag[24];
    float lo = centre - CRAWL_SPAN / 2.0f;
    float hi = centre + CRAWL_SPAN * 2.0f;  // overshoot to see the full descent
    if (lo < 0.0f) lo = 0.0f;

    // Ascent from rest below the dropout: this leg captures the static
    // breakaway at fine resolution, which sits above the dropout.
    sprintf(tag, "crawl_%s_up", dir);
    for (float d = lo; d <= hi; d += CRAWL_STEP) measure(motor_a, sign * d, tag);

    sprintf(tag, "crawl_%s_dn", dir);
    descend_to_stop(motor_a, sign, hi, CRAWL_STEP, tag);

    stop_and_rest(motor_a);
}

void run_full_sweep() {
    Serial.println("tag,motor,duty,v_batt,v_eff,ticks,dt_s,mmps");

    for (int m = 0; m < 2; m++) {
        bool motor_a = (m == 0);
        for (int d = 0; d < 2; d++) {
            float sign = (d == 0) ? 1.0f : -1.0f;
            const char* dir = (d == 0) ? "fwd" : "rev";

            float d0 = find_deadzone(motor_a, sign);
            stop_and_rest(motor_a);   // both sweeps start from rest, as measured

            float stop = coarse_sweep(motor_a, sign, dir, d0);
            crawl_sweep(motor_a, sign, dir, stop > 0.0f ? stop : d0);

            // No commas: the loader skips any line without a motor column.
            char note[64];
            sprintf(note, "# thresholds %c %s breakaway %.4f dropout %.4f",
                    motor_a ? 'A' : 'B', dir, d0, stop);
            Serial.println(note);
        }
    }

    motor_set_a(0.0f);
    motor_set_b(0.0f);
    Serial.println("SWEEP_DONE");
}
