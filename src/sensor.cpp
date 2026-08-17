
#include "sensor.h"
#include "types.h"
#include "utils.h"
#include "nrf.h"
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include "Arduino.h"
#include <algorithm>


#define PIN_ENCODER_A1 6
#define PIN_ENCODER_A2 5
#define PIN_ENCODER_B1 3
#define PIN_ENCODER_B2 4

#define NRF_PIN_ENCODER_A1 14
#define NRF_PIN_ENCODER_A2 13
#define NRF_PIN_ENCODER_B1 12
#define NRF_PIN_ENCODER_B2 15

#define BATT_V_PIN A0

#define GYRO_CALIB_SAMPLES 256
#define SPEED_TICK_TIMEOUT_US 50000

static const float SPEED_SMOOTHING_STEPS = 10.0f;

static LowPassFilter low_pass_speed_a(SPEED_SMOOTHING_STEPS);
static LowPassFilter low_pass_speed_b(SPEED_SMOOTHING_STEPS);

static const int8_t DECODE_TABLE[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
    -1,  0,  0,  1,
    0,  1, -1,  0
};

// Per-transition distance scale, measured 2026-08-07 with run_edge_phase_test
// over four duties in both directions (shares agreed within 0.2 pp throughout).
//
// A quadrature cycle is 4 counts, but the four edges are not evenly spaced: A's
// channels sit 89.4 deg apart and B's only 68.8 deg, so each transition covers a
// different arc. Indexed by (prev_state << 2) | state, these hold the arc each
// transition actually spans, as MM_PER_COUNT * 1e6 scaled by (4 * cycle share) --
// so speed is just SCALE[idx] / delta_us and the correction costs nothing.
//
// The index encodes direction as well as edge, which matters: comparator
// hysteresis shifts the switching angles slightly depending on which way the
// shaft turns. The 8 unreachable indices hold the nominal value.
#define EDGE_SCALE_NOMINAL 531764.7f

static const float EDGE_SCALE_A[16] = {
    EDGE_SCALE_NOMINAL, 538145.9f, 528255.1f, EDGE_SCALE_NOMINAL,
    528680.5f,          EDGE_SCALE_NOMINAL, EDGE_SCALE_NOMINAL, 520012.7f,
    516237.2f,          EDGE_SCALE_NOMINAL, EDGE_SCALE_NOMINAL, 526766.1f,
    EDGE_SCALE_NOMINAL, 543304.0f, 552663.1f, EDGE_SCALE_NOMINAL,
};

static const float EDGE_SCALE_B[16] = {
    EDGE_SCALE_NOMINAL, 652741.2f, 648221.2f, EDGE_SCALE_NOMINAL,
    402545.9f,          EDGE_SCALE_NOMINAL, EDGE_SCALE_NOMINAL, 398876.7f,
    404673.0f,          EDGE_SCALE_NOMINAL, EDGE_SCALE_NOMINAL, 410469.2f,
    EDGE_SCALE_NOMINAL, 665875.8f, 670768.0f, EDGE_SCALE_NOMINAL,
};

static volatile int32_t enc_count_a = 0, enc_count_b = 0;
static volatile float wheel_a_speed_mms = 0, wheel_b_speed_mms = 0;
static volatile uint8_t prev_state_a = 0, prev_state_b = 0;
static volatile uint32_t last_tick_a = 0, last_tick_b = 0;
static volatile uint32_t prev_delta_a = 0, prev_delta_b = 0;

static Vec3 gyro_cal{};
static Vec3 gyr_raw{}, acc_raw{};
static float accel_norm_rest = 1.0f;  // resting |accel| measured at calibration; absorbs the accel's static offset/scale error
static float accel_dev_floor = 0.0f;  // 3 sigma of resting |accel| noise; deviations below this are indistinguishable from noise

void encode_a();
void encode_b();


// ---------------------------------------------------------------------------
// Pole geometry correction.
//
// EDGE_SCALE above fixes the phase between the two channels, which is one
// property shared by every pole, so it pools all 11 pole pairs into four
// numbers. What it cannot see is that the poles themselves are unevenly spaced:
// B's magnet is eccentric, leaving ~6% RMS error per edge, and A's channels are
// nearly perfect so almost all of its ~3% is pole error too.
//
// These tables are the measured arc of every transition across one magnet
// rotation, normalised so a flawless encoder reads 1.0. Measured 2026-08-08,
// reproducible to ~1% of their own amplitude across a power cycle and a
// disassembly. Regeneration procedure: docs/encoder-calibration.md.
static const float POLE_FULL_A[POLE_BINS] = {
    0.9751f, 1.0192f, 0.9561f, 1.0338f,
    0.9651f, 1.0364f, 0.9689f, 1.0285f,
    1.0077f, 0.9873f, 0.9777f, 1.0320f,
    1.0014f, 0.9774f, 1.0089f, 0.9755f,
    1.0195f, 0.9868f, 0.9944f, 1.0408f,
    1.0344f, 0.9409f, 0.9764f, 1.0481f,
    0.9916f, 1.0141f, 1.0032f, 0.9847f,
    1.0204f, 0.9590f, 1.0150f, 1.0414f,
    0.9701f, 0.9616f, 1.0320f, 1.0011f,
    0.9883f, 1.0319f, 0.9653f, 1.0380f,
    0.9589f, 1.0207f, 0.9736f, 1.0368f,
};

static const float POLE_FULL_B[POLE_BINS] = {
    0.7405f, 1.2116f, 0.7021f, 1.3346f,
    0.6785f, 1.3035f, 0.7055f, 1.2916f,
    0.7041f, 1.2449f, 0.7216f, 1.3395f,
    0.6752f, 1.2244f, 0.7755f, 1.2477f,
    0.7265f, 1.2742f, 0.7617f, 1.2453f,
    0.7658f, 1.1898f, 0.8344f, 1.1997f,
    0.8223f, 1.1647f, 0.8389f, 1.1503f,
    0.8305f, 1.1849f, 0.8180f, 1.2058f,
    0.8480f, 1.1647f, 0.7988f, 1.2312f,
    0.7943f, 1.2026f, 0.7963f, 1.1957f,
    0.7851f, 1.2057f, 0.7579f, 1.3062f,
};

// Recovery needs at least this many samples in every bin before it will even
// look, and the peak must beat the runner-up by this margin before it commits.
// B is the demanding case: its pattern is a smooth once-per-rotation hump, so a
// wrong-by-one-pole alignment still correlates around 0.84 against 0.99 for the
// right one. 40 samples per bin puts the noise on those figures near 0.03.
#define POLE_MIN_SAMPLES 40
#define POLE_MIN_QUALITY 0.10f

// Pole error only, with the channel phase divided out, so it multiplies onto
// EDGE_SCALE rather than replacing it. Keeping them separate matters: a wrong
// lock then costs the ~6% pole term instead of scrambling the 25% channel term.
static float pole_corr_a[POLE_BINS], pole_corr_b[POLE_BINS];

// The four per-transition means the residual was taken around: the channel
// phase, which EDGE_SCALE already applies. Kept because recovery matches
// against them to find the alignment modulo 4.
static float pole_mean_a[4], pole_mean_b[4];

// The correction already rotated into the running index, so the ISR is one
// array read and one multiply.
static float pole_live_a[POLE_BINS], pole_live_b[POLE_BINS];

// Free-running 0..43, stepped with the count. Its offset from the table is
// exactly what recovery solves for, so the true count is never needed.
static volatile uint8_t pole_k_a = 0, pole_k_b = 0;
static volatile bool pole_locked_a = false, pole_locked_b = false;

static volatile uint32_t pole_sum_a[POLE_BINS], pole_cnt_a[POLE_BINS];
static volatile uint32_t pole_sum_b[POLE_BINS], pole_cnt_b[POLE_BINS];
static volatile uint32_t miss_a = 0, miss_b = 0;

static uint8_t pole_shift_a = 0, pole_shift_b = 0;
static float pole_quality_a = 0.0f, pole_quality_b = 0.0f;
static uint32_t miss_seen_a = 0, miss_seen_b = 0;


// Split each table into its four per-transition means and the residual around
// them. The means duplicate what EDGE_SCALE already applies; the residual is
// what is actually new, and it averages 1.0 within each transition type.
//
// The means also drive stage 1 of recovery. Every residue class k % 4 spans all
// 11 poles, so its mean does not depend on which pole the count started at --
// rotating the live pattern only permutes the four means. That makes them an
// exact handle on the alignment modulo 4, with the pole error cancelled rather
// than merely averaged down.
static void pole_derive(const float* full, float* corr, float* mean) {
    for (int t = 0; t < 4; t++) {
        float sum = 0.0f;
        for (int k = t; k < POLE_BINS; k += 4) sum += full[k];
        mean[t] = sum / (POLE_BINS / 4);
        for (int k = t; k < POLE_BINS; k += 4) corr[k] = full[k] / mean[t];
    }
}


void pole_lock_reset(bool motor_a) {
    volatile uint32_t* sum = motor_a ? pole_sum_a : pole_sum_b;
    volatile uint32_t* cnt = motor_a ? pole_cnt_a : pole_cnt_b;
    float* live = motor_a ? pole_live_a : pole_live_b;

    noInterrupts();
    if (motor_a) pole_locked_a = false; else pole_locked_b = false;
    for (int k = 0; k < POLE_BINS; k++) { sum[k] = 0; cnt[k] = 0; live[k] = 1.0f; }
    interrupts();
}


// Pearson correlation of a[] against b[] rotated by `shift`.
static float pole_correlate(const float *a, const float *b, int shift)
{
    float ma = 0.0f, mb = 0.0f;
    for (int k = 0; k < POLE_BINS; k++) { ma += a[k]; mb += b[k]; }
    ma /= POLE_BINS;
    mb /= POLE_BINS;

    float cov = 0.0f, va = 0.0f, vb = 0.0f;
    for (int k = 0; k < POLE_BINS; k++)
    {
        float da = a[k] - ma;
        float db = b[(k + shift) % POLE_BINS] - mb;
        cov += da * db;
        va += da * da;
        vb += db * db;
    }
    return (va > 0.0f && vb > 0.0f) ? cov / sqrtf(va * vb) : -2.0f;
}


// Find where the live pattern sits in the table. Two stages, because the two
// features in the pattern resolve different things and neither can do both:
//
//   - The channel-phase alternation is identical for every pole, so it fixes
//     the alignment only modulo 4. It lives entirely in the four per-transition
//     means, which is where stage 1 reads it -- comparing whole patterns here
//     fails, see below.
//   - The pole residual is unique per pole, so it picks among the 11 candidates
//     that share an alignment. Noiseless, the right shift scores 1.0 against
//     0.67 for B's next-best and 0.13 for A's.
static void pole_try_lock(bool motor_a)
{
    volatile uint32_t *vsum = motor_a ? pole_sum_a : pole_sum_b;
    volatile uint32_t *vcnt = motor_a ? pole_cnt_a : pole_cnt_b;

    uint32_t sum[POLE_BINS], cnt[POLE_BINS];
    noInterrupts();
    for (int k = 0; k < POLE_BINS; k++) { sum[k] = vsum[k]; cnt[k] = vcnt[k]; }
    interrupts();

    uint32_t n_min = 0xFFFFFFFFu;
    for (int k = 0; k < POLE_BINS; k++) if (cnt[k] < n_min) n_min = cnt[k];
    if (n_min < POLE_MIN_SAMPLES) return;

    // Same normalisation the table was built with: each bin's mean interval as
    // a multiple of the nominal 1/44 of a rotation. Speed drift cancels because
    // every bin is visited once per rotation.
    float f[POLE_BINS], total = 0.0f;
    for (int k = 0; k < POLE_BINS; k++)
    {
        f[k] = (float)sum[k] / (float)cnt[k];
        total += f[k];
    }
    if (total <= 0.0f) return;
    for (int k = 0; k < POLE_BINS; k++) f[k] = POLE_BINS * f[k] / total;

    // Strip the alternation the same way pole_derive() does, leaving only what
    // distinguishes one pole from another.
    float fres[POLE_BINS], mlive[4];
    for (int t = 0; t < 4; t++)
    {
        float m = 0.0f;
        for (int k = t; k < POLE_BINS; k += 4) m += f[k];
        mlive[t] = m / (POLE_BINS / 4);
        for (int k = t; k < POLE_BINS; k += 4) fres[k] = f[k] / mlive[t];
    }

    const float *corr = motor_a ? pole_corr_a : pole_corr_b;
    const float *mtab = motor_a ? pole_mean_a : pole_mean_b;

    // Stage 1: alignment modulo 4, by matching the four per-transition means.
    //
    // Correlating the whole pattern against the table here does not work, and
    // fails silently. The alternation being sought is the *smaller* feature on
    // A -- 1.4% against 2.4% of pole residual -- so the residual acts as
    // structured noise and outvotes it. Simulated over all 44 boot positions on
    // the exact tables, that version picked the wrong base for 16 of A's and 12
    // of B's, and a wrong base puts the true shift outside stage 2's candidates
    // entirely, so it can never lock however long it gathers. Matching the
    // means instead recovers all 44 on both.
    int base = 0;
    float base_err = 1e30f;
    for (int s = 0; s < 4; s++)
    {
        float e = 0.0f;
        for (int t = 0; t < 4; t++)
        {
            float d = mlive[t] - mtab[(t + s) & 3];
            e += d * d;
        }
        if (e < base_err) { base_err = e; base = s; }
    }

    // Stage 2: which pole, from the residual.
    float best = -2.0f, second = -2.0f;
    int best_s = base;
    for (int i = 0; i < POLE_BINS / 4; i++)
    {
        int s = base + 4 * i;
        float r = pole_correlate(fres, corr, s);
        if (r > best) { second = best; best = r; best_s = s; }
        else if (r > second) { second = r; }
    }

    float quality = best - second;
    if (best < 0.5f || quality < POLE_MIN_QUALITY) return; // keep gathering

    float *live = motor_a ? pole_live_a : pole_live_b;
    for (int k = 0; k < POLE_BINS; k++) live[k] = corr[(k + best_s) % POLE_BINS];

    noInterrupts();
    if (motor_a) { pole_shift_a = best_s; pole_quality_a = quality; pole_locked_a = true; }
    else         { pole_shift_b = best_s; pole_quality_b = quality; pole_locked_b = true; }
    interrupts();
}


void pole_lock_update() {
    // A lost count breaks the mapping from count to shaft position, so the lock
    // is void and the gathered pattern is smeared. Start over.
    if (miss_a != miss_seen_a) { miss_seen_a = miss_a; pole_lock_reset(true); }
    if (miss_b != miss_seen_b) { miss_seen_b = miss_b; pole_lock_reset(false); }

    if (!pole_locked_a) pole_try_lock(true);
    if (!pole_locked_b) pole_try_lock(false);
}


PoleLock pole_lock_state(bool motor_a) {
    const volatile uint32_t* cnt = motor_a ? pole_cnt_a : pole_cnt_b;
    uint32_t n_min = 0xFFFFFFFFu;
    noInterrupts();
    for (int k = 0; k < POLE_BINS; k++) if (cnt[k] < n_min) n_min = cnt[k];
    interrupts();

    return PoleLock {
        motor_a ? pole_locked_a : pole_locked_b,
        motor_a ? pole_shift_a : pole_shift_b,
        motor_a ? pole_quality_a : pole_quality_b,
        n_min,
        motor_a ? miss_a : miss_b,
    };
}


// The live pattern as the matcher sees it. Printed raw so the comparison
// against the table can be done offline rather than guessed at from a score.
void pole_dump(bool motor_a)
{
    const volatile uint32_t *vsum = motor_a ? pole_sum_a : pole_sum_b;
    const volatile uint32_t *vcnt = motor_a ? pole_cnt_a : pole_cnt_b;

    uint32_t sum[POLE_BINS], cnt[POLE_BINS];
    noInterrupts();
    for (int k = 0; k < POLE_BINS; k++) { sum[k] = vsum[k]; cnt[k] = vcnt[k]; }
    interrupts();

    float f[POLE_BINS], total = 0.0f;
    for (int k = 0; k < POLE_BINS; k++)
    {
        f[k] = cnt[k] ? (float)sum[k] / (float)cnt[k] : 0.0f;
        total += f[k];
    }
    if (total <= 0.0f) return;

    Serial.print(motor_a ? "POLE A" : "POLE B");
    for (int k = 0; k < POLE_BINS; k++)
    {
        char v[12];
        snprintf(v, sizeof(v), ",%.4f", (double)(POLE_BINS * f[k] / total));
        Serial.print(v);
    }
    Serial.println();
}


WheelState sensor_get_wheels() {
    uint32_t current_us = micros();
    float speed_a = ((int32_t)(current_us - last_tick_a)) > SPEED_TICK_TIMEOUT_US ? 0 : wheel_a_speed_mms;
    float speed_b = ((int32_t)(current_us - last_tick_b)) > SPEED_TICK_TIMEOUT_US ? 0 : wheel_b_speed_mms;
    return WheelState {
        WheelSpeed {
            speed_a,
            speed_b
        },
        WheelPosition {
            enc_count_a,
            enc_count_b
        }
    };
}


static void write_imu_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
}


int init_imu() {
    if (!IMU.begin()) return 0;
    Wire1.setClock(400000);
    write_imu_reg(0x6B, 0x10, 0xB8);  // CTRL_REG1_G: ODR=101 (476Hz), FS=2000dps, BW=00
    write_imu_reg(0x6B, 0x20, 0xB0);  // CTRL_REG6_XL: ODR=101 (476Hz), FS=4g

    return 1;
}


void init_encoders() {

    pole_derive(POLE_FULL_A, pole_corr_a, pole_mean_a);
    pole_derive(POLE_FULL_B, pole_corr_b, pole_mean_b);
    pole_lock_reset(true);   // live tables start at 1.0 = uncorrected
    pole_lock_reset(false);

    pinMode(PIN_ENCODER_A1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_A2, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B2, INPUT_PULLUP);


    // Seed from the actual pin states: starting at 0 makes the first edge
    // decode as illegal, and a miss now means the pole lock is void. Seed the
    // timestamps too, or the first edge measures its interval from boot.
    last_tick_a = last_tick_b = micros();
    uint32_t in = NRF_P1->IN;
    prev_state_a = (((in >> NRF_PIN_ENCODER_A1) & 1) << 1) | ((in >> NRF_PIN_ENCODER_A2) & 1);
    prev_state_b = (((in >> NRF_PIN_ENCODER_B1) & 1) << 1) | ((in >> NRF_PIN_ENCODER_B2) & 1);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A1), encode_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A2), encode_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B1), encode_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B2), encode_b, CHANGE);
}


float sensor_read_battery() {
    uint64_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(BATT_V_PIN);
    }
    return (sum / 16.0f) * V_BATT_SCALE;
}


int get_raw_gyro(Vec3 &out) {
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(out.x, out.y, out.z);
        return 0;
    }
    return 1;
}


int get_raw_accel(Vec3 &out) {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(out.x, out.y, out.z);
        return 0;
    }
    return 1;
}

int sensor_poll_imu() {
    if (get_raw_gyro(gyr_raw)) return 1;
    if (get_raw_accel(acc_raw)) return 1;
    return 0;
}

void sensor_reset_position() {
    enc_count_a = 0;
    enc_count_b = 0;
}

int sensor_get_pitch(PitchMeasurement &out) {
    out.angle = atan2f(acc_raw.y, acc_raw.z) * 180.0f / PI;
    out.rate = -(gyr_raw.x - gyro_cal.x);
    // How far the total accel is from pure gravity = how hard `angle` is lying
    // right now. The measured noise floor is subtracted so plain sensor noise
    // reads as 0 and the filter keeps full baseline accel trust at rest.
    float norm = sqrtf(acc_raw.x*acc_raw.x + acc_raw.y*acc_raw.y + acc_raw.z*acc_raw.z);
    float dev = fabsf(norm - accel_norm_rest) - accel_dev_floor;
    out.accel_dev = dev > 0.0f ? dev : 0.0f;
    return 0;
}

float sensor_get_yaw_rate(float pitch_deg) {
    float t = pitch_deg * PI / 180.f;
    return (gyr_raw.y - gyro_cal.y) * sinf(t)
         + (gyr_raw.z - gyro_cal.z) * cosf(t);
}

int sensor_calibrate_imu() {
    float sum[3] = {0,0,0};
    float sumsq[3] = {0,0,0};
    float norm_sum = 0, norm_sumsq = 0;
    int norm_n = 0;
    Vec3 vals;
    Vec3 acc;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        while (get_raw_gyro(vals)) {}
        for (int j = 0; j < 3; j++) {
            sum[j] += vals[j];
            sumsq[j] += vals[j]*vals[j];
        }
        // Bot is still during calibration: average the resting accel magnitude
        // as the reference for PitchMeasurement.accel_dev.
        if (!get_raw_accel(acc)) {
            float norm = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
            norm_sum += norm;
            norm_sumsq += norm * norm;
            norm_n++;
        }
    }
    Vec3 mean_vals;
    for (int i = 0; i < 3; i++) {
        float mean = sum[i] / GYRO_CALIB_SAMPLES;
        float variance = sumsq[i] / GYRO_CALIB_SAMPLES - mean * mean;
        float stddev = sqrtf(variance);
        Serial.print("value ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(stddev);
        if (stddev >= GYRO_STDDEV) {
            return -1;
        }
        mean_vals[i] = mean;
    }
    for (int i = 0; i < 3; i++) {
        gyro_cal[i] = mean_vals[i];
    }
    if (norm_n > 0) {
        accel_norm_rest = norm_sum / norm_n;
        float var = norm_sumsq / norm_n - accel_norm_rest * accel_norm_rest;
        accel_dev_floor = 3.0f * sqrtf(var > 0.0f ? var : 0.0f);
    }
    return 0;
}


void encode_a() {
    uint32_t current = micros();
    uint32_t delta_us = (current - last_tick_a) + 1;
    uint32_t in = NRF_P1->IN;
    uint8_t s = (((in >> NRF_PIN_ENCODER_A1) & 1) << 1) | ((in >> NRF_PIN_ENCODER_A2) & 1);
    uint8_t idx = (prev_state_a << 2) | s;
    int8_t decoded = DECODE_TABLE[idx];

    if (decoded) {
        // Step the pole index with the count. Free-running: its offset from the
        // table is unknown until recovery finds it.
        int8_t k = (int8_t)pole_k_a + decoded;
        if (k >= POLE_BINS) k = 0;
        else if (k < 0) k = POLE_BINS - 1;
        pole_k_a = (uint8_t)k;

        float measured_speed = decoded * (EDGE_SCALE_A[idx] * pole_live_a[k] / delta_us);
        wheel_a_speed_mms = low_pass_speed_a.update(measured_speed);

        // Gather for recovery in one direction only: the reverse pattern is the
        // forward one offset by two bins, and mixing them would blur it.
        //
        // Reject intervals that cannot be geometry. Adjacent transitions differ
        // by at most ~2x (B's widest neighbouring pair), so 4x means the wheel
        // stalled, or this is the first edge and delta is measured from an
        // arbitrary zero. One such sample poisons its bin permanently -- the
        // accumulator never forgets -- and a single bad bin out of 44 is enough
        // to drop the correlation from 0.98 to 0.25. Compared by division
        // because 4 * delta_us overflows.
        if (!pole_locked_a && decoded > 0 && prev_delta_a &&
            (delta_us >> 2) < prev_delta_a && (prev_delta_a >> 2) < delta_us) {
            pole_sum_a[k] += delta_us;
            pole_cnt_a[k]++;
        }
        prev_delta_a = delta_us;
    }
    else {
        miss_a++;   // illegal transition: a count was lost, the lock is void
    }

    enc_count_a += decoded;
    prev_state_a = s;
    last_tick_a = current;
}


void encode_b() {
    uint32_t current = micros();
    uint32_t delta_us = (current - last_tick_b) + 1;
    uint32_t in = NRF_P1->IN;
    uint8_t s = (((in >> NRF_PIN_ENCODER_B1) & 1) << 1) | ((in >> NRF_PIN_ENCODER_B2) & 1);
    uint8_t idx = (prev_state_b << 2) | s;
    int8_t decoded = DECODE_TABLE[idx];

    if (decoded) {
        // Step the pole index with the count. Free-running: its offset from the
        // table is unknown until recovery finds it.
        int8_t k = (int8_t)pole_k_b + decoded;
        if (k >= POLE_BINS) k = 0;
        else if (k < 0) k = POLE_BINS - 1;
        pole_k_b = (uint8_t)k;

        float measured_speed = decoded * (EDGE_SCALE_B[idx] * pole_live_b[k] / delta_us);
        wheel_b_speed_mms = low_pass_speed_b.update(measured_speed);

        // Gather in one direction only, and reject non-geometry intervals; see
        // encode_a() for why both matter.
        if (!pole_locked_b && decoded > 0 && prev_delta_b &&
            (delta_us >> 2) < prev_delta_b && (prev_delta_b >> 2) < delta_us) {
            pole_sum_b[k] += delta_us;
            pole_cnt_b[k]++;
        }
        prev_delta_b = delta_us;
    }
    else {
        miss_b++;   // illegal transition: a count was lost, the lock is void
    }

    enc_count_b += decoded;
    prev_state_b = s;
    last_tick_b = current;
}

