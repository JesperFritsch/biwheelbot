
#include "sensor.h"
#include "types.h"
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include "Arduino.h"
#include <algorithm>


#define PIN_ENCODER_A1 6
#define PIN_ENCODER_A2 5
#define PIN_ENCODER_B1 3
#define PIN_ENCODER_B2 4


#define GYRO_CALIB_SAMPLES 128
#define SPEED_TICK_TIMEOUT_US 100000

constexpr float GYRO_STDDEV = 0.17f; // standard deviation threshold, if lower than this then the bot is completely still.
constexpr float COMP_BALANCE = 0.95f; // What balance to use in the complimentary filter

constexpr float WHEEL_CIRCUM_MM = 226.0f;
constexpr float COUNTS_PER_REV = 425.0f;
constexpr float MM_PER_COUNT = WHEEL_CIRCUM_MM / COUNTS_PER_REV;

static const int8_t DECODE_TABLE[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
    -1,  0,  0,  1,
    0,  1, -1,  0
};

static volatile int32_t motor_a = 0, motor_b = 0;
static volatile float motor_a_speed_mms = 0, motor_b_speed_mms = 0;
static volatile uint8_t prev_state_a = 0, prev_state_b = 0;
static volatile uint32_t last_tick_a = 0, last_tick_b = 0; 

Vec3 gyro_cal{};

void encode_a();
void encode_b();


WheelStates sensor_wheels() {
    uint32_t current_us = micros();
    float speed_a = current_us - last_tick_a > SPEED_TICK_TIMEOUT_US ? 0 : motor_a_speed_mms;
    float speed_b = current_us - last_tick_b > SPEED_TICK_TIMEOUT_US ? 0 : motor_b_speed_mms;
    return WheelStates {
        WheelState {
            speed_a,
            motor_a,
        },
        WheelState {
            speed_b,
            motor_b
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

    write_imu_reg(0x6B, 0x10, 0xB8);  // CTRL_REG1_G: ODR=101 (476Hz), FS=2000dps, BW=00
    write_imu_reg(0x6B, 0x20, 0xB0);  // CTRL_REG6_XL: ODR=101 (476Hz), FS=4g

    return 1;
}


void init_encoders() {

    pinMode(PIN_ENCODER_A1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_A2, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B2, INPUT_PULLUP);


    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A1), encode_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A2), encode_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B1), encode_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B2), encode_b, CHANGE);
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


Vec3 sensor_get_angles() {
    static uint32_t timestamp = 0;
    static bool initialized = false;
    static Vec3 prev{};
    static Vec3 gyr{};
    static Vec3 acc{};
    uint32_t time_us = micros();
    uint32_t period_us = time_us - timestamp;
    if (get_raw_gyro(gyr)) {
        Serial.println("Angle measurement not available");
    }
    if (get_raw_accel(acc)) {
        Serial.println("Accel measurement not available");
    }

    gyr.x -= gyro_cal.x;
    gyr.y -= gyro_cal.y;
    gyr.z -= gyro_cal.z;

    float accX = (atan2f(acc.y, acc.z) * 180.0f / PI);
    float accY = (atan2f(acc.x, acc.z) * 180.0f / PI);
    float accZ = (atan2f(acc.y, acc.x) * 180.0f / PI);
    
    if (!initialized) {
        prev = {accX, accY, accZ};
        timestamp = time_us;
        initialized = true;
        return prev;
    }
    
    Vec3 out;
    out.x = COMP_BALANCE * (prev.x + (-gyr.x * (period_us / 1000000.0f))) + (1 - COMP_BALANCE) * accX;
    out.y = COMP_BALANCE * (prev.y + (-gyr.y * (period_us / 1000000.0f))) + (1 - COMP_BALANCE) * accY;
    out.z = COMP_BALANCE * (prev.z + (-gyr.z * (period_us / 1000000.0f))) + (1 - COMP_BALANCE) * accZ;
    prev = out;
    timestamp = time_us;
    return out;
}


int sensor_calibrate_gyro() {
    float sum[3] = {0,0,0};
    float sumsq[3] = {0,0,0};
    Vec3 vals;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        while (get_raw_gyro(vals)) {}
        for (int j = 0; j < 3; j++) {
            sum[j] += vals[j];
            sumsq[j] += vals[j]*vals[j];
        }
    }
    Vec3 mean_vals;
    for (int i = 0; i < 3; i++) {
        float mean = sum[i] / GYRO_CALIB_SAMPLES;
        float variance = sumsq[i] / GYRO_CALIB_SAMPLES - mean * mean;
        float stddev = sqrtf(variance);
        if (stddev >= GYRO_STDDEV) {
            return -1;
        }
        mean_vals[i] = mean;
    }
    for (int i = 0; i < 3; i++) {
        gyro_cal[i] = mean_vals[i];
    }
    return 0;
}


void encode_a() {
    uint32_t current = micros();
    uint32_t delta_us = current - last_tick_a;
    uint8_t s = (digitalRead(PIN_ENCODER_A1) << 1) | digitalRead(PIN_ENCODER_A2);
    int8_t decoded = DECODE_TABLE[(prev_state_a << 2) | s]; // subtract to get both encoders to count the same direction
    motor_a_speed_mms = decoded * (MM_PER_COUNT * 1000000 / delta_us);
    motor_a += decoded;
    prev_state_a = s;
    last_tick_a = current;
}


void encode_b() {
    uint32_t current = micros();
    uint32_t delta_us = current - last_tick_b;
    uint8_t s = (digitalRead(PIN_ENCODER_B1) << 1) | digitalRead(PIN_ENCODER_B2);
    int8_t decoded = DECODE_TABLE[(prev_state_b << 2) | s];
    motor_b_speed_mms = decoded * (MM_PER_COUNT * 1000000.f / delta_us);
    motor_b += decoded;
    prev_state_b = s;
    last_tick_b = current;
}

