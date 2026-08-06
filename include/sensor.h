#pragma once
#include <stdint.h>
#include "types.h"


constexpr float V_BATT_SCALE = 0.003429327f;

constexpr float GYRO_STDDEV = 0.5f; // standard deviation threshold, if lower than this then the bot is completely still.
constexpr float COMP_BALANCE = 0.95f; // What balance to use in the complimentary filter

constexpr float WHEEL_CIRCUM_MM = 226.0f;
constexpr float COUNTS_PER_REV = 425.0f;
constexpr float MM_PER_COUNT = WHEEL_CIRCUM_MM / COUNTS_PER_REV;


int init_imu();
void init_encoders();
void init_adc();
float sensor_read_battery();
int sensor_calibrate_gyro();
Vec3 sensor_get_angles();
int sensor_get_pitch(PitchMeasurement &out);
WheelStates sensor_wheels();