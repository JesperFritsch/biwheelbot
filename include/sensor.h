#pragma once
#include <stdint.h>
#include "types.h"

int init_imu();
void init_encoders();
void init_adc();
float sensor_read_battery();
int sensor_calibrate_gyro();
Vec3 sensor_get_angles();
WheelStates sensor_wheels();