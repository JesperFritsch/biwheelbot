#pragma once
#include <stdint.h>
#include "types.h"

int init_imu();
int sensor_calibrate_gyro();
Vec3 sensor_get_angles();
void init_encoders();
WheelStates sensor_wheels();