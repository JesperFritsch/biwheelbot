#pragma once
#include "Arduino.h"
#include <math.h>
#include <stdint.h>

enum class MotorState {
   OFF,
   ON
};

MotorState safety_update(float angle_deg, float angle_rate, float battery_voltage, uint32_t imu_misses);