#pragma once
#include "Arduino.h"
#include <math.h>
#include <stdint.h>

const float SAFETY_DRIVE_STALE_TICKS = 40; // balance loop is 200Hz timeout should be 200ms

enum class MotorState {
   OFF,
   ON
};

MotorState safety_update(float angle_deg, float angle_rate, float battery_voltage, uint32_t imu_misses);