#pragma once

// #include <mbed.h>
#include "Arduino.h"

constexpr float sym_cap(float val, float lim) {
    return val > lim ? lim : (val < -lim ? -lim : val);
}

inline bool ms_period(uint32_t period_ms, uint32_t &timestamp) {
  uint32_t current = millis();
  if (period_ms <= current - timestamp) {
    timestamp = current;
    return true;
  }
  return false;
}

class LowPassFilter {
public: 
    LowPassFilter(float steps, float initial=0.0f) : steps(steps), value(initial) {}
    float update(float new_value) {
        value += (new_value - value) / steps;
        return value;
    }
private:
    uint8_t steps;
    float value;
};