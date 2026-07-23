#pragma once

#include "Arduino.h"

inline bool ms_period(uint32_t period_ms, uint32_t &timestamp) {
  uint32_t current = millis();
  if (period_ms <= current - timestamp) {
    timestamp = current;
    return true;
  }
  return false;
}