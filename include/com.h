#pragma once
#include "types.h"

struct ComHooks {
    void (*set_gains)(GainId, PIDGains);
    PIDGains (*get_gains)(GainId);
    void (*set_drive)(DriveCmd);
};

// Number of f32 in one telemetry packet. Must match the field count in the
// 0x2901 schema string in com.cpp and the pack order in main.cpp's loop().
constexpr int TELEM_FIELDS = 16;

void com_set_hooks(ComHooks hooks);
void init_ble();
void com_poll();
void com_publish_telemetry(const float *values, int count);
