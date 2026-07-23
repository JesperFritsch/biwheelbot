#pragma once

struct Vec3 {
    float x, y, z;
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }
};


struct WheelState {
    float speed; // mm per second
    int32_t count;
};


struct WheelStates {
    WheelState a;
    WheelState b;
};