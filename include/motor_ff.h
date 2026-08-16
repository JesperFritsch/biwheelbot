#pragma once
#include <math.h>
#include <stdbool.h>

#define FF_N 20

static const float ff_table[FF_N] = {
    0.000f, 0.55f, 0.575f, 0.6f, 0.625f, 0.65f, 0.675f, 0.7f, 0.725f, 0.75f, 
    0.775f, 0.8f, 0.825f, 0.85f, 0.875f, 0.9f, 0.925f, 0.95f, 0.975f, 1.0f
};

// interpolates the table above to map a requested effort u in [0,1] onto the duty that actually delivers u of full speed.
// static inline float ff_duty(float u) {
//     if (u == 0.0f) return 0.0f;

//     bool forward = (u > 0.0f);
//     float mag = fabsf(u);
//     if (mag > 1.0f) mag = 1.0f;

//     float x = mag * (FF_N - 1);
//     int i = (int)x;
//     if (i > FF_N - 2) i = FF_N - 2;
//     float duty = ff_table[i] + (x - (float)i) * (ff_table[i + 1] - ff_table[i]);

//     if (duty > 1.0f) duty = 1.0f;
//     return forward ? duty : -duty;
// }

static inline float ff_duty(float u) {
    if (u > 0) {
        return std::pow(u, 0.3);
    }
    return -std::pow(fabsf(u), 0.3);
}