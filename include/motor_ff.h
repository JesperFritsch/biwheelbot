#pragma once
#include <math.h>
#include <stdbool.h>

#define FF_N 20

static const float ff_table[FF_N] = {
    0.000f, 0.35f, 0.386f, 0.422f, 0.458f, 0.494f, 0.531f, 0.567f, 0.603f, 0.639f, 
    0.675f, 0.711f, 0.747f, 0.783f, 0.819f, 0.856f, 0.892f, 0.928f, 0.964f, 1.0f
};

// interpolates the table above to map a requested effort u in [0,1] onto the duty that actually delivers u of full speed.
static inline float ff_duty(float u) {
    if (u == 0.0f) return 0.0f;

    bool forward = (u > 0.0f);
    float mag = fabsf(u);
    if (mag > 1.0f) mag = 1.0f;

    float x = mag * (FF_N - 1);
    int i = (int)x;
    if (i > FF_N - 2) i = FF_N - 2;
    float duty = ff_table[i] + (x - (float)i) * (ff_table[i + 1] - ff_table[i]);

    if (duty > 1.0f) duty = 1.0f;
    return forward ? duty : -duty;
}

// static inline float ff_duty(float u) {
//     if (u > 0) {
//         return std::pow(u, 0.3);
//     }
//     return -std::pow(fabsf(u), 0.3);
// }