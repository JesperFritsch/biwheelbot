#pragma once
#include <math.h>
#include <stdint.h>

// Feedforward duty tables, measured 2026-07 at ~11.55 V pack, 1 kHz PWM,
// wheels free-spinning. speed in mm/s, index i -> duty = DUTY_MIN + i*DUTY_STEP.

#define SPD_N 33
static const float FF_DUTY_MIN  = 0.200f;
static const float FF_DUTY_STEP = 0.025f;
static const float FF_V_REF     = 11.55f;   // pack voltage during calibration

static const float SPEED_A_FWD[SPD_N] = {
       0.0f,   137.7f,   372.0f,   593.2f,   725.1f,   848.9f,   957.2f,  1052.1f,
    1147.0f,  1217.5f,  1287.9f,  1344.5f,  1401.0f,  1445.3f,  1489.7f,  1529.2f,
    1568.7f,  1598.9f,  1629.1f,  1657.0f,  1685.0f,  1711.1f,  1737.2f,  1757.7f,
    1778.2f,  1796.9f,  1815.7f,  1833.3f,  1850.9f,  1871.3f,  1891.8f,  1955.6f,
    2019.4f,
};

static const float SPEED_A_REV[SPD_N] = {
       0.0f,   116.2f,   206.9f,   274.7f,   638.4f,   774.3f,   889.4f,   997.0f,
    1104.7f,  1176.1f,  1247.5f,  1309.9f,  1372.2f,  1426.6f,  1481.0f,  1522.3f,
    1563.7f,  1597.9f,  1632.2f,  1662.7f,  1693.2f,  1719.5f,  1745.8f,  1766.7f,
    1787.6f,  1806.6f,  1825.6f,  1844.7f,  1863.8f,  1887.9f,  1911.9f,  1981.5f,
    2051.0f,
};

static const float SPEED_B_FWD[SPD_N] = {
      98.6f,   178.7f,   255.2f,   655.9f,   783.5f,   897.4f,  1000.3f,  1107.4f,
    1214.6f,  1285.0f,  1355.4f,  1410.8f,  1466.1f,  1512.8f,  1559.4f,  1602.5f,
    1645.5f,  1676.4f,  1707.2f,  1732.6f,  1758.0f,  1779.4f,  1800.8f,  1820.2f,
    1839.7f,  1856.5f,  1873.5f,  1890.3f,  1907.2f,  1928.6f,  1950.0f,  2010.1f,
    2070.2f,
};

static const float SPEED_B_REV[SPD_N] = {
      85.6f,   236.9f,   486.0f,   634.4f,   761.2f,   875.8f,   975.2f,  1071.9f,
    1168.5f,  1242.6f,  1316.7f,  1370.1f,  1423.5f,  1469.1f,  1514.7f,  1552.1f,
    1589.5f,  1621.2f,  1653.0f,  1678.0f,  1703.1f,  1726.0f,  1749.0f,  1773.2f,
    1797.4f,  1813.7f,  1830.1f,  1847.4f,  1864.6f,  1882.2f,  1899.8f,  1961.2f,
    2022.5f,
};

// Stiction thresholds, measured. Breakaway = duty needed to start from rest.
// Dropout = duty below which an already-turning wheel stalls.
typedef struct {
    const float* table;
    float breakaway;
    float dropout;
} FFProfile;

static const FFProfile FF_A_FWD = { SPEED_A_FWD, 0.250f, 0.205f };
static const FFProfile FF_A_REV = { SPEED_A_REV, 0.285f, 0.210f };
static const FFProfile FF_B_FWD = { SPEED_B_FWD, 0.265f, 0.185f };
static const FFProfile FF_B_REV = { SPEED_B_REV, 0.225f, 0.190f };

static inline const FFProfile* ff_profile(bool motor_a, bool forward) {
    if (motor_a) return forward ? &FF_A_FWD : &FF_A_REV;
    else         return forward ? &FF_B_FWD : &FF_B_REV;
}

// Slowest speed this motor/direction can actually hold, given the dropout duty.
static inline float ff_min_speed(const FFProfile* p) {
    int i = (int)((p->dropout - FF_DUTY_MIN) / FF_DUTY_STEP);
    if (i < 0) i = 0;
    if (i >= SPD_N) i = SPD_N - 1;
    return p->table[i];
}

// Inverse lookup: desired |speed| in mm/s -> duty magnitude, before battery
// compensation and before the stiction floor is applied.
static float ff_duty_raw(const float* tbl, float target) {
    if (target <= tbl[0]) return FF_DUTY_MIN;
    for (int i = 1; i < SPD_N; i++) {
        if (target <= tbl[i]) {
            float span = tbl[i] - tbl[i - 1];
            float f = (span > 1e-6f) ? (target - tbl[i - 1]) / span : 0.0f;
            return FF_DUTY_MIN + FF_DUTY_STEP * ((float)(i - 1) + f);
        }
    }
    return 1.0f;
}

/*
 * Feedforward duty for a commanded wheel speed.
 *
 *   target_mmps : signed desired speed
 *   measured_mmps: signed current speed (used to pick breakaway vs dropout floor)
 *   v_batt      : current pack voltage from sensor_read_battery()
 *   motor_a     : true for motor A
 *
 * Returns signed duty in [-1, 1]. Zero means "command nothing" - the requested
 * speed is below what this drivetrain can produce.
 */
static inline float ff_duty(float target_mmps, float measured_mmps,
                            float v_batt, bool motor_a) {
    bool forward = (target_mmps >= 0.0f);
    const FFProfile* p = ff_profile(motor_a, forward);

    float mag = fabsf(target_mmps);

    // Below the slowest speed this drivetrain can hold, don't pretend.
    if (mag < ff_min_speed(p) * 0.5f) return 0.0f;

    float duty = ff_duty_raw(p->table, mag);

    // Compensate for pack voltage drift away from the calibration point.
    if (v_batt > 1.0f) duty *= (FF_V_REF / v_batt);

    // Stiction floor: a stationary wheel needs breakaway, a moving one only
    // needs dropout. Using breakaway always would throw away the low end;
    // using dropout always would fail to start.
    bool moving = fabsf(measured_mmps) > 20.0f;
    float floor_duty = moving ? p->dropout : p->breakaway;
    if (duty < floor_duty) duty = floor_duty;

    if (duty > 1.0f) duty = 1.0f;
    return forward ? duty : -duty;
}