#pragma once
#include <stdint.h>
#include "types.h"


constexpr float V_BATT_SCALE = 0.003429327f;

constexpr float GYRO_STDDEV = 1.0f; // standard deviation threshold, if lower than this then the bot is completely still.

// The encoders' edge geometry repeats every 44 counts -- one motor revolution,
// 11 pole pairs x 4 quadrature edges. Measured, not assumed; see
// docs/encoder-calibration.md for the method and the numbers.
#define POLE_BINS 44


int init_imu();
void init_encoders();
float sensor_read_battery();
int sensor_calibrate_imu();
int sensor_get_pitch(PitchMeasurement &out);
float sensor_get_yaw_rate(float pitch_deg);
int sensor_poll_imu();
WheelState sensor_get_wheels();
void sensor_reset_position();

// Where the free-running pole index sits relative to the calibration table.
//
// The table is fixed to the magnet, but the count starts at zero wherever the
// shaft happens to be at boot, so the alignment between them is unknown and
// differs every power-up. There is no index pulse to ask, so it is recovered by
// correlating live measurements against the table while the wheels turn.
//
// Until locked, speeds carry the uncorrected pole error -- a few percent, and
// nothing else is affected, because the larger channel-phase correction is
// keyed on the quadrature state and needs no alignment.
struct PoleLock {
    bool locked;
    uint8_t shift;     // table rotation in use, 0..43
    float quality;     // correlation peak minus runner-up; higher is safer
    uint32_t samples;  // smallest per-bin sample count gathered so far
    uint32_t misses;   // illegal transitions; any at all voids the lock
};

// Cheap when there is nothing to do. Call from loop(), not the control tick --
// it is background work and the correlation takes a few tens of microseconds.
void pole_lock_update();

PoleLock pole_lock_state(bool motor_a);

// Drop the lock and start gathering again. Done automatically on a missed
// count, since a lost count breaks the mapping from count to shaft position.
void pole_lock_reset(bool motor_a);

// Print the accumulated live pattern as CSV, normalised the same way the table
// is, for offline comparison against it (tools/polesim.py --match).
void pole_dump(bool motor_a);
