#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"
#include "control.h"

#include "sweep.h"

void setup()
{
    Serial.begin(115200);

    com_set_hooks({
        .set_gains = control_set_gains,
        .get_gains = control_get_gains,
        .set_drive = control_set_drive
    });

    init_ble(); 
    init_imu();
    init_encoders();
    init_motors();
    analogWriteResolution(12);
    analogReadResolution(12);

    while (sensor_calibrate_imu())
    {
        Serial.println("Keep the bot still, calibrating sensors");
    }

    set_motors_enabled(true);
    control_start();
}

void loop()
{
    com_poll();

    static uint32_t t_telem(0);
    if (ms_period(10, t_telem)) {   // 40 Hz -- the link cannot carry the 200 Hz tick
        auto snap = control_get_snapshot();
        auto wheel_state = sensor_get_wheels();
        const float telem[] = {
            snap.angle,
            snap.rate,
            snap.kf_angle,
            snap.t_pos_mm,
            snap.pos_mm,
            snap.target_angle,
            snap.target_speed,
            snap.effort_duty,
            wheel_state.speed.avg_speed(),
            snap.motor_a_duty,
            snap.motor_b_duty,
            snap.turning_duty,
            snap.battery_voltage,
            snap.motors_enabled ? 1.0f : 0.0f,
            (float)snap.overruns
        };
        static_assert(sizeof(telem) / sizeof(telem[0]) == TELEM_FIELDS,
                      "telemetry pack order is out of step with the schema");
        com_publish_telemetry(telem, TELEM_FIELDS);
    }

    pole_lock_update();
}
