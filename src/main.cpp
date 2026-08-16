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
        .set_balance_gains = set_balance_gains,
        .get_balance_gains = get_balance_gains,
        .set_speed_gains = set_speed_gains,
        .get_speed_gains = get_speed_gains,
        .set_pos_gains = set_pos_gains,
        .get_pos_gains = get_pos_gains
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

    // Background: hunts for where the encoders sit in the pole table, and starts
    // over by itself if a count is ever lost. Costs nothing once locked.
    // static uint32_t t_print(0);
    // if (ms_period(10, t_print)) {
    //     char buffer[256];
    //     auto b_gains = get_balance_gains();
    //     auto wheel_state = sensor_get_wheels();
    //     auto snap = control_get_snapshot();
    //     sprintf(buffer, "angle=%6.2f, rate=%6.2f, t_pos=%7.1f, pos=%7.1f n_θ=%6.2f, n_s=%6.2f, n_duty=%6.2f, t_s=%6.2f, t_duty=%6.2f, bat=%5.2f, mr=%s, kp=%6.2f, kd=%6.2f",
    //         snap.angle, snap.rate, snap.t_pos_mm, snap.pos_mm, snap.target_angle, snap.target_speed, snap.effort_duty, wheel_state.speed.avg_speed(), snap.motor_a_duty, snap.battery_voltage,
    //         snap.motors_enabled ? "ON" : "OFF", b_gains.kp, b_gains.kd);
    //     Serial.println(buffer);
    // }

    // Telemetry over BLE. Order must match the 0x2901 schema in com.cpp:
    //   telem:ang,rate,kfa,cmp,tpos,pos,tang,tspd,eff,spd,duty,bat,en,ovr
    static uint32_t t_telem(0);
    if (ms_period(10, t_telem)) {   // 40 Hz -- the link cannot carry the 200 Hz tick
        auto snap = control_get_snapshot();
        auto wheel_state = sensor_get_wheels();
        const float telem[] = {
            snap.angle,
            snap.rate,
            snap.kf_angle,
            snap.comp_angle,
            snap.t_pos_mm,
            snap.pos_mm,
            snap.target_angle,
            snap.target_speed,
            snap.effort_duty,
            wheel_state.speed.avg_speed(),
            snap.motor_a_duty,
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
