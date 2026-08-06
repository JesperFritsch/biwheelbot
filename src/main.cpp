#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"
#include "control.h"

#include "sweep.h"


void setup() {
  Serial.begin(115200);
  init_ble();
  init_imu();
  init_encoders();
  init_motors();
  analogWriteResolution(12);
  analogReadResolution(12);

  while (sensor_calibrate_imu()) {
    Serial.println("Keep the bot still, calibrating sensors");
  }

  set_motors_enabled(true);

  // run_full_sweep();  // disabled while testing the Kalman filter

  control_start();

}

void loop() {
  com_poll();

  static uint32_t t_print = 0;
  static char buffer[100];
  
  if (ms_period(100, t_print)) {
    auto snap = control_get_snapshot();   
    snprintf(buffer, sizeof(buffer), "Angle: %.2f, Rate: %.2f, Battery: %.2f, motors_enabled: %s", snap.angle, snap.rate, snap.battery_voltage, snap.motors_enabled ? "true" : "false");
    Serial.println(buffer);
  }
}