#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"

void setup() {
  Serial.begin(115200);
  init_ble();
  init_imu();
  init_encoders();
  Serial.println("all init");
  while (sensor_calibrate_gyro()) {
    Serial.println("Keep the bot still, calibrating sensors");
  }
}

void loop() {
  com_poll();
  static uint32_t t_read = 0, t_print = 0;
  static Vec3 angles;
  static char buffer[100]; 
  if (ms_period(200, t_print)) {
    auto wheel_states = sensor_wheels();
    sprintf(buffer, "left: %.3f, right: %.3f", wheel_states.left.speed, wheel_states.right.speed);
    Serial.println(buffer);
  }

  if (ms_period(5, t_read)){
    angles = sensor_get_angles();
  }

  // if (ms_period(200, t_print)) {
  //   char msg_buff[255];
  //   sprintf(msg_buff, "Vec3 X: %.2f, Y: %.2f, Z: %.2f", angles.x, angles.y, angles.z);
  //   Serial.println(msg_buff);
  // }
}