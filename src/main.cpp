#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"

void setup() {
  Serial.begin(115200);
  init_ble();
  init_imu();
  init_encoders();
  init_adc();
  while (sensor_calibrate_gyro()) {
    Serial.println("Keep the bot still, calibrating sensors");
  }
}

void loop() {
  com_poll();
  static uint32_t t_read = 0, t_print = 0, t_batt = 0;
  static Vec3 angles;
  static char buffer[100]; 
  if (ms_period(200, t_print)) {
    auto wheel_states = sensor_wheels();
    sprintf(buffer, "A: %05d, %.3f, B: %05d, %.3f", wheel_states.a.count, wheel_states.a.speed, wheel_states.b.count, wheel_states.b.speed);
    Serial.println(buffer);
  }

  // if (ms_period(5, t_read)){
  //   angles = sensor_get_angles();
  // }

  // if (ms_period(100, t_batt)) {
  //   float raw = sensor_read_battery();
  //   Serial.println(raw);
  // }

  // if (ms_period(200, t_print)) {
  //   char msg_buff[255];
  //   sprintf(msg_buff, "Vec3 X: %.2f, Y: %.2f, Z: %.2f", angles.x, angles.y, angles.z);
  //   Serial.println(msg_buff);
  // }

  Serial.println("Running sweep");

  char log_buffer[100];
  for (int i = 1; i <= 10; i++) {
    float duty = 0.1 * i;
    motor_set_b(duty);
    delay(700);
    float v_batt = sensor_read_battery();
    auto wheel_data = sensor_wheels();
    sprintf(log_buffer, "motor A v_batt: %.2f, duty: %.3f, mmps: %.2f", v_batt, duty, wheel_data.b.speed);
    Serial.println(log_buffer);
  }

  for (int i = 10; i > 0; i--) {
    float duty = 0.1 * i;
    motor_set_b(duty);
    delay(700);
    float v_batt = sensor_read_battery();
    auto wheel_data = sensor_wheels();
    sprintf(log_buffer, "motor A v_batt: %.2f, duty: %.3f, mmps: %.2f", v_batt, duty, wheel_data.b.speed);
    Serial.println(log_buffer);
  }

}