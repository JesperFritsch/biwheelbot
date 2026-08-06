#include "Arduino.h"
#include <Arduino_LSM9DS1.h>

#include "utils.h"
#include "types.h"
#include "com.h"
#include "sensor.h"
#include "motor.h"

#include "sweep.h"
#include "kalman.h"

static KalmanFilter kf;

constexpr int BAR_WIDTH = 50;

// Writes a BAR_WIDTH-char bar into out (no terminator handling for the
// caller to worry about: returns chars written, leaves out[BAR_WIDTH] alone).
// -90 deg -> empty (0), +90 deg -> full (100), clamped outside that range.
static int render_bar(char *out, float angle_deg) {
  float pct = (angle_deg + 90.0f) * (100.0f / 180.0f);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  int fill = (int)(pct * (BAR_WIDTH / 100.0f) + 0.5f);
  for (int i = 0; i < BAR_WIDTH; i++) {
    out[i] = i < fill ? '#' : ' ';
  }
  return BAR_WIDTH;
}

void setup() {
  Serial.begin(115200);
  init_ble();
  init_imu();
  init_encoders();
  init_motors();
  analogWriteResolution(12);
  analogReadResolution(12);

  while (sensor_calibrate_gyro()) {
    Serial.println("Keep the bot still, calibrating sensors");
  }

  // run_full_sweep();  // disabled while testing the Kalman filter

  PitchMeasurement m0;
  while (sensor_get_pitch(m0)) {}
  float z0[KF_M] = { m0.angle, m0.rate };
  kalman_init(kf, z0);
}

void loop() {
  com_poll();
  static uint32_t t_read = 0, t_print = 0, t_batt = 0;
  static Vec3 angles;
  static char buffer[100];

  // Kalman filter test: run KF and a complementary filter on the exact same
  // measurements, print both for comparison.
  static PitchMeasurement m{};
  static float comp = 0;
  static bool comp_init = false;

  if (ms_period(5, t_read)) {
    kalman_predict(kf);
    if (!sensor_get_pitch(m)) {
      float z[KF_M] = { m.angle, m.rate };
      kalman_update(kf, z);

      if (!comp_init) { comp = m.angle; comp_init = true; }
      comp = COMP_BALANCE * (comp + m.rate * KF_DT) + (1 - COMP_BALANCE) * m.angle;
    }
  }

  if (ms_period(20, t_print)) {
    // comp vs kf as live bars: -90 deg -> 0, +90 deg -> 100 (clamped), one
    // '#' per 2%. Single line with \r so it redraws in place instead of
    // scrolling.
    static char line[140];
    int n = sprintf(line, "\rcomp [");
    n += render_bar(line + n, comp);
    n += sprintf(line + n, "]  kf [");
    n += render_bar(line + n, kf.x[0]);
    line[n++] = ']';
    line[n] = 0;
    Serial.print(line);
  }

  // if (ms_period(200, t_print)) {
  //   auto wheel_states = sensor_wheels();
  //   sprintf(buffer, "A: %05d, %.3f, B: %05d, %.3f", wheel_states.a.count, wheel_states.a.speed, wheel_states.b.count, wheel_states.b.speed);
  //   Serial.println(buffer);
  // }

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
  // static uint32_t total_count = 0; 
  // char log_buffer[100];

  // for (int i = 1; i <= 10; i++) {
  //   float duty = 0.1f * i;
  //   motor_set_b(duty);
  //   delay(300);

  //   int32_t c0 = sensor_wheels().b.count;
  //   uint32_t t0 = micros();
  //   delay(1000);
  //   int32_t c1 = sensor_wheels().b.count;
  //   uint32_t t1 = micros();

  //   float v_batt = sensor_read_battery();
  //   float dt = (t1 - t0) / 1e6f;
  //   float speed = (c1 - c0) * MM_PER_COUNT / dt;

  //   sprintf(log_buffer, "motor B v_batt: %.2f, duty: %.3f, mmps: %.2f", v_batt, duty, speed);
  //   Serial.println(log_buffer);
  // }

  // for (int i = 10; i > 0; i--) {
  //   float duty = 0.1f * i;
  //   motor_set_b(duty);
  //   delay(300);

  //   int32_t c0 = sensor_wheels().b.count;
  //   uint32_t t0 = micros();
  //   delay(1000);
  //   int32_t c1 = sensor_wheels().b.count;
  //   uint32_t t1 = micros();

  //   float v_batt = sensor_read_battery();
  //   float dt = (t1 - t0) / 1e6f;
  //   float speed = (c1 - c0) * MM_PER_COUNT / dt;

  //   sprintf(log_buffer, "motor B v_batt: %.2f, duty: %.3f, mmps: %.2f", v_batt, duty, speed);
  //   Serial.println(log_buffer);
  // }

}