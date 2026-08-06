#pragma once

// Linear Kalman filter for pitch estimation.
// State x = [theta, theta_dot], measurement z = [theta_accel, theta_dot_gyro].
// Everything in degrees / deg/s — convert at the sensor boundary, never here.

constexpr int KF_N = 2;         // state dimension
constexpr int KF_M = 2;         // measurement dimension
constexpr float KF_DT = 0.005f; // fixed filter period, seconds (200 Hz)

struct KalmanFilter {
    float x[KF_N];        // state estimate
    float P[KF_N][KF_N];  // estimate covariance
};

void kalman_init(KalmanFilter &kf, const float z0[KF_M]);
void kalman_predict(KalmanFilter &kf);
// R is per-update so it can be scheduled on measurement quality: build it with
// kalman_measurement_R from the current accel-magnitude deviation.
void kalman_update(KalmanFilter &kf, const float z[KF_M], const float R[KF_M]);
void kalman_measurement_R(float accel_dev, float R_out[KF_M]);
