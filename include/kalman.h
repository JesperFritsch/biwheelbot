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
void kalman_update(KalmanFilter &kf, const float z[KF_M]);
