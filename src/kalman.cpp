#include "kalman.h"

#include <math.h>

// Baseline measurement noise R (diagonal), same units as the state: deg^2 and
// (deg/s)^2. Valid while the bot is quiet. Starting guesses — replace with
// variances computed from a stationary bench log (a few thousand samples).
static const float R_DIAG[KF_M] = { 1.0f, 0.16f };

// Scheduled-R gain for the accel channel, deg^2 per g^2. accel_dev is how much
// non-gravity acceleration is corrupting the accel tilt angle right now, so
// inflating R[0] with its square makes the filter smoothly stop trusting the
// accel while the bot is shoved or translated, and coast on the gyro instead.
// Physically derived: a lateral accel of dev [g] fakes a tilt of
// atan(dev) ~ 57.3*dev deg; treating that worst case as the 1-sigma error gives
// R inflation (57.3*dev)^2 ~ 3300*dev^2.
static const float ACCEL_MOTION_GAIN = 3300.0f;

// The magnitude test above has a blind spot: lateral acceleration adds to
// gravity in quadrature, so it barely moves |a| while faking a large tilt. A
// 0.1 g shove fakes 5.7 deg of tilt and changes the magnitude by 0.5%, inflating
// R by 8% -- effectively nothing.
//
// The innovation catches what the magnitude misses, because it consults the
// gyro instead of the accelerometer. Over a single 5 ms tick the gyro cannot be
// wrong by several degrees, so a large disagreement means the accelerometer is
// reading translation, not tilt.
//
// 3 deg sits comfortably outside both legitimate sources of innovation: real
// tilt manages about 1 deg per tick even at 200 deg/s, and the accel's own
// noise is ~1 deg (R_DIAG[0] = 1.0).
static const float INNOV_GATE_DEG = 3.0f;

void kalman_measurement_R(float accel_dev, float innovation, float R_out[KF_M]) {
    R_out[0] = R_DIAG[0] + ACCEL_MOTION_GAIN * accel_dev * accel_dev;

    // Linear past the gate, deliberately. Inflating with the square would make
    // the correction K*y peak and then shrink as the innovation grows, which
    // locks the accelerometer out entirely if the state is genuinely wrong.
    // Linear makes the correction saturate instead, so the filter always
    // recovers -- slowly, but it recovers.
    float a = fabsf(innovation);
    if (a > INNOV_GATE_DEG) {
        R_out[0] *= a / INNOV_GATE_DEG;
    }

    R_out[1] = R_DIAG[1];
}

// Process noise Q (diagonal), hand-tuned on the robot. Acts as a floor on P:
// too low -> filter goes overconfident and reacts sluggishly to shoves,
// too high -> raw sensor noise passes through to the control loop.
// Q[1] is large because the constant-velocity model is a poor fit while the
// motors are actively torquing the body around.
//
// Note Q[1] has almost no effect on how much accelerometer reaches the angle:
// the gyro measurement pins P[1][1] directly, and the leakage into P[0][0] is
// dt^2 * Q[1] = 0.000625, negligible beside Q[0]. Q[0] is the knob for that.
//
// Q[0] stays at 0.005 despite that putting the at-rest accel weight at 0.068,
// against 0.030 for a complementary filter at alpha = 0.97. The at-rest figure
// is the wrong comparison: while balancing, accel_dev is continuously nonzero
// and the schedule already pulls the effective weight to ~0.035 at dev = 0.03
// and ~0.020 at dev = 0.06. Dropping Q[0] to 0.001 would put those at 0.016 and
// 0.009 -- a third of the complementary filter during exactly the motion that
// matters, which is the lag recorded in docs/kalman-filter.md.
static const float Q_DIAG[KF_N] = { 0.005f, 25.0f };

// --- The four EKF touch-points ---------------------------------------------
// Trivial in the linear filter. Upgrading to an EKF means replacing these with
// the nonlinear f/h and their Jacobians; the predict/update code below stays.

static void predict_state(const float x[KF_N], float out[KF_N]) {
    out[0] = x[0] + KF_DT * x[1];
    out[1] = x[1];
}

static void get_F(const float x[KF_N], float F[KF_N][KF_N]) {
    (void)x;
    F[0][0] = 1.0f; F[0][1] = KF_DT;
    F[1][0] = 0.0f; F[1][1] = 1.0f;
}

static void predict_measurement(const float x[KF_N], float out[KF_M]) {
    out[0] = x[0];
    out[1] = x[1];
}

static void get_H(const float x[KF_N], float H[KF_M][KF_N]) {
    (void)x;
    H[0][0] = 1.0f; H[0][1] = 0.0f;
    H[1][0] = 0.0f; H[1][1] = 1.0f;
}

// ---------------------------------------------------------------------------

void kalman_init(KalmanFilter &kf, const float z0[KF_M]) {
    kf.x[0] = z0[0];
    kf.x[1] = z0[1];
    for (int i = 0; i < KF_N; i++)
        for (int j = 0; j < KF_N; j++)
            kf.P[i][j] = 0.0f;
    // State starts as a copy of a measurement, so its uncertainty is R.
    kf.P[0][0] = R_DIAG[0];
    kf.P[1][1] = R_DIAG[1];
}

void kalman_predict(KalmanFilter &kf) {
    float F[KF_N][KF_N];
    get_F(kf.x, F);

    float x_new[KF_N];
    predict_state(kf.x, x_new);

    // P = F P F^T + Q
    float FP[KF_N][KF_N];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += F[i][k] * kf.P[k][j];
            FP[i][j] = s;
        }
    }
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += FP[i][k] * F[j][k];
            kf.P[i][j] = s;
        }
    }
    for (int i = 0; i < KF_N; i++) kf.P[i][i] += Q_DIAG[i];

    for (int i = 0; i < KF_N; i++) kf.x[i] = x_new[i];
}

void kalman_update(KalmanFilter &kf, const float z[KF_M], const float R[KF_M]) {
    float H[KF_M][KF_N];
    get_H(kf.x, H);

    // S = H P H^T + R
    float HP[KF_M][KF_N];
    for (int i = 0; i < KF_M; i++) {
        for (int j = 0; j < KF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += H[i][k] * kf.P[k][j];
            HP[i][j] = s;
        }
    }
    float S[KF_M][KF_M];
    for (int i = 0; i < KF_M; i++) {
        for (int j = 0; j < KF_M; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += HP[i][k] * H[j][k];
            S[i][j] = s;
        }
        S[i][i] += R[i];
    }

    static_assert(KF_M == 2, "S inverse below is hard-coded for 2x2");
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    float Sinv[KF_M][KF_M] = {
        {  S[1][1] / det, -S[0][1] / det },
        { -S[1][0] / det,  S[0][0] / det },
    };

    // K = P H^T S^-1
    float PHt[KF_N][KF_M];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_M; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += kf.P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    }
    float K[KF_N][KF_M];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_M; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_M; k++) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }
    }

    // x = x + K (z - h(x))
    float zhat[KF_M];
    predict_measurement(kf.x, zhat);
    float y[KF_M];
    for (int i = 0; i < KF_M; i++) y[i] = z[i] - zhat[i];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_M; j++) kf.x[i] += K[i][j] * y[j];
    }

    // P = (I - K H) P
    float IKH[KF_N][KF_N];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_M; k++) s += K[i][k] * H[k][j];
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - s;
        }
    }
    float P_new[KF_N][KF_N];
    for (int i = 0; i < KF_N; i++) {
        for (int j = 0; j < KF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < KF_N; k++) s += IKH[i][k] * kf.P[k][j];
            P_new[i][j] = s;
        }
    }
    for (int i = 0; i < KF_N; i++)
        for (int j = 0; j < KF_N; j++)
            kf.P[i][j] = P_new[i][j];
}
