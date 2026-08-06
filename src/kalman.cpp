#include "kalman.h"

// Measurement noise R (diagonal), same units as the state: deg^2 and (deg/s)^2.
// Starting guesses — replace with variances computed from a stationary bench
// log (a few thousand samples per channel).
static const float R_DIAG[KF_M] = { 1.0f, 0.16f };

// Process noise Q (diagonal), hand-tuned on the robot. Acts as a floor on P:
// too low -> filter goes overconfident and reacts sluggishly to shoves,
// too high -> raw sensor noise passes through to the control loop.
// Q[1] is large because the constant-velocity model is a poor fit while the
// motors are actively torquing the body around.
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

void kalman_update(KalmanFilter &kf, const float z[KF_M]) {
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
        S[i][i] += R_DIAG[i];
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
