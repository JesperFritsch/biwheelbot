# Pitch Kalman Filter

Linear Kalman filter estimating the tip-over (pitch) angle, fusing the LSM9DS1
gyro and accelerometer. Runs at a fixed 200 Hz (`KF_DT` = 5 ms). Lives in
`src/kalman.cpp` / `include/kalman.h`; sensor conversion in `sensor_get_pitch()`
(`src/sensor.cpp`); wired up in `main.cpp`.

## Model

**State** `x = [θ, θ̇]` — tilt angle (deg) and angular rate (deg/s).

**Process** (constant-velocity):

```
θ  ← θ + Δt·θ̇
θ̇  ← θ̇
```

The "θ̇ stays constant" assumption is knowingly wrong (motors/gravity torque the
body); the error is absorbed by a large process noise on θ̇ (see Tuning).

**Measurement** `z = [θ_accel, θ̇_gyro]`, so `H = I`:

- `θ_accel` = `atan2(acc.y, acc.z)` in degrees — computed in the sensor driver,
  *not* in the filter. Keeping the nonlinear accel→angle conversion at the
  sensor boundary keeps `H` linear/trivial with uniform sensitivity over ±180°.
- `θ̇_gyro` = `-(gyr.x − gyro_cal.x)` — bias-calibrated at boot, sign flipped to
  match d(θ_accel)/dt.

**Unit rule:** everything inside the filter is degrees / deg/s. All conversion
happens in `sensor_get_pitch()`. Don't mix units past that boundary.

## Code structure

The four model-specific pieces are isolated functions (`predict_state`,
`get_F`, `predict_measurement`, `get_H`) even though they're trivial here —
they're the only things that change for an EKF/UKF upgrade. The predict/update
code is dimension-generic loops over `KF_N`/`KF_M`; only the 2×2 inverse of S
is hard-coded (guarded by a `static_assert`).

The five equations map directly:

```
kalman_predict:  x = f(x)                        // predict_state
                 P = F P Fᵀ + Q
kalman_update:   S = H P Hᵀ + R
                 K = P Hᵀ S⁻¹
                 x = x + K (z − h(x))            // h = predict_measurement
                 P = (I − K H) P
```

## Tuning

All values are hand-tuned on the robot; R baselines should eventually come from
a stationary bench log (variance of a few thousand samples per channel).

| Constant | Value | Meaning |
|---|---|---|
| `R_DIAG[0]` | 1.0 deg² | accel-angle noise while quiet |
| `R_DIAG[1]` | 0.16 (deg/s)² | gyro rate noise |
| `Q_DIAG[0]` | 0.005 deg² | per-step angle model slack. **Main responsiveness knob**: sets how fast θ pulls toward the accel angle (settle speed after motion). 0.001 was visibly laggier than the comp filter; 0.005 matches it. Confirmed numerically: at rest 0.005 gives K[0][0] = 0.068 against the comp filter's 0.030, but at rest is the wrong operating point — while balancing `accel_dev` is continuously nonzero and the schedule already brings K to ≈0.035 at dev = 0.03 and ≈0.020 at dev = 0.06. Q[0] = 0.001 would put those at 0.016 and 0.009. |
| `Q_DIAG[1]` | — | has almost no effect on the *angle*: the gyro measurement pins P[1][1] directly and the leakage into P[0][0] is dt²·Q[1] = 0.000625, negligible beside Q[0]. K[0][0] is unchanged (0.0683) for Q[1] anywhere from 0.5 to 25. Tune the angle's responsiveness with Q[0], not Q[1]. |
| `Q_DIAG[1]` | 25 (deg/s)² | per-step rate slack (√25 = 5 deg/s per 5 ms). Deliberately large so θ̇ tracks the gyro at ≈0.99 gain despite the constant-velocity lie. |

Q is a diagonal simplification (the exact constant-velocity Q has dt-coupled
off-diagonals); the difference is absorbed into tuning. Note P's off-diagonal
is *not* simplified — the θ↔θ̇ cross-covariance arises from `F P Fᵀ` and is
what lets a rate measurement correct the angle.

## Addition: scheduled R (accel gating)

**Problem:** the accelerometer measures gravity + linear acceleration and can't
distinguish them, so translating the robot fakes a tilt. Any filter with fixed
accel trust (comp filter, plain KF) leans with it.

**Mechanism:** R is passed per-update (`kalman_update(kf, z, R)`), built by:

```
R[0] = R_DIAG[0] + 3300 · dev²        // kalman_measurement_R()
if |y[0]| > 3°:  R[0] *= |y[0]| / 3°  // innovation gate, same function
```

`dev` (g) is the current non-gravity acceleration, detected from the accel
itself: gravity alone has constant magnitude, so `|‖a‖ − rest|` measures the
corruption. Big `dev` → huge `R[0]` → Kalman gain on the accel collapses → the
filter coasts on the gyro until the motion stops. No modes or thresholds in the
filter; the gain equation itself does the gating.

Two details that matter (both learned the hard way — the first cut was sluggish):

- **Reference and noise floor are measured, not assumed.** During
  `sensor_calibrate_imu()` (bot still), the driver records mean and σ of `‖a‖`.
  `dev = max(0, |‖a‖ − mean| − 3σ)`. The mean absorbs the accel's static offset
  (comparing against ideal 1.0 g kept the gate permanently half-closed); the 3σ
  floor makes sensor noise and hand tremor cost exactly nothing, so at rest the
  filter is identical to the ungated baseline.
- **The gain 3300 is derived, not guessed.** `dev` g of lateral accel fakes at
  most `atan(dev) ≈ 57.3·dev` deg of tilt; treating that as the 1σ error gives
  `(57.3·dev)² ≈ 3300·dev²`. Distrust matches the worst-case lie — an earlier
  10000 over-penalized small deviations 3× and felt mushy.

**Accepted tradeoff:** while `dev > 0` the filter is slightly less accel-responsive
than the comp filter — which is "faster" there only because it trusts corrupted
data (its speed and its false tilt are the same behavior). Blind spots: gentle
accelerations that barely move `‖a‖` slip through; long gated stretches drift on
open-loop gyro until the next quiet moment.

## Tried and reverted (don't redo without reading this)

- **EKF with `h = sin θ`** (raw normalized accel-y as the measurement):
  measurably worse. `sin θ` has zero slope at ±90° (accel stops informing the
  angle exactly where swing tests go) and folds past 90° so overshoot corrects
  the wrong way; near upright it's identical to the linear filter anyway
  (sin θ ≈ θ). Keep the atan2 conversion in the driver; EKF machinery only pays
  off for measurements that can't be pre-inverted.
- **Gyro-bias third state** `[θ, θ̇, b]` (gyro measures θ̇+b, H row [0,1,1]):
  conceptually the win over the comp filter (tracks bias drift online instead
  of a one-shot boot calibration), but it was added bundled with the EKF and an
  untuned gate, and fast hand-turns leaked ±5–6 deg/s of accel corruption into
  b̂, making recovery very slow. Worth re-adding *alone* on top of the current
  state, now that scheduled R blocks the leak path — validate the warm-gyro
  drift scenario on hardware before keeping it.

## Possible next steps

- Re-add the bias state (above).
- Encoder-based accel compensation: wheel acceleration → expected drive-axis
  linear acceleration → subtract from the accel reading before atan2. Handles
  sustained gentle acceleration the magnitude gate can't see; stacks at the
  sensor boundary without filter changes.
- Motor command as control input `G·u` in predict — only worth it with a decent
  dynamics model (sloppy G makes the filter worse); the motor FF sweep data may
  feed this.
- Replace guessed `R_DIAG` baselines with bench-measured variances.
