#pragma once

// Classic complementary filter: trust the gyro over the short term, let the
// accelerometer pull the estimate back over the long term.
//
//     angle = a * (angle + rate * dt) + (1 - a) * accel_angle
//
// `a` on its own says nothing -- it only means something together with the tick
// rate. What actually characterises the filter is the crossover time constant:
//
//     tau = a * dt / (1 - a)
//
// Above 1/tau the estimate is integrated gyro; below it, the accelerometer wins.
//
// At the 5 ms tick, a = 0.97 gives tau = 162 ms. The old project ran a = 0.97 at
// a 10 ms tick, so its tau was 323 ms -- twice as long. To reproduce that feel
// here you want a = 0.985, not 0.97.
//
// Unlike the old implementation this is unit-consistent: `accel_angle` and
// `rate` are both in degrees / deg/s, so the blend of the two is meaningful.
// The old filter mixed an accel term scaled by ~1.57x with a gyro term at 1x,
// which is why its steady-state and transient gains disagreed.
class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = 0.97f)
        : alpha(alpha), angle(0.0f), primed(false) {}

    float update(float accel_angle, float rate, float dt) {
        // Start from the accelerometer rather than converging to it over the
        // first tau, which would otherwise look like a slow drift at boot.
        if (!primed) {
            angle = accel_angle;
            primed = true;
            return angle;
        }
        angle = alpha * (angle + rate * dt) + (1.0f - alpha) * accel_angle;
        return angle;
    }

    float value() const { return angle; }

    // Crossover time constant in seconds, for the tick rate it is being run at.
    float tau(float dt) const { return alpha * dt / (1.0f - alpha); }

    void reset(float accel_angle) {
        angle = accel_angle;
        primed = true;
    }

private:
    float alpha;
    float angle;
    bool primed;
};
