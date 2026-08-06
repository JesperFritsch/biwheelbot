#include "control.h"

#include <mbed.h>
#include "safety.h"
#include "sensor.h"
#include "motor.h"
#include "kalman.h"

using namespace std::chrono_literals;

// Static stack keeps the allocation out of the heap and makes the footprint
// visible at link time.
static unsigned char control_stack[4096] __attribute__((aligned(8)));
static rtos::Thread control_thread(osPriorityHigh, sizeof(control_stack),
                                   control_stack, "control");

static rtos::Mutex snap_mutex;
static ControlSnapshot snap;
static volatile uint32_t overruns = 0;
static KalmanFilter kf;
static PitchMeasurement m{};
static MotorState motor_state = MotorState::OFF;
static float v_batt = 0.0f;
static uint32_t cycles_batt = 0;

static void control_loop() {
    auto next = rtos::Kernel::Clock::now();
    while (true) {
        
        kalman_predict(kf);
        if (!sensor_get_pitch(m)) {
            float z[KF_M] = { m.angle, m.rate };
            float R[KF_M];
            kalman_measurement_R(m.accel_dev, R);
            kalman_update(kf, z, R);
        }
        
        cycles_batt++;
        if (cycles_batt >= 20) {  // every 100 ms
            v_batt = sensor_read_battery();
            cycles_batt = 0;
        }

        auto new_motor_state = safety_update(kf.x[0], kf.x[1], v_batt);
        if (new_motor_state != motor_state) {
            motor_state = new_motor_state;
            if (motor_state == MotorState::OFF) {
                set_motors_enabled(false);
            }
            else {
                set_motors_enabled(true);
            }
        }

        
        
        // predict -> measure/update -> safety gate -> cascade -> motor_set_*
        
        {
            rtos::ScopedMutexLock lock(snap_mutex);
            snap = {
                .angle = kf.x[0],
                .rate = kf.x[1],
                .battery_voltage = v_batt,
                .motor_a_duty = 0.0f, // TODO: fill in with actual motor duty
                .motor_b_duty = 0.0f, // TODO: fill in with actual motor duty
                .motors_enabled = (motor_state == MotorState::ON),
                .overruns = overruns
            };
        }
        
        next += 5ms;
        auto now = rtos::Kernel::Clock::now();
        if (now > next) {
            overruns++;
            next = now;   // resync; don't chase missed ticks back-to-back
        }
        rtos::ThisThread::sleep_until(next);
    }
}

void control_start() {

    PitchMeasurement m0;
    while (sensor_get_pitch(m0)) {}
    float z0[KF_M] = { m0.angle, m0.rate };
    kalman_init(kf, z0);

    control_thread.start(control_loop);   // call at the end of setup()
}

ControlSnapshot control_get_snapshot() {
    rtos::ScopedMutexLock lock(snap_mutex);
    return snap;
}