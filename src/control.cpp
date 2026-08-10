#include "control.h"

#include <mbed.h>
#include "safety.h"
#include "types.h"
#include "sensor.h"
#include "motor.h"
#include "kalman.h"
#include "motor_ff.h"

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
static WheelPosition t_pos{};

static const float MAX_SPEED = 1200.0f;
static const float MAX_ANGLE = 45.f;

static volatile PIDGains pos_to_speed = { 
    kp: 0.001f,
    ki: 0.0f, // 20 after windup implemented
    kd: 0.0002f
};

static volatile PIDGains speed_to_angle = { 
    kp: 0.0160f,
    ki: 0.000f,
    kd: -0.000141f
};

static volatile PIDGains angle_to_duty = { 
    kp: 0.084147f,
    ki: 0.0f,
    kd: 0.00096f
};
 
static void control_loop() {
    auto next = rtos::Kernel::Clock::now();
    auto pos_pid = PIDController(pos_to_speed, MAX_SPEED);
    auto speed_pid = PIDController(speed_to_angle, MAX_ANGLE);
    auto duty_pid = PIDController(angle_to_duty);

    LowPassFilter speed_lp(10);

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
        // always disable motors if safety gate says so, but only enable if it was previously off
        auto w_state = sensor_get_wheels();
        auto w_speed = w_state.speed.avg_speed();

        if (new_motor_state == MotorState::OFF) {
            set_motors_enabled(false);
        }
        else if (new_motor_state == MotorState::ON && motor_state == MotorState::OFF) {
            set_motors_enabled(true);
            t_pos = w_state.position; 
        }

        motor_state = new_motor_state;

        float pos_mm = w_state.position.avg_pos() * MM_PER_COUNT;
        float t_mm = t_pos.avg_pos() * MM_PER_COUNT;
        auto target_speed = pos_pid.update(pos_mm, t_mm, KF_DT);
        auto target_angle = speed_lp.update(speed_pid.update(w_speed, target_speed, KF_DT));
        auto effort_duty = -duty_pid.update(kf.x[0], target_angle, KF_DT);
        auto true_duty = ff_duty(effort_duty);
        
        motor_set_a(true_duty);
        motor_set_b(true_duty);

        {
            rtos::ScopedMutexLock lock(snap_mutex);
            snap = {
                .angle = kf.x[0],
                .rate = kf.x[1],
                .battery_voltage = v_batt,
                .turning_duty = 0.0f, // TODO: implement turning control
                .effort_duty = effort_duty,
                .target_angle = target_angle,
                .target_speed = target_speed,
                .motor_a_duty = true_duty,
                .motor_b_duty = true_duty,
                .motors_enabled = (motor_state == MotorState::ON),
                .t_pos_mm = t_mm,
                .pos_mm = pos_mm,
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

void set_balance_gains(PIDGains gains) {
    angle_to_duty.kp = gains.kp;
    angle_to_duty.ki = gains.ki;
    angle_to_duty.kd = gains.kd;
};

void set_speed_gains(PIDGains gains) {
    speed_to_angle.kp = gains.kp;
    speed_to_angle.ki = gains.ki;
    speed_to_angle.kd = gains.kd;
};

void set_pos_gains(PIDGains gains) {
    pos_to_speed.kp = gains.kp;
    pos_to_speed.ki = gains.ki;
    pos_to_speed.kd = gains.kd;
};

PIDGains get_balance_gains() {
    return {
        angle_to_duty.kp, 
        angle_to_duty.ki, 
        angle_to_duty.kd
    };
};

PIDGains get_speed_gains() {
    return {
        speed_to_angle.kp, 
        speed_to_angle.ki, 
        speed_to_angle.kd
    };
};

PIDGains get_pos_gains() {
    return {
        pos_to_speed.kp, 
        pos_to_speed.ki, 
        pos_to_speed.kd
    };
};