#include "control.h"

#include <mbed.h>
#include <atomic>
#include "hal/us_ticker_api.h"
#include "platform/mbed_critical.h"
#include "safety.h"
#include "types.h"
#include "sensor.h"
#include "motor.h"
#include "kalman.h"
#include "complementary.h"
#include "motor_ff.h"

using namespace std::chrono_literals;


static_assert(sizeof(DriveCmd) == 4, "drive command must fit one word");
// std::atomic<T>::is_always_lock_free would say this more directly, but it is
// C++17 and the mbed core builds at gnu++14. The builtin is equivalent and
// standard-agnostic; the 0 means "assume natural alignment".
static_assert(__atomic_always_lock_free(sizeof(DriveCmd), 0),
              "drive slot must stay lock-free: past one word std::atomic falls "
              "back to a library lock, which would put a hidden mutex on this "
              "thread's hot path");

static const float pos_speed_lp_steps = 10;
static const float speed_angle_lp_steps = 20;

// Static stack keeps the allocation out of the heap and makes the footprint
// visible at link time.
static unsigned char control_stack[4096] __attribute__((aligned(8)));
static rtos::Thread control_thread(osPriorityHigh, sizeof(control_stack),
                                   control_stack, "control");

static rtos::Mutex snap_mutex;
static ControlSnapshot snap;
static volatile uint32_t overruns = 0, imu_misses = 0, consecutive_imu_misses = 0;
static KalmanFilter kf;
static PitchMeasurement m{};
static MotorState motor_state = MotorState::OFF;
static float v_batt = 0.0f;
static uint32_t cycles_batt = 0;
static WheelPosition t_pos{};
static WheelPosition last_position{};
static float t_wheel_diff_mm = 0.0f;
static uint32_t timestamp;

static ComplementaryFilter comp(0.97f);   // tau = 162 ms at the 5 ms tick
static volatile bool use_complementary = false;

static const float MAX_SPEED = 1200.0f;
static const float MAX_ANGLE = 45.f;
// static const float MAX_TURN = 

// Gains live in two copies. `gains_shared` is the retune target, written by
// whichever thread runs the BLE stack; `gains_live` is private to the control
// thread and is what the PIDControllers hold references to. The loop refreshes
// live from shared at a tick boundary, so a retune can never land between the
// kp and kd reads of a single update() -- which at 200 Hz on a balancing robot
// shows up as a kick that looks like an estimator fault.
//
// `gains_dirty` is the handshake: set inside the same critical section as the
// write, so the loop's steady-state cost is one load and a branch.
static volatile PIDGains gains_shared[GAIN_COUNT] = {
    [GAIN_BALANCE] = { .kp = 0.085744f, .ki = 0.0f,      .kd =  0.000912f },
    [GAIN_SPEED]   = { .kp = 0.013509f, .ki = 0.000188f, .kd = -0.001219f },
    [GAIN_POS]     = { .kp = 1.351516f, .ki = 0.000777f, .kd =  0.623395f }, // ki 20 after windup implemented
    [GAIN_TURN]    = { .kp = 0.1f,      .ki = 0.0f,      .kd =  0.0001f   },
};
static volatile bool gains_dirty = true;   // forces the first refresh to seed gains_live
static PIDGains gains_live[GAIN_COUNT];

static std::atomic<DriveCmd> drive_slot{DriveCmd{}};


// Pull any pending retune into the control thread's copy. Called once per tick,
// before the PIDs run, and once from control_start() to seed the defaults.
static void gains_refresh() {
    if (!gains_dirty) return;
    core_util_critical_section_enter();
    for (int i = 0; i < GAIN_COUNT; i++) {
        gains_live[i].kp = gains_shared[i].kp;
        gains_live[i].ki = gains_shared[i].ki;
        gains_live[i].kd = gains_shared[i].kd;
    }
    gains_dirty = false;
    core_util_critical_section_exit();
}

void reset_position() {
    sensor_reset_position();
    t_pos = sensor_get_wheels().position;
    t_wheel_diff_mm = t_pos.pos_diff_mm();
}

// returns the adjustign duty for turning (a, b)
std::pair<float, float> assign_turn_duty(float base_duty, float turn_duty) {
    float real_t_d = turn_duty * (1 - base_duty);
    return std::pair<float, float>(base_duty + real_t_d, base_duty - real_t_d);
}

static void control_loop() {
    auto next = rtos::Kernel::Clock::now();
    // controllers for balancing
    auto pos_pid = PIDController(gains_live[GAIN_POS], MAX_SPEED);
    auto speed_pid = PIDController(gains_live[GAIN_SPEED], MAX_ANGLE);
    auto duty_pid = PIDController(gains_live[GAIN_BALANCE], 1.0f);

    // controllers for turning
    auto pos_turn_pid = PIDController(gains_live[GAIN_TURN], 1.0f);

    auto pos_speed_lp = LowPassFilter(pos_speed_lp_steps);
    auto speed_angle_lp = LowPassFilter(speed_angle_lp_steps);

    uint8_t last_cmd_seq = 0;
    uint32_t drive_stale = 0;

    while (true) {
        gains_refresh();

        DriveCmd cmd = drive_slot.load(std::memory_order_relaxed);
        if (cmd.seq != last_cmd_seq) {
            last_cmd_seq = cmd.seq;
            drive_stale = 0;
        } else if (++drive_stale > SAFETY_DRIVE_STALE_TICKS) {
            cmd = DriveCmd{};
        }

        float cmd_drive = cmd.linear / 127.0f;
        float cmd_turn = cmd.angular / 127.0f;

        auto current_time = us_ticker_read();
        auto passed_time = (current_time - timestamp);
        auto passed_time_s = passed_time / 1000000.f;
        timestamp = current_time;
        kalman_predict(kf);
        if (!sensor_poll_imu()) {
            consecutive_imu_misses = 0;
            sensor_get_pitch(m);
            float z[KF_M] = { m.angle, m.rate };
            float R[KF_M];
            kalman_measurement_R(m.accel_dev, m.angle - kf.x[0], R);
            kalman_update(kf, z, R);
            comp.update(m.angle, m.rate, passed_time_s);
        } else {
            imu_misses++;
            consecutive_imu_misses++;
        }

        float est_angle = use_complementary ? comp.value() : kf.x[0];
        float est_rate  = use_complementary ? m.rate       : kf.x[1];
        float est_yaw_rate = sensor_get_yaw_rate(est_angle);

        cycles_batt++;
        if (cycles_batt >= 20) {
            v_batt = sensor_read_battery();
            cycles_batt = 0;
        }

        auto new_motor_state = safety_update(est_angle, est_rate, v_batt, consecutive_imu_misses);
        auto w_state = sensor_get_wheels();
        auto curr_pos = w_state.position;
        auto w_speed = w_state.speed.avg_speed();

        if (new_motor_state == MotorState::OFF) {
            set_motors_enabled(false);
            reset_position();
        }
        else if (new_motor_state == MotorState::ON && motor_state == MotorState::OFF) {
            reset_position();
            set_motors_enabled(true);
        }

        motor_state = new_motor_state;

        float pos_mm = curr_pos.avg_pos_mm();
        float t_mm = t_pos.avg_pos_mm();
        // TODO implement commanded speed
        // balancing pids
        auto target_speed = pos_speed_lp.update(pos_pid.update(pos_mm, t_mm, passed_time_s));
        if (cmd_drive != 0) {
            target_speed = cmd_drive * MAX_SPEED;
        }
        auto target_angle = speed_angle_lp.update(speed_pid.update(w_speed, target_speed, passed_time_s));
        auto effort_duty = -duty_pid.update(est_angle, target_angle, passed_time_s);

        // turning pids
        auto a_b_diff_d = last_position.pos_diff_mm() - curr_pos.pos_diff_mm();
        auto turn_duty = pos_turn_pid.update(curr_pos.pos_diff_mm(), t_pos.pos_diff_mm(), passed_time_s);
        auto assigned_duties = assign_turn_duty(effort_duty, turn_duty);
        auto e_duty_a = assigned_duties.first;
        auto e_duty_b = assigned_duties.second;
        auto true_duty_a = ff_duty(e_duty_a);
        auto true_duty_b = ff_duty(e_duty_b);
        last_position = curr_pos;

        motor_set_a(true_duty_a);
        motor_set_b(true_duty_b);


        {
            rtos::ScopedMutexLock lock(snap_mutex);
            snap = {
                .angle = est_angle,
                .rate = est_rate,
                .kf_angle = kf.x[0],
                .comp_angle = comp.value(),
                .battery_voltage = v_batt,
                .turning_duty = turn_duty, // TODO: implement turning control
                .effort_duty = effort_duty,
                .target_angle = target_angle,
                .target_speed = target_speed,
                .ab_diff_drift = a_b_diff_d,
                .motor_a_duty = true_duty_a,
                .motor_b_duty = true_duty_b,
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
            next = now; 
        }
        rtos::ThisThread::sleep_until(next);
    }
}

void control_start() {
    // Seed gains_live before the PIDControllers bind references to it.
    gains_refresh();

    PitchMeasurement m0;
    while (sensor_get_pitch(m0)) {}
    float z0[KF_M] = { m0.angle, m0.rate };
    kalman_init(kf, z0);

    control_thread.start(control_loop);
    timestamp = us_ticker_read();
}

ControlSnapshot control_get_snapshot() {
    rtos::ScopedMutexLock lock(snap_mutex);
    return snap;
}

// Field-wise rather than a struct copy: assigning a whole volatile struct is
// ill-formed, the implicit copy-assignment operator does not accept a volatile
// operand.
void control_set_gains(GainId id, PIDGains gains) {
    if (id >= GAIN_COUNT) return;
    core_util_critical_section_enter();
    gains_shared[id].kp = gains.kp;
    gains_shared[id].ki = gains.ki;
    gains_shared[id].kd = gains.kd;
    gains_dirty = true;
    core_util_critical_section_exit();
}

PIDGains control_get_gains(GainId id) {
    if (id >= GAIN_COUNT) return { 0.0f, 0.0f, 0.0f };
    core_util_critical_section_enter();
    PIDGains gains = {
        gains_shared[id].kp,
        gains_shared[id].ki,
        gains_shared[id].kd
    };
    core_util_critical_section_exit();
    return gains;
}

void control_set_drive(DriveCmd cmd) {
    drive_slot.store(cmd, std::memory_order_relaxed);
}