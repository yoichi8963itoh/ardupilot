#include "Copter.h"

/*
 * Init and run calls for deadman flight mode
 */

#define DEADMAN_TIMER_MS 30000
#define INPUT_SAMPLING_MS 200
#define NEUTRAL_THRESHOLD 100.0
#define CHANGED_THRESHOLD 100.0
#define ROLL_OFFSET 0.0
#define PITCH_OFFSET 0.0
#define YAW_OFFSET 0.0
#define THROTTLE_OFFSET 484.0

// deadman_init - initialise deadman controller
bool Copter::ModeDeadman::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !_copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    // initialize deadman
    is_pilot_active = true;
    reset_timer(deadman_timer_start_ms);
    reset_timer(input_sampling_timer_start_ms);

    previous_input_roll = 0.0f;
    previous_input_pitch = 0.0f;
    previous_input_yaw = 0.0f;
    previous_input_throttle = 0.0f;

    return true;
}

// deadman_run - runs the main deadman controller
// should be called at 100hz or more
void Copter::ModeDeadman::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = _copter.aparm;

    // get pilot input
    float input_roll = channel_roll->get_control_in();
    float input_pitch = channel_pitch->get_control_in();
    float input_yaw = channel_yaw->get_control_in();
    float input_throttle = channel_throttle->get_control_in();

    // deadman is activated by timer, and copter hovers at constant position.
    // timer is reset when pilot input changed or copter resumed normal flight.
    // copter resumes normal flight when roll, pitch yaw and throttle are all neutral.
    if (is_pilot_active) {
        if (is_changed_pilot_input(input_roll, input_pitch, input_yaw, input_throttle)) {
            reset_timer(deadman_timer_start_ms);
        } else {
            if (is_timeout(deadman_timer_start_ms, DEADMAN_TIMER_MS) && !is_neutral(input_roll, input_pitch, input_yaw, input_throttle)) {
                is_pilot_active = false;
                gcs().send_text(MAV_SEVERITY_INFO, "Deadman is active");
            }
        }
    } else {
        if (is_neutral(input_roll, input_pitch, input_yaw, input_throttle)) {
            reset_timer(deadman_timer_start_ms);
            is_pilot_active = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Normal flight");
        } else {
            // override pilot input
            input_roll = ROLL_OFFSET;
            input_pitch = PITCH_OFFSET;
            input_yaw = YAW_OFFSET;
            input_throttle = THROTTLE_OFFSET;
        }
    }

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(input_roll, input_pitch, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(input_yaw);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(input_throttle);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

// reset_deadman_timer - preserve system timer for calculate elapsed time in function is_timeout
void Copter::ModeDeadman::reset_timer(uint32_t& timer_ms)
{
    timer_ms = AP_HAL::millis();
}

// is_timeout - return if the elapsed time exceeds limit_duration_ms
bool Copter::ModeDeadman::is_timeout(uint32_t start_ms, uint32_t limit_duration_ms)
{
    uint32_t diff_ms;
    uint32_t current_ms = AP_HAL::millis();

    if (start_ms <= current_ms) {
        diff_ms = current_ms - start_ms;
    } else {
        diff_ms = (UINT32_MAX - start_ms) + 1 + current_ms;
    }

    return limit_duration_ms <= diff_ms;
}

// is_changed_pilot_input - return if pilot input is different from previous sample
bool Copter::ModeDeadman::is_changed_pilot_input(float current_roll, float current_pitch, float current_yaw, float current_throttle)
{
    // checking pilot input is executed every INPUT_SAMPLING_MS milliseconds
    if (!is_timeout(input_sampling_timer_start_ms, INPUT_SAMPLING_MS)) {
        return false;
    }

    // get pilot input difference between current sample and previous sample
    float diff_roll = fabsf(current_roll - previous_input_roll);
    float diff_pitch = fabsf(current_pitch - previous_input_pitch);
    float diff_yaw = fabsf(current_yaw - previous_input_yaw);
    float diff_throttle = fabsf(current_throttle - previous_input_throttle);

    // preserve current pilot input
    previous_input_roll = current_roll;
    previous_input_pitch = current_pitch;
    previous_input_yaw = current_yaw;
    previous_input_throttle = current_throttle;

    return (CHANGED_THRESHOLD < diff_roll) || (CHANGED_THRESHOLD < diff_pitch) || (CHANGED_THRESHOLD < diff_yaw) || (CHANGED_THRESHOLD < diff_throttle);
}

// is_neutral - return if roll, pitch, yaw and throttle are all neutral
bool Copter::ModeDeadman::is_neutral(float roll, float pitch, float yaw, float throttle)
{
    return (fabsf(roll) < NEUTRAL_THRESHOLD) && (fabsf(pitch) < NEUTRAL_THRESHOLD) && (fabsf(yaw) < NEUTRAL_THRESHOLD) && (fabsf(throttle - THROTTLE_OFFSET) < NEUTRAL_THRESHOLD);
}
