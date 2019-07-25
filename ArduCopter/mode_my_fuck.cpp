#include "Copter.h"

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ModeFuck::init(bool ignore_checks)
{

    // initialise position and desired velocity
    if (!pos_control->is_active_z())
    {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    target_climb_rate = 0.0f;
    target_roll = 0.0f;
    target_pitch = 0.0f;
    target_yaw_rate = 0.0f;
    myfuck_state = Fuck_MotorStopped;
    return true;
}

#ifdef BS_Add
void Copter::ModeFuck::my_fuck_init_vel()
{
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);
}
void Copter::ModeFuck::my_fuck_change_state(FuckModeState state_change_to)
{
    myfuck_state = state_change_to;
}
void Copter::ModeFuck::my_fuck_set_target_para(float target_climb_rate_in)
{
    target_climb_rate = target_climb_rate_in;
}

void Copter::ModeFuck::my_fuck_MotorStopped()
{
    motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    attitude_control->reset_rate_controller_I_terms();
    attitude_control->set_yaw_target_to_current_heading();
    // forces throttle output to go to zero
    pos_control->relax_alt_hold_controllers(0.0f);
    pos_control->update_z_controller();
}

void Copter::ModeFuck::my_fuck_takeoff_run()
{
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initiate take-off
    if (!takeoff.running())
    {
        takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
        // indicate we are taking off
        set_land_complete(false);
        // clear i terms
        set_throttle_takeoff();
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
    pos_control->update_z_controller();
}

void Copter::ModeFuck::my_fuck_flying_run()
{
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}
void Copter::ModeFuck::my_fuck_land_run()
{
    // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
    if (target_climb_rate < 0.0f)
    {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    }
    else
    {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    }

    attitude_control->reset_rate_controller_I_terms();
    attitude_control->set_yaw_target_to_current_heading();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
    pos_control->update_z_controller();
}

void Copter::ModeFuck::run()
{
    takeoff_climb_rate = 0.0f;
    my_fuck_init_vel();

    // call the correct auto controller
    switch (myfuck_state)
    {

    case Fuck_MotorStopped:
        my_fuck_MotorStopped();
        break;
    case Fuck_Takeoff:
        // run takeoff controller
        my_fuck_takeoff_run();
        break;
    case Fuck_Flying:
        my_fuck_flying_run();
        break;
    case Fuck_Landed:
        my_fuck_land_run();
        break;
    }
}
#endif

#ifndef BS_Origin
// // althold_run - runs the althold controller
// // should be called at 100hz or more
// void Copter::ModeFuck::run()
// {
//     AltHoldModeState althold_state;
//     float takeoff_climb_rate = 0.0f;

//     // initialize vertical speeds and acceleration
//     pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
//     pos_control->set_accel_z(g.pilot_accel_z);

//     // apply SIMPLE mode transform to pilot inputs
//     // 存到channel_roll的control_in中
//     update_simple_mode();

//     // get pilot desired lean angles
//     // 从channel_roll的control_in存到这里定义的变量中
//     float target_roll, target_pitch;
//     get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

//     // get pilot's desired yaw rate
//     float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

//     // get pilot desired climb rate
//     float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
//     target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

//     // target_roll,target_pitch,target_yaw_rate,target_climb_rate都准备好了

// #ifdef BS_DEBUG_INFO
//     gcs().send_text(MAV_SEVERITY_DEBUG,
//                     "[%.3f]target_climb_rate:%.3f",
//                     (double)(AP_HAL::millis() * 0.001f), target_climb_rate);
// #endif

//     // Alt Hold State Machine Determination
//     if (!motors->armed() || !motors->get_interlock())
//     {
//         althold_state = AltHold_MotorStopped;
//     }
//     //不知道是那个触发了起飞
//     // else if (takeoff.running() || takeoff.triggered(target_climb_rate))
//     else if (takeoff.triggered(target_climb_rate))
//     {

//         althold_state = AltHold_Takeoff;
//     }
//     else if (!ap.auto_armed || ap.land_complete)
//     {
//         althold_state = AltHold_Landed;
//     }
//     else
//     {

//         althold_state = AltHold_Flying;
//     }

//     // Alt Hold State Machine
//     switch (althold_state)
//     {

//     case AltHold_MotorStopped:
// #ifdef BS_DEBUG
//         gcs().send_text(MAV_SEVERITY_DEBUG, "[%.3f]AltHold_MotorStopped",
//                         (double)(AP_HAL::millis() * 0.001f));
// #endif
//         motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
//         attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->set_yaw_target_to_current_heading();
//         // forces throttle output to go to zero
//         pos_control->relax_alt_hold_controllers(0.0f);
//         pos_control->update_z_controller();
//         break;

//         /*------------------------------------------*/
//     case AltHold_Takeoff:
//         // set motors to full range
//         motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

// #ifdef BS_DEBUG
//         gcs().send_text(MAV_SEVERITY_DEBUG, "[%.3f]AltHold_Takeoff",
//                         (double)(AP_HAL::millis() * 0.001f));
// #endif

//         // initiate take-off
//         if (!takeoff.running())
//         {
//             takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
//             // indicate we are taking off
//             set_land_complete(false);
//             // clear i terms
//             set_throttle_takeoff();
//         }

//         // get take-off adjusted pilot and takeoff climb rates
// takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

//         // get avoidance adjusted climb rate
//         target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

//         // call attitude controller
//         attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

//         // call position controller
//         pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
//         pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
//         pos_control->update_z_controller();
//         break;

//         /*------------------------------------------*/
//     case AltHold_Landed:
// #ifdef BS_DEBUG
//         gcs().send_text(MAV_SEVERITY_DEBUG, "[%.3f]AltHold_Landed",
//                         (double)(AP_HAL::millis() * 0.001f));
// #endif
//         // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
//         if (target_climb_rate < 0.0f)
//         {
//             motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
//         }
//         else
//         {
//             motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
//         }

//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->set_yaw_target_to_current_heading();

//         attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
//         pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
//         pos_control->update_z_controller();
//         break;
//         /*------------------------------------------*/
//     case AltHold_Flying:

//         motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

// #if AC_AVOID_ENABLED == ENABLED
//         // apply avoidance
//         copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
// #endif

// #ifdef BS_DEBUG
//         gcs().send_text(MAV_SEVERITY_DEBUG, "[%.3f]attitude_controller",
//                         (double)(AP_HAL::millis() * 0.001f));
// #endif
//         // call attitude controller
//         attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

//         // adjust climb rate using rangefinder
//         target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

//         // get avoidance adjusted climb rate
//         target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

//         // call position controller
//         pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
//         pos_control->update_z_controller();
//         break;
//     }
// }
#endif