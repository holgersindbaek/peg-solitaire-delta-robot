#include "trajectory_functions.h"

void get_traj(float start_pos[3], float end_pos[3], float max_pos_traj_vel[3], float max_pos_traj_acc[3], TrajValues traj_values[3]) {
  for (int i = 0; i < 3; i++) {
    TrajValues &traj_value = traj_values[i];
    traj_value.start_pos = start_pos[i];
    traj_value.end_pos = end_pos[i];
    traj_value.start_vel = 0.0; // Starting velocity
    traj_value.max_vel = max_pos_traj_vel[i];
    traj_value.max_acc = max_pos_traj_acc[i];
    traj_value.max_dec = max_pos_traj_acc[i];
    traj_value.acc_time_step = 0.0;
    traj_value.vel_time_step = 0.0;
    traj_value.complete_time_step = 0.0;
    traj_value.max_acc_signed = 0.0;
    traj_value.max_vel_signed = 0.0;
    traj_value.max_dec_signed = 0.0;
    traj_value.y_acc = 0.0;

    get_trapezoidal_traj(traj_value.start_pos, traj_value.end_pos, traj_value.start_vel, traj_value.max_vel, traj_value.max_acc, traj_value.max_dec, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.max_acc_signed, traj_value.max_vel_signed, traj_value.max_dec_signed, traj_value.y_acc);
  }
}

// Symbol                                             Description
// acc_time_step, vel_time_step and dec_time_step     Duration of the stages of the AL profile
// start_pos and vel                                  Adapted initial conditions for the AL profile
// end_pos                                            Position set-theta
// sign                                               Direction (sign) of the trajectory
// max_vel, max_acc, max_dec                          Kinematic bounds
// max_acc_signed, max_dec_signed and max_vel_signed  Reached values of acceleration and velocity
void get_trapezoidal_traj(float start_pos, float end_pos, float start_vel, float max_vel, float max_acc, float max_dec, float &acc_time_step_, float &vel_time_step_, float &complete_time_step_, float &max_acc_signed_, float &max_vel_signed_, float &max_dec_signed_, float &y_acc_) {
  float delta_dis = end_pos - start_pos;                        // Distance to travel
  float stop_dist = (start_vel * start_vel) / (2.0f * max_dec); // Minimum stopping distance
  float delta_dis_stop = copysign(stop_dist, start_vel);    // Minimum stopping displacement
  float sign = ((delta_dis - delta_dis_stop < 0) ? -1 : 1); // Sign of coast velocity (if any)
  float max_acc_signed = sign * max_acc; // Maximum Acceleration (signed)
  float max_dec_signed = -sign * max_dec; // Maximum Deceleration (signed)
  float max_vel_signed = sign * max_vel; // Maximum Velocity (signed)

  float acc_time_step;
  float vel_time_step;
  float dec_time_step;

  // If we start with a speed faster than cruising, then we need to decel instead of accel
  // aka "float deceleration move" in the paper
  if ((sign * start_vel) > (sign * max_vel_signed)) {
    max_acc_signed = -sign * max_acc;
  }

  // Time to accel/decel to/from max_vel_signed (cruise speed)
  acc_time_step = (max_vel_signed - start_vel) / max_acc_signed;
  dec_time_step = -max_vel_signed / max_dec_signed;

  // Integral of velocity ramps over the full accel and decel times to get
  // minimum displacement required to reach cuising speed
  float delta_dis_min = 0.5f * acc_time_step * (max_vel_signed + start_vel) + 0.5f * dec_time_step * max_vel_signed;

  // Are we displacing enough to reach cruising speed?
  if (sign * delta_dis < sign * delta_dis_min) {
    // Short move (triangle profile)
    max_vel_signed = sign * sqrt(max((max_dec_signed * max_vel_signed * max_vel_signed + 2 * max_acc_signed * max_dec_signed * delta_dis) / (max_dec_signed - max_acc_signed), 0.0f));
    acc_time_step = max(0.0f, (max_vel_signed - start_vel) / max_acc_signed);
    dec_time_step = max(0.0f, -max_vel_signed / max_dec_signed);
    vel_time_step = 0.0f;
  } else {
    // Long move (trapezoidal profile)
    vel_time_step = (delta_dis - delta_dis_min) / max_vel_signed;
  }

  // Fill in the rest of the values used at evaluation-time
  float complete_time_step = acc_time_step + vel_time_step + dec_time_step;
  float y_acc = start_pos + start_vel * acc_time_step + 0.5f * max_acc_signed * pow(acc_time_step, 2); // pos at end of accel phase

  acc_time_step_ = acc_time_step;
  vel_time_step_ = vel_time_step;
  complete_time_step_ = complete_time_step;
  max_acc_signed_ = max_acc_signed;
  max_vel_signed_ = max_vel_signed;
  max_dec_signed_ = max_dec_signed;
  y_acc_ = y_acc;
}

void get_traj_step(float start_pos, float end_pos, float time_step, float acc_time_step, float vel_time_step, float complete_time_step, float vel, float max_acc_signed, float max_dec_signed, float max_vel_signed, float y_acc, TrajStep &traj_step, int coordinate_index) {
  if (time_step <= 0.0) { // Initial Condition
    traj_step.pos[coordinate_index] = start_pos;
    traj_step.vel[coordinate_index] = vel;
    traj_step.acc[coordinate_index] = 0.0;

  } else if (time_step < acc_time_step) { // Accelerating
    traj_step.pos[coordinate_index] = start_pos + vel * time_step + 0.5 * max_acc_signed * pow(time_step, 2);
    traj_step.vel[coordinate_index] = vel + max_acc_signed * time_step;
    traj_step.acc[coordinate_index] = max_acc_signed;

  } else if (time_step < acc_time_step + vel_time_step) { // Coasting
    traj_step.pos[coordinate_index] = y_acc + max_vel_signed * (time_step - acc_time_step);
    traj_step.vel[coordinate_index] = max_vel_signed;
    traj_step.acc[coordinate_index] = 0.0;
  
  } else if (time_step < complete_time_step) { // Deceleration
    float td = time_step - complete_time_step;
    traj_step.pos[coordinate_index] = end_pos + 0.5 * max_dec_signed * pow(td, 2);
    traj_step.vel[coordinate_index] = max_dec_signed * td;
    traj_step.acc[coordinate_index] = max_dec_signed;
  
  } else if (time_step >= complete_time_step) { // Final Condition
    traj_step.pos[coordinate_index] = end_pos;
    traj_step.vel[coordinate_index] = 0.0;
    traj_step.acc[coordinate_index] = 0.0;
  }
}