#include "trajectory_functions.h"

void get_traj(float start_degs[3], float end_degs[3], float max_deg_traj_vel, float max_deg_traj_acc, float distance_ratios[3], TrajValues traj_values[3]) {
  for (int i = 0; i < 3; i++) {
    TrajValues &traj_value = traj_values[i];
    traj_value.start_deg = start_degs[i];
    traj_value.end_deg = end_degs[i];
    traj_value.start_vel = 0.0; // Starting velocity
    traj_value.max_vel = max_deg_traj_vel * distance_ratios[i];
    traj_value.max_acc = max_deg_traj_acc * distance_ratios[i];
    traj_value.max_dec = max_deg_traj_acc * distance_ratios[i];
    traj_value.acc_time_step = 0.0;
    traj_value.vel_time_step = 0.0;
    traj_value.complete_time_step = 0.0;
    traj_value.max_acc_signed = 0.0;
    traj_value.max_vel_signed = 0.0;
    traj_value.max_dec_signed = 0.0;
    traj_value.y_acc = 0.0;

    get_trapezoidal_traj(traj_value.start_deg, traj_value.end_deg, traj_value.start_vel, traj_value.max_vel, traj_value.max_acc, traj_value.max_dec, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.max_acc_signed, traj_value.max_vel_signed, traj_value.max_dec_signed, traj_value.y_acc);
  }
}

// Symbol                                             Description
// acc_time_step, vel_time_step and dec_time_step     Duration of the stages of the AL profile
// start_deg and vel                                  Adapted initial conditions for the AL profile
// end_deg                                            Position set-theta
// sign                                               Direction (sign) of the trajectory
// max_vel, max_acc, max_dec                          Kinematic bounds
// max_acc_signed, max_dec_signed and max_vel_signed  Reached values of acceleration and velocity
void get_trapezoidal_traj(float start_deg, float end_deg, float start_vel, float max_vel, float max_acc, float max_dec, float &acc_time_step_, float &vel_time_step_, float &complete_time_step_, float &max_acc_signed_, float &max_vel_signed_, float &max_dec_signed_, float &y_acc_) {
  float delta_dis = end_deg - start_deg;                    // Distance to travel
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
  float y_acc = start_deg + start_vel * acc_time_step + 0.5f * max_acc_signed * pow(acc_time_step, 2); // pos at end of accel phase

  acc_time_step_ = acc_time_step;
  vel_time_step_ = vel_time_step;
  complete_time_step_ = complete_time_step;
  max_acc_signed_ = max_acc_signed;
  max_vel_signed_ = max_vel_signed;
  max_dec_signed_ = max_dec_signed;
  y_acc_ = y_acc;

  // Serial.println("--");
  // Serial.println("delta_dis: " + String(delta_dis));
  // Serial.println("stop_dist: " + String(stop_dist));
  // Serial.println("delta_dis_stop: " + String(delta_dis_stop));
  // Serial.println("stop_dist: " + String(stop_dist));
  // Serial.println("acc_time_step_: " + String(acc_time_step_));
  // Serial.println("vel_time_step_: " + String(vel_time_step_));
  // Serial.println("complete_time_step_: " + String(complete_time_step_));
  // Serial.println("max_vel_signed_: " + String(max_vel_signed_));
}

void get_traj_step(float start_deg, float end_deg, float time_step, float acc_time_step, float vel_time_step, float complete_time_step, float vel, float max_acc_signed, float max_dec_signed, float max_vel_signed, float y_acc, TrajStep &traj_step, int coordinate_index) {
  // Serial.print("STEP: " + String(time_step) + " - " + String(acc_time_step) + " - " + String(vel_time_step) + " - " + String(complete_time_step));
  if (time_step < 0.0) { // Initial Condition
    // Serial.println(" - Initial Condition");
    traj_step.deg[coordinate_index] = start_deg;
    traj_step.vel[coordinate_index] = vel;
    traj_step.acc[coordinate_index] = 0.0;

  } else if (time_step < acc_time_step) { // Accelerating
    // Serial.println(" - Accelerating");
    traj_step.deg[coordinate_index] = start_deg + vel * time_step + 0.5 * max_acc_signed * pow(time_step, 2);
    traj_step.vel[coordinate_index] = vel + max_acc_signed * time_step;
    traj_step.acc[coordinate_index] = max_acc_signed;

  } else if (time_step < acc_time_step + vel_time_step) { // Coasting
    // Serial.println(" - Coasting");
    traj_step.deg[coordinate_index] = y_acc + max_vel_signed * (time_step - acc_time_step);
    traj_step.vel[coordinate_index] = max_vel_signed;
    traj_step.acc[coordinate_index] = 0.0;
  
  } else if (time_step < complete_time_step) { // Deceleration
    // Serial.println(" - Deceleration");

    float td = time_step - complete_time_step;
    traj_step.deg[coordinate_index] = end_deg + 0.5 * max_dec_signed * pow(td, 2);
    traj_step.vel[coordinate_index] = max_dec_signed * td;
    traj_step.acc[coordinate_index] = max_dec_signed;
  
  } else if (time_step >= complete_time_step) { // Final Condition
    // Serial.println(" - Final Condition");
    traj_step.deg[coordinate_index] = end_deg;
    traj_step.vel[coordinate_index] = 0.0;
    traj_step.acc[coordinate_index] = 0.0;
  }
}