#ifndef KINEMATIC_FUNCTIONS_H
#define KINEMATIC_FUNCTIONS_H
#include <Arduino.h>
#include <ODriveArduino.h>
#include <math.h>
#include "trajectory_functions.h"

  extern ODriveArduino odrive_array[];
  extern double zero_offset_array[];
  extern double current_position[];
  extern const double l_s;
  extern const double r_o;
  extern const double r_u;
  extern const double l_m;
  extern const int gear_ratio;
  extern const float horizontal_offset_angle;
  extern const float x_max;
  extern const float x_min;
  extern const float y_max;
  extern const float y_min;
  extern const float z_max;
  extern const float z_min;
  extern const float time_step_delta;
  extern const float max_vel;

  void move_to_position(double x, double y, double z);
  void set_position(double x, double y, double z, double vel_x = max_vel, double vel_y = max_vel, double vel_z = max_vel);
  void set_angle(int odrive_number, double theta_deg, double vel = max_vel);
  void calculate_motor_angles(double x, double y, double z, double &theta_deg_1, double &theta_deg_2, double &theta_deg_3);
  void calculate_motor_angle(double x0, double y0, double z0, double &theta_deg);
  void calculate_motor_position(float theta_rounds[3], float coordinates[3]);

#endif