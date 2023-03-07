#ifndef KINEMATIC_FUNCTIONS_H
#define KINEMATIC_FUNCTIONS_H
#include <Arduino.h>
#include <ODriveArduino.h>
#include <math.h>
#include "trajectory_functions.h"

  extern ODriveArduino odrive_array[];
  extern HardwareSerial *odrive_serial_array[];
  extern float zero_offset_array[];
  extern float last_thetas[];
  extern float current_pos[];
  extern const float l_s;
  extern const float r_o;
  extern const float r_u;
  extern const float l_m;
  extern const int gear_ratio;
  extern const float horizontal_offset_angle;
  extern const float x_max;
  extern const float x_min;
  extern const float y_max;
  extern const float y_min;
  extern const float z_max;
  extern const float z_min;
  extern const float max_angle_deg;
  extern const float min_angle_deg;
  extern const float time_step_delta;
  extern const float max_pos_vel;
  extern const float max_pos_acc;
  extern const float max_pos_dec;
  extern const float max_theta_vel;
  extern const float max_theta_acc;

  void move_to_position(float x, float y, float z, float speed_throttle = 1.0);
  void set_position(float x, float y, float z, float vel_x = max_theta_vel, float vel_y = max_theta_vel, float vel_z = max_theta_vel);
  void set_angle(int odrive_number, float theta_deg, float vel = max_theta_vel);
  void calculate_motor_angles(float x, float y, float z, float theta_degs[3]);
  void calculate_motor_angle(float x0, float y0, float z0, float &theta_deg);
  void calculate_motor_position(float theta_rounds[3], float coordinates[3]);

#endif