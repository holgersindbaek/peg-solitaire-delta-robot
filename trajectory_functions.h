#ifndef TRAJECTORY_FUNCTIONS_H
#define TRAJECTORY_FUNCTIONS_H
#include <Arduino.h>

  extern const float max_traj_vel;
  extern const float max_traj_acc;
  extern const float max_traj_dec;

  struct TrajValues {
    float start_point;
    float end_point;
    float start_vel;
    float max_vel;
    float max_acc;
    float max_dec;
    float acc_time_step;
    float vel_time_step;
    float complete_time_step;
    float max_acc_signed;
    float max_vel_signed;
    float max_dec_signed;
    float y_acc;
  };

  struct TrajStep {
    float pos[3];
    float vel[3];
    float acc[3];
  };

  void get_traj(float start_points[3], float end_points[3], TrajValues traj_values[3], float distance_ratios[3]);
  void get_trapezoidal_traj(float start_point, float end_point, float start_vel, float max_vel, float max_acc, float max_dec, float &acc_time_step_, float &vel_time_step_, float &complete_time_step_, float &max_acc_signed_, float &max_vel_signed_, float &max_dec_signed_, float &y_acc_);
  void get_traj_step(float start_point, float end_point, float time_step, float acc_time_step, float vel_time_step, float complete_time_step, float vel, float max_acc_signed, float max_dec_signed, float max_vel_signed, float y_acc, TrajStep &traj_step, int coordinate_index);

#endif