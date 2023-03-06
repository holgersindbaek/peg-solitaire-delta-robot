#ifndef TRAJECTORY_FUNCTIONS_H
#define TRAJECTORY_FUNCTIONS_H
#include <Arduino.h>

  extern const float max_deg_vel;
  extern const float max_deg_acc;
  extern const float max_deg_dec;

  struct TrajValues {
    float start_deg;
    float end_deg;
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
    float deg[3];
    float vel[3];
    float acc[3];
  };

  void get_traj(float start_degs[3], float end_degs[3], float max_deg_traj_vel[3], float max_deg_traj_acc[3], TrajValues traj_values[3]);
  void get_trapezoidal_traj(float start_deg, float end_deg, float start_vel, float max_vel, float max_acc, float max_dec, float &acc_time_step_, float &vel_time_step_, float &complete_time_step_, float &max_acc_signed_, float &max_vel_signed_, float &max_dec_signed_, float &y_acc_);
  void get_traj_step(float start_deg, float end_deg, float time_step, float acc_time_step, float vel_time_step, float complete_time_step, float vel, float max_acc_signed, float max_dec_signed, float max_vel_signed, float y_acc, TrajStep &traj_step, int coordinate_index);

#endif