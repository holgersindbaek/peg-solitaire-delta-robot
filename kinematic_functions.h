#ifndef KINEMATIC_FUNCTIONS_H
#define KINEMATIC_FUNCTIONS_H
#include <Arduino.h>
#include <ODriveArduino.h>
#include <math.h>
#include "trajectory_functions.h"

extern ODriveArduino odrive_array[];
extern double zero_offset_array[];
extern const double l_s;
extern const double r_o;
extern const double r_u;
extern const double l_m;
extern const int gear_ratio;

void move_to_position(double x, double y, double z);
void calculate_motor_angles(double x, double y, double z, double &theta_1, double &theta_2, double &theta_3);
void calculate_motor_angle(double x0, double y0, double z0, double &theta);
void set_angle(int odrive_number, double theta);

#endif