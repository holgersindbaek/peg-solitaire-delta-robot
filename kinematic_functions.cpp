#include "kinematic_functions.h"

float time_step_delta = 1.0; // Decide which timesteps to divide the trajectory into TODO: Change to something smaller

// Move the robot to a specified end effector position
void move_to_position(double x, double y, double z) {
  long max_steps = 0;
  long acc_time_step = 0;
  long vel_time_step = 0;
  long complete_time_step = 0;
  TrajValues traj_values[3];
  get_traj(x, y, z, traj_values, max_steps, time_step_delta);
  
  // Get the highest step values to make those driving the motors move the same amount of steps
  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    TrajValues traj_value = traj_values[coordinate_index];

    if (traj_value.complete_time_step > max_steps) {
      max_steps = traj_value.complete_time_step;
    }
  }

  for (int step_index = 0; step_index <= max_steps; step_index++) {
    TrajStep traj_step;

    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float time_step = time_step_delta * step_index;
      TrajValues traj_value = traj_values[coordinate_index];

      get_traj_step(traj_value.start_point, traj_value.end_point, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
    }
    Serial.println("---");
    Serial.println("pos: " + String(traj_step.pos[0]) + " - " + String(traj_step.pos[1]) + " - " + String(traj_step.pos[2]) + " || vel: " + String(traj_step.vel[0]) + " || acc: " + String(traj_step.acc[0]));

    // // Calculate angles
    // double theta_1, theta_2, theta_3;
    // calculate_motor_angles(traj_step.pos[0], traj_step.pos[1], traj_step.pos[2], theta_1, theta_2, theta_3);

    // // Move the motors to the calculated angles
    // set_angle(1, theta_1);
    // set_angle(2, theta_2);
    // set_angle(3, theta_3);

    // delay(time_step_delta);
  }
}

// Calculate the motor angles for a given end effector position
void calculate_motor_angles(double x, double y, double z, double& theta_1, double& theta_2, double& theta_3) {
  calculate_motor_angle(x, y, z, theta_1);
  calculate_motor_angle(x * cos(120 * PI/180) + y * sin(120 * PI/180), y * cos(120 * PI/180) - x * sin(120 * PI/180), z, theta_2); // Rotate coordinates to +120 deg
  calculate_motor_angle(x * cos(120 * PI/180) - y * sin(120 * PI/180), y * cos(120 * PI/180) + x * sin(120 * PI/180), z, theta_3); // Rotate coordinates to -120 deg
}

// Calculate angle theta for YZ plane
void calculate_motor_angle(double x0, double y0, double z0, double &theta) {
  // Find y-coordinates of point S and M_m
  double S_y = -l_s * tan(30 * PI/180) / 2.0;
  double M_m_y = y0 - l_m * tan(30 * PI/180) / 2.0;

  // z = a + b*y
  double a = (x0 * x0 + M_m_y * M_m_y + z0 * z0 + r_o * r_o - r_u * r_u - S_y * S_y) / (2.0 * z0);
  double b = (S_y - M_m_y) / z0;

  // Discriminant
  double d = -(a + b * S_y) * (a + b * S_y) + r_o * (b * b * r_o + r_o);

  // Non-existing point, return error
  if (d < 0) {
    theta = NULL;
    return;
  }

  // Calculate two sides of triangle made from the point A and S
  double A_y = (S_y - a * b - sqrt(d)) / (b * b + 1);
  double A_z = a + b * A_y;

  // Calculate theta
  theta = atan(-A_z / (S_y - A_y)) * 180.0 / M_PI + ((A_y > S_y) ? 180.0 : 0.0);
}

void set_angle(int odrive_number, double theta) {
  Serial.println("== " + String(odrive_number) + " ==");

  if (theta == NULL) {
    Serial.println("Position is out of bounds.");
    return;
  }

  // Check to see if angle will collide with spacer
  double min_angle = 0.0;
  double max_angle = 115.0;
  if (min_angle > theta || min_angle > theta) {
    Serial.println("Position more than max or less than min.");
    Serial.println(theta);
    return;
  }

  // Calculate the actual theta given the zero offset and gear ratio
  double zero_offset = zero_offset_array[odrive_number - 1];
  double horizontal_offset = -63.2 + zero_offset;
  double real_theta = horizontal_offset + theta;
  double real_theta_rad = real_theta * PI / 180;
  double real_theta_rounds = real_theta_rad / (2 * PI) * gear_ratio;

  Serial.println("theta: " + String(theta));
  Serial.println("zero_offset: " + String(zero_offset));
  Serial.println("horizontal_offset: " + String(horizontal_offset));
  Serial.println("real_theta: " + String(real_theta));
  Serial.println("real_theta_rad: " + String(real_theta_rad));
  Serial.println("real_theta_rounds: " + String(real_theta_rounds));
  Serial.println("real_theta_rounds: " + String(real_theta_rounds * gear_ratio));

  // Set the motor angle
  // odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds);
  // odrive_array[odrive_number - 1].TrapezoidalMove(0, real_theta_rounds);
  odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, 0.1);
}