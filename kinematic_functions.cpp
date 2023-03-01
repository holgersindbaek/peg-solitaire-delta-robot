#include "kinematic_functions.h"

// Move the robot to a specified end effector position
void move_to_position(double x, double y, double z) {
  long max_complete_time_step = 0;
  float time_step_deltas[3] = {time_step_delta, time_step_delta, time_step_delta};

  float start_thetas[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};
  float start_points[3];
  calculate_motor_position(start_thetas, start_points);

  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    double pos = start_points[coordinate_index];

    Serial.println("pos: " + String(pos));
  }

  double end_points[3] = {x, y, z};
  TrajValues traj_values[3];
  get_traj(start_points, end_points, traj_values);

  // Get the highest step values to make those driving the motors move the same amount of steps
  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    TrajValues traj_value = traj_values[coordinate_index];

    if (traj_value.complete_time_step > max_complete_time_step) {
      max_complete_time_step = traj_value.complete_time_step;
    }
  }

  // Change the time step deltas to make the motors move the same amount of steps
  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    TrajValues traj_value = traj_values[coordinate_index];

    Serial.println("traj_value.complete_time_step: " + String(traj_value.complete_time_step));
    time_step_deltas[coordinate_index] = traj_value.complete_time_step / max_complete_time_step * time_step_delta;
    Serial.println("traj_value.complete_time_step / max_complete_time_step * time_step_delta: " + String(traj_value.complete_time_step / max_complete_time_step * time_step_delta));
  }

  Serial.println("max_complete_time_step: " + String(max_complete_time_step));

  long max_steps = ceil(max_complete_time_step / time_step_delta);
  Serial.println("max_steps: " + String(max_steps));
  for (int step_index = 0; step_index <= max_steps; step_index++) {
    TrajStep traj_step;

    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float time_step = time_step_deltas[coordinate_index] * step_index;
      TrajValues traj_value = traj_values[coordinate_index];

      get_traj_step(traj_value.start_point, traj_value.end_point, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
    }
    // Serial.println("pos: " + String(traj_step.pos[0]) + " - " + String(traj_step.pos[1]) + " - " + String(traj_step.pos[2]) + " || vel: " + String(traj_step.vel[0]) + " - " + String(traj_step.vel[1]) + " - " + String(traj_step.vel[2]) + " || acc: " + String(traj_step.acc[0]) + " - " + String(traj_step.acc[1]) + " - " + String(traj_step.acc[2]));

    // Calculate angles
    double theta_deg_1, theta_deg_2, theta_deg_3;
    calculate_motor_angles(traj_step.pos[0], traj_step.pos[1], traj_step.pos[2], theta_deg_1, theta_deg_2, theta_deg_3);

    // Move the motors to the calculated angles
    set_angle(1, theta_deg_1, abs(traj_step.vel[0]));
    set_angle(2, theta_deg_2, abs(traj_step.vel[1]));
    set_angle(3, theta_deg_3, abs(traj_step.vel[2]));

    delay(time_step_delta * 1000);
  }
}

// Calculate the motor angles for a given end effector position
void calculate_motor_angles(double x, double y, double z, double &theta_deg_1, double &theta_deg_2, double &theta_deg_3) {
  calculate_motor_angle(x, y, z, theta_deg_1);
  calculate_motor_angle(x * cos(120 * PI / 180) + y * sin(120 * PI / 180), y * cos(120 * PI / 180) - x * sin(120 * PI / 180), z, theta_deg_2); // Rotate coordinates to +120 deg
  calculate_motor_angle(x * cos(120 * PI / 180) - y * sin(120 * PI / 180), y * cos(120 * PI / 180) + x * sin(120 * PI / 180), z, theta_deg_3); // Rotate coordinates to -120 deg
}

void calculate_motor_angle(double x0, double y0, double z0, double &theta_deg) {
  // Find y-coordinates of point S and M_m
  double S_y = -l_s * tan(30 * PI/180) / 2.0;
  double M_m_y = y0 - l_m * tan(30 * PI/180) / 2.0;

  // z = a + b*y
  double a = (x0 * x0 + M_m_y * M_m_y + z0 * z0 + r_o * r_o - r_u * r_u - S_y * S_y) / (2.0 * z0);
  double b = (S_y - M_m_y) / z0;

  // Discriminant
  double d = -(a + b * S_y) * (a + b * S_y) + r_o * (b * b * r_o + r_o);

  // Non-existing point
  if (d < 0) {
    Serial.println("Non-existing point");
    theta_deg = NULL;
    return;
  }

  // Calculate two sides of triangle made from the point A and S
  double A_y = (S_y - a * b - sqrt(d)) / (b * b + 1);
  double A_z = a + b * A_y;

  // Calculate theta
  theta_deg = atan(-A_z / (S_y - A_y)) * 180.0 / PI + ((A_y > S_y) ? 180.0 : 0.0);
}

// Calculate the position of the end-effector
void calculate_motor_position(float theta_rounds[3], float coordinates[3]) {

  // Calculate the real theta degrees
  double real_theta_degs[3];
  for (int odrive_index = 0; odrive_index < 3; odrive_index++) {
    double zero_offset_deg = zero_offset_array[odrive_index];
    double horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    double theta_rad = theta_rounds[odrive_index] * (2 * PI) / gear_ratio;
    double theta_deg = theta_rad * 180 / PI;
    real_theta_degs[odrive_index] = abs(horizontal_offset_deg) + theta_deg;
  }

  double theta_1_rad = real_theta_degs[0] * PI / 180;
  double theta_2_rad = real_theta_degs[1] * PI / 180;
  double theta_3_rad = real_theta_degs[2] * PI / 180;

  double t = ((l_s - l_m) * tan(30.0 * PI / 180)) / 2.0;

  double y_1 = -(t + r_o * cos(theta_1_rad));
  double z_1 = -r_o * sin(theta_1_rad);

  double y_2 = (t + r_o * cos(theta_2_rad)) * sin(30.0 * PI / 180);
  double x_2 = y_2 * tan(60.0 * PI / 180);
  double z_2 = -r_o * sin(theta_2_rad);

  double y_3 = (t + r_o * cos(theta_3_rad)) * sin(30.0 * PI / 180);
  double x_3 = -y_3 * tan(60.0 * PI / 180);
  double z_3 = -r_o * sin(theta_3_rad);

  double dnm = (y_2 - y_1) * x_3 - (y_3 - y_1) * x_2;

  double w_1 = y_1 * y_1 + z_1 * z_1;
  double w_2 = x_2 * x_2 + y_2 * y_2 + z_2 * z_2;
  double w_3 = x_3 * x_3 + y_3 * y_3 + z_3 * z_3;

  // x = (a_1*z + b_1)/dnm
  double a_1 = (z_2 - z_1) * (y_3 - y_1) - (z_3 - z_1) * (y_2 - y_1);
  double b_1 = -((w_2 - w_1) * (y_3 - y_1) - (w_3 - w_1) * (y_2 - y_1)) / 2.0;

  // y = (a_2*z + b_2)/dnm;
  double a_2 = -(z_2 - z_1) * x_3 + (z_3 - z_1) * x_2;
  double b_2 = ((w_2 - w_1) * x_3 - (w_3 - w_1) * x_2) / 2.0;

  // a*z^2 + b*z + c = 0
  double a = a_1 * a_1 + a_2 * a_2 + dnm * dnm;
  double b = 2.0 * (a_1 * b_1 + a_2 * (b_2 - y_1 * dnm) - z_1 * dnm * dnm);
  double c = (b_2 - y_1 * dnm) * (b_2 - y_1 * dnm) + b_1 * b_1 + dnm * dnm * (z_1 * z_1 - r_u * r_u);

  // Discriminant
  double d = b * b - 4.0 * a * c;
  if (d < 0) {
    Serial.println("Non-existing point");
    return;
  }

  float z = -0.5 * (b + sqrt(d)) / a;
  float x = (a_1 * z + b_1) / dnm;
  float y = (a_2 * z + b_2) / dnm;

  coordinates[0] = x;
  coordinates[1] = y;
  coordinates[2] = z;
}

void set_angle(int odrive_number, double theta_deg, double vel = 1.0) {
  // Serial.println("== " + String(odrive_number) + " ==");
  // Serial.println("theta_deg: " + String(theta_deg));

  if (theta_deg == NULL) {
    Serial.println("Position is out of bounds.");
    return;
  }

  // Check to see if angle will collide with spacer
  double min_angle = 0.0;
  double max_angle = 115.0;
  if (min_angle > theta_deg || min_angle > theta_deg) {
    Serial.println("Position more than max or less than min.");
    Serial.println(theta_deg);
    return;
  }

  // Calculate the actual theta given the zero offset and gear ratio
  double zero_offset_deg = zero_offset_array[odrive_number - 1];
  double horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
  double real_theta_deg = horizontal_offset_deg + theta_deg;
  double real_theta_rad = real_theta_deg * PI / 180;
  double real_theta_rounds = real_theta_rad / (2 * PI) * gear_ratio;

  // Serial.println("theta: " + String(theta_deg));
  // Serial.println("zero_offset: " + String(zero_offset_deg));
  // Serial.println("horizontal_offset: " + String(horizontal_offset_deg));
  // Serial.println("real_theta_deg: " + String(real_theta_deg));
  // Serial.println("real_theta_rad: " + String(real_theta_rad));
  // Serial.println("real_theta_rounds: " + String(real_theta_rounds));
  // Serial.println("real_theta_rounds: " + String(real_theta_rounds * gear_ratio));

  // Set the motor angle
  // odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds);
  // odrive_array[odrive_number - 1].TrapezoidalMove(0, real_theta_rounds);
  odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, vel);
}