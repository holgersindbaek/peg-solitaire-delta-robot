#include "kinematic_functions.h"

// Move the robot to a specified end effector position
void move_to_position(float x, float y, float z) {
  long max_complete_time_step = 0;
  float end_points[3] = {x, y, z};
  float time_step_deltas[3] = {time_step_delta, time_step_delta, time_step_delta};
  float start_thetas[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};
  float start_points[3];
  calculate_motor_position(start_thetas, start_points);

  // Find the longest distance and create ratio based on that
  float distances[3] = {abs(end_points[0] - start_points[0]), abs(end_points[1] - start_points[1]), abs(end_points[2] - start_points[2])};
  float max_distance = 0.0;
  int max_distance_index;
  for (int index = 0; index < 3; index++) {
    if (distances[index] > max_distance) {
      max_distance = distances[index];
      max_distance_index = index;
    }
  }
  float distance_ratios[3] = {distances[0] / max_distance, distances[1] / max_distance, distances[2] / max_distance};
  Serial.println("distances: " + String(distances[0]) + ", " + String(distances[1]) + ", " + String(distances[2]));

  // Get the trapezoidal trajectory values for each coordinate
  TrajValues traj_values[3];
  get_traj(start_points, end_points, traj_values, distance_ratios);

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

    time_step_deltas[coordinate_index] = traj_value.complete_time_step / max_complete_time_step * time_step_delta;
  }

  // Check to see if trajectory is unstable
  long max_steps = ceil(max_complete_time_step / time_step_delta);
  float complete_move_time = max_complete_time_step * time_step_delta;
  float delay_time = complete_move_time / max_steps;
  float last_coordinates[3] = {start_points[0], start_points[1], start_points[2]};
  float step_distances[3] = {0.0, 0.0, 0.0};
  float durations[3] = {0.0, 0.0, 0.0};
  float positive_directions[3] = {start_points[0] < end_points[0], start_points[1] < end_points[1], start_points[2] < end_points[2]};
  float last_thetas[3] = {start_thetas[0], start_thetas[1], start_thetas[2]};
  float theta_durations[3] = {0.0, 0.0, 0.0};
  float theta_round_distances[3] = {0.0, 0.0, 0.0};
  float last_vel[3] = {0.0, 0.0, 0.0};
  for (int step_index = 0; step_index <= max_steps; step_index++) {
    TrajStep traj_step;
    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float time_step = time_step_deltas[coordinate_index] * step_index;
      TrajValues traj_value = traj_values[coordinate_index];

      get_traj_step(traj_value.start_point, traj_value.end_point, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.start_vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
    }

    // // Print out trajectory step values
    // Serial.println("step_index: " + String(String(step_index) + "      ").substring(0, 3) + " || " + "pos: " + String(traj_step.pos[0]) + " - " + String(traj_step.pos[1]) + " - " + String(traj_step.pos[2]) + " || vel: " + String(traj_step.vel[0]) + " - " + String(traj_step.vel[1]) + " - " + String(traj_step.vel[2]) + " || acc: " + String(traj_step.acc[0]) + " - " + String(traj_step.acc[1]) + " - " + String(traj_step.acc[2]));

    if (
      (positive_directions[0] && (traj_step.pos[0] < last_coordinates[0] || traj_step.pos[0] > end_points[0])) ||
      (!positive_directions[0] && (traj_step.pos[0] > last_coordinates[0] || traj_step.pos[0] < end_points[0])) ||
      (positive_directions[1] && (traj_step.pos[1] < last_coordinates[1] || traj_step.pos[1] > end_points[1])) ||
      (!positive_directions[1] && (traj_step.pos[1] > last_coordinates[1] || traj_step.pos[1] < end_points[1])) ||
      (positive_directions[2] && (traj_step.pos[2] < last_coordinates[2] || traj_step.pos[2] > end_points[2])) ||
      (!positive_directions[2] && (traj_step.pos[2] > last_coordinates[2] || traj_step.pos[2] < end_points[2]))
    ) {
      Serial.println("Couldn't move along path. Trajectory is unstable. We suggest you change velocity and acceleration values.");
      return;
    }

    last_coordinates[0] = traj_step.pos[0];
    last_coordinates[1] = traj_step.pos[1];
    last_coordinates[2] = traj_step.pos[2];
  }

  // // Move the motors along trajectory until the end position is reached
  // Serial.println("max_steps: " + String(max_steps));
  // float measured_max_vel = 0.0;
  // float max_step_distance = distances[max_distance_index] / max_steps;
  // long current_step = 0;
  // do {
  //   float current_points[3];
  //   float current_theta[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};
  //   calculate_motor_position(current_theta, current_points);

  //   long next_step;
  //   TrajStep traj_step;
  //   for (int step_index = current_step; step_index <= max_steps; step_index++) {
  //     // Get step for max distance
  //     for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
  //       float time_step = time_step_deltas[coordinate_index] * (step_index + 1);
  //       TrajValues traj_value = traj_values[coordinate_index];

  //       get_traj_step(traj_value.start_point, traj_value.end_point, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.start_vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
  //     }

  //     // Exit loop when next step has been found
  //     float difference = fabs(traj_step.pos[max_distance_index] - current_points[max_distance_index]);
  //     if (difference > (max_step_distance / 2)) {

  //       next_step = step_index + 1;
  //       break;
  //     }
  //   }

  //   // Continue loop if still on the same step
  //   if (current_step == next_step) {
  //     continue;
  //   }

  //   // Break the loop if outside of bounds
  //   if (traj_step.pos[0] > x_max || traj_step.pos[0] < x_min || traj_step.pos[1] > y_max || traj_step.pos[1] < y_min || traj_step.pos[2] > z_max || traj_step.pos[2] < z_min) {
  //     Serial.println("Outside of bounds. Movement stopped.");
  //     break;
  //   }

  //   float measured_vel = abs(odrive_array[max_distance_index].GetVelocity(0));
  //   if (measured_max_vel < measured_vel) {
  //     measured_max_vel = measured_vel;
  //   }

  //   // Calculate theta velocities from step velocities
  //   float theta_vel_1 = (traj_step.vel[0] / max_traj_vel) * max_traj_vel;
  //   float theta_vel_2 = (traj_step.vel[1] / max_traj_vel) * max_traj_vel;
  //   float theta_vel_3 = (traj_step.vel[2] / max_traj_vel) * max_traj_vel;

  //   // Move the motors to the calculated angles
  //   set_position(traj_step.pos[0], traj_step.pos[1], traj_step.pos[2], theta_vel_1, theta_vel_2, theta_vel_3);

  //   // Update current step
  //   current_step = next_step;
  // } while (max_steps > current_step);

  // Serial.println("measured_max_vel: " + String(measured_max_vel));
}

void set_position(float x, float y, float z, float vel_1, float vel_2, float vel_3) {
  // Serial.println("set_position x: " + String(x) + " || y: " + String(y) + " || z: " + String(z) + " || vel_x: " + String(vel_x) + " || vel_y: " + String(vel_y) + " || vel_z: " + String(vel_z));

  // Calculate angles
  float theta_deg_1, theta_deg_2, theta_deg_3;
  calculate_motor_angles(x, y, z, theta_deg_1, theta_deg_2, theta_deg_3);

  // Move the motors to the calculated angles
  set_angle(1, theta_deg_1, vel_1);
  set_angle(2, theta_deg_2, vel_2);
  set_angle(3, theta_deg_3, vel_3);

  // Update current position
  current_position[0] = x;
  current_position[1] = y;
  current_position[2] = z;
}

void set_angle(int odrive_number, float theta_deg, float vel) {
  // Serial.println("set_angle odrive_number: " + String(odrive_number) + " || theta_deg: " + String(theta_deg) + " || vel: " + String(vel));

  if (theta_deg == NULL) {
    Serial.println("Position is out of bounds.");
    return;
  }

  // Check to see if angle will collide with spacer
  if (min_angle > theta_deg || min_angle > theta_deg) {
    Serial.println("Position more than max or less than min.");
    Serial.println(theta_deg);
    return;
  }

  // Calculate the actual theta given the zero offset and gear ratio
  float zero_offset_deg = zero_offset_array[odrive_number - 1];
  float horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
  float real_theta_deg = horizontal_offset_deg + theta_deg;
  float real_theta_rad = real_theta_deg * PI / 180;
  float real_theta_rounds = real_theta_rad / (2 * PI) * gear_ratio;

  // TODO: Find out if we can se velocity using `q` (https://docs.odriverobotics.com/v/latest/ascii-protocol.html#motor-position) or figure out what feed-forward is
  // Set the motor angle
  // odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds);
  odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, vel, 0.1);
}

// Calculate the motor angles for a given end effector position
void calculate_motor_angles(float x, float y, float z, float &theta_deg_1, float &theta_deg_2, float &theta_deg_3) {
  calculate_motor_angle(x, y, z, theta_deg_1);
  calculate_motor_angle(x * cos(120 * PI / 180) + y * sin(120 * PI / 180), y * cos(120 * PI / 180) - x * sin(120 * PI / 180), z, theta_deg_2); // Rotate coordinates to +120 deg
  calculate_motor_angle(x * cos(120 * PI / 180) - y * sin(120 * PI / 180), y * cos(120 * PI / 180) + x * sin(120 * PI / 180), z, theta_deg_3); // Rotate coordinates to -120 deg
}

void calculate_motor_angle(float x0, float y0, float z0, float &theta_deg) {
  // Find y-coordinates of point S and M_m
  float S_y = -l_s * tan(30 * PI/180) / 2.0;
  float M_m_y = y0 - l_m * tan(30 * PI/180) / 2.0;

  // z = a + b*y
  float a = (x0 * x0 + M_m_y * M_m_y + z0 * z0 + r_o * r_o - r_u * r_u - S_y * S_y) / (2.0 * z0);
  float b = (S_y - M_m_y) / z0;

  // Discriminant
  float d = -(a + b * S_y) * (a + b * S_y) + r_o * (b * b * r_o + r_o);

  // Non-existing point
  if (d < 0) {
    Serial.println("Non-existing point");
    theta_deg = NULL;
    return;
  }

  // Calculate two sides of triangle made from the point A and S
  float A_y = (S_y - a * b - sqrt(d)) / (b * b + 1);
  float A_z = a + b * A_y;

  // Calculate theta
  theta_deg = atan(-A_z / (S_y - A_y)) * 180.0 / PI + ((A_y > S_y) ? 180.0 : 0.0);
}

// Calculate the position of the end-effector
void calculate_motor_position(float theta_rounds[3], float coordinates[3]) {

  // Calculate the real theta degrees
  float real_theta_degs[3];
  for (int odrive_index = 0; odrive_index < 3; odrive_index++) {
    float zero_offset_deg = zero_offset_array[odrive_index];
    float horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    float theta_rad = theta_rounds[odrive_index] * (2 * PI) / gear_ratio;
    float theta_deg = theta_rad * 180 / PI;
    real_theta_degs[odrive_index] = abs(horizontal_offset_deg) + theta_deg;
  }

  float theta_1_rad = real_theta_degs[0] * PI / 180;
  float theta_2_rad = real_theta_degs[1] * PI / 180;
  float theta_3_rad = real_theta_degs[2] * PI / 180;

  float t = ((l_s - l_m) * tan(30.0 * PI / 180)) / 2.0;

  float y_1 = -(t + r_o * cos(theta_1_rad));
  float z_1 = -r_o * sin(theta_1_rad);

  float y_2 = (t + r_o * cos(theta_2_rad)) * sin(30.0 * PI / 180);
  float x_2 = y_2 * tan(60.0 * PI / 180);
  float z_2 = -r_o * sin(theta_2_rad);

  float y_3 = (t + r_o * cos(theta_3_rad)) * sin(30.0 * PI / 180);
  float x_3 = -y_3 * tan(60.0 * PI / 180);
  float z_3 = -r_o * sin(theta_3_rad);

  float dnm = (y_2 - y_1) * x_3 - (y_3 - y_1) * x_2;

  float w_1 = y_1 * y_1 + z_1 * z_1;
  float w_2 = x_2 * x_2 + y_2 * y_2 + z_2 * z_2;
  float w_3 = x_3 * x_3 + y_3 * y_3 + z_3 * z_3;

  // x = (a_1*z + b_1)/dnm
  float a_1 = (z_2 - z_1) * (y_3 - y_1) - (z_3 - z_1) * (y_2 - y_1);
  float b_1 = -((w_2 - w_1) * (y_3 - y_1) - (w_3 - w_1) * (y_2 - y_1)) / 2.0;

  // y = (a_2*z + b_2)/dnm;
  float a_2 = -(z_2 - z_1) * x_3 + (z_3 - z_1) * x_2;
  float b_2 = ((w_2 - w_1) * x_3 - (w_3 - w_1) * x_2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a_1 * a_1 + a_2 * a_2 + dnm * dnm;
  float b = 2.0 * (a_1 * b_1 + a_2 * (b_2 - y_1 * dnm) - z_1 * dnm * dnm);
  float c = (b_2 - y_1 * dnm) * (b_2 - y_1 * dnm) + b_1 * b_1 + dnm * dnm * (z_1 * z_1 - r_u * r_u);

  // Discriminant
  float d = b * b - 4.0 * a * c;
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