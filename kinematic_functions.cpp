#include "kinematic_functions.h"

// Move the robot to a specified end effector position
void move_to_position(float x, float y, float z, float speed_throttle) {
  // Calculate the real start theta degrees
  float start_thetas[3];
  float start_degs[3];
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    if (isnan(last_thetas[drive_index])) {
      start_thetas[drive_index] = odrive_array[drive_index].GetPosition(0);
    } else {
      start_thetas[drive_index] = last_thetas[drive_index];
    }

    float zero_offset_deg = zero_offset_array[drive_index];
    float horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    float theta_rad = start_thetas[drive_index] * (2 * PI) / gear_ratio;
    float theta_deg = theta_rad * 180 / PI;
    start_degs[drive_index] = abs(horizontal_offset_deg) + theta_deg;
  }

  // Calculate end theta degrees
  float end_degs[3];
  float end_thetas[3];
  calculate_motor_angles(x, y, z, end_degs);
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    float zero_offset_deg = zero_offset_array[drive_index];
    float horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    float real_theta_deg = horizontal_offset_deg + end_degs[drive_index];
    float real_theta_rad = real_theta_deg * PI / 180;
    end_thetas[drive_index] = real_theta_rad / (2 * PI) * gear_ratio;
  }

  // Get start position
  float start_pos[3];
  float end_pos[3] = {x, y, z};
  calculate_motor_position(start_thetas, start_pos);

  // Calculate distance between start and end and position
  float pos_distances[3] = {abs(end_pos[0] - start_pos[0]), abs(end_pos[1] - start_pos[1]), abs(end_pos[2] - start_pos[2])};
  float max_pos_distance = 0;
  int max_pos_distance_index = 0;
  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    if (pos_distances[coordinate_index] > max_pos_distance) {
      max_pos_distance = pos_distances[coordinate_index];
      max_pos_distance_index = coordinate_index;
    }
  }

  // Calculate distance ratios
  float pos_distance_ratios[3] = {1.0, 1.0, 1.0};
  for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
    pos_distance_ratios[coordinate_index] = pos_distances[coordinate_index] / pos_distances[max_pos_distance_index];
  }

  // Calculate the max acceleration and velocity so the motor reached max velocity at 1/4 the distance
  // https://sciencing.com/acceleration-velocity-distance-7779124.html
  float max_pos_traj_acc_distance = max_pos_distance * 0.25;
  float max_pos_traj_acc;
  float max_pos_traj_vel;
  for (int max_pos_vel_index = max_pos_vel * speed_throttle; max_pos_vel_index > 0; max_pos_vel_index--) {
    max_pos_traj_acc = pow(max_pos_vel_index, 2) / (2 * max_pos_traj_acc_distance);

    if (max_pos_traj_acc <= max_pos_acc * speed_throttle) {
      max_pos_traj_vel = max_pos_vel_index;
      break;
    }
  }
  float max_pos_traj_vels[3] = {max_pos_traj_vel * pos_distance_ratios[0], max_pos_traj_vel * pos_distance_ratios[1], max_pos_traj_vel * pos_distance_ratios[2]};
  float max_pos_traj_accs[3] = {max_pos_traj_acc * pos_distance_ratios[0], max_pos_traj_acc * pos_distance_ratios[1], max_pos_traj_acc * pos_distance_ratios[2]};

  // Get the trapezoidal trajectory values for each coordinate
  TrajValues traj_positions[3];
  get_traj(start_pos, end_pos, max_pos_traj_vels, max_pos_traj_accs, traj_positions);

  // Calculate distance between start and end and get max distance
  float theta_distances[3] = {abs(end_thetas[0] - start_thetas[0]), abs(end_thetas[1] - start_thetas[1]), abs(end_thetas[2] - start_thetas[2])};
  float max_theta_distance = 0;
  int max_theta_distance_index = 0;
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    if (theta_distances[drive_index] > max_theta_distance) {
      max_theta_distance = theta_distances[drive_index];
      max_theta_distance_index = drive_index;
    }
  }

  // Calculate distance ratios
  float theta_distance_ratios[3] = {1.0, 1.0, 1.0};
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    theta_distance_ratios[drive_index] = theta_distances[drive_index] / theta_distances[max_theta_distance_index];
  }

  // Calculate the max acceleration and velocity so the motor reached max velocity at 1/4 the distance
  // https://sciencing.com/acceleration-velocity-distance-7779124.html
  float max_theta_traj_acc_distance = max_theta_distance * 0.25;
  float max_theta_traj_acc;
  float max_theta_traj_vel;
  for (int max_theta_vel_index = max_theta_vel * speed_throttle; max_theta_vel_index > 0; max_theta_vel_index--) {
    max_theta_traj_acc = pow(max_theta_vel_index, 2) / (2 * max_theta_traj_acc_distance);

    if (max_theta_traj_acc <= max_theta_acc * speed_throttle) {
      max_theta_traj_vel = max_theta_vel_index;
      break;
    }
  }
  float max_theta_traj_vels[3] = {max_theta_traj_vel * theta_distance_ratios[0], max_theta_traj_vel * theta_distance_ratios[1], max_theta_traj_vel * theta_distance_ratios[2]};
  float max_theta_traj_accs[3] = {max_theta_traj_acc * theta_distance_ratios[0], max_theta_traj_acc * theta_distance_ratios[1], max_theta_traj_acc * theta_distance_ratios[2]};

  // Get the trapezoidal trajectory values for each motor
  TrajValues traj_thetas[3];
  get_traj(start_thetas, end_thetas, max_theta_traj_vels, max_theta_traj_accs, traj_thetas);

  // If max steps are less than 3, then just move the motors to the end position
  long pos_steps = ceilf(traj_positions[max_pos_distance_index].complete_time_step / time_step_delta);
  if (pos_steps <= 3) {
    Serial.println("Moving directly to position!");

    set_angle(1, end_degs[0], 0.0);
    set_angle(2, end_degs[1], 0.0);
    set_angle(3, end_degs[2], 0.0);

    return;
  }

  // Check to see if trajectories are unstable
  long vel_steps = ceilf(traj_thetas[max_theta_distance_index].complete_time_step / time_step_delta);
  float complete_pos_time = pos_steps * time_step_delta;
  float complete_vel_time = vel_steps * time_step_delta;
  float positive_directions[3] = {start_pos[0] < end_pos[0], start_pos[1] < end_pos[1], start_pos[2] < end_pos[2]};
  float prev_pos[3] = {start_pos[0], start_pos[1], start_pos[2]};
  for (int step_index = 0; step_index <= pos_steps; step_index++) {
    TrajStep traj_pos_step;
    TrajStep traj_theta_step;
    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float pos_time_step = time_step_delta * step_index;
      TrajValues traj_pos_value = traj_positions[coordinate_index];
      get_traj_step(traj_pos_value.start_pos, traj_pos_value.end_pos, pos_time_step, traj_pos_value.acc_time_step, traj_pos_value.vel_time_step, traj_pos_value.complete_time_step, traj_pos_value.start_vel, traj_pos_value.max_acc_signed, traj_pos_value.max_dec_signed, traj_pos_value.max_vel_signed, traj_pos_value.y_acc, traj_pos_step, coordinate_index);

      float percentage_done = pos_time_step / complete_pos_time;
      float vel_time_step = traj_thetas[max_theta_distance_index].complete_time_step * percentage_done;
      TrajValues traj_theta_value = traj_thetas[coordinate_index];
      get_traj_step(traj_theta_value.start_pos, traj_theta_value.end_pos, vel_time_step, traj_theta_value.acc_time_step, traj_theta_value.vel_time_step, traj_theta_value.complete_time_step, traj_theta_value.start_vel, traj_theta_value.max_acc_signed, traj_theta_value.max_dec_signed, traj_theta_value.max_vel_signed, traj_theta_value.y_acc, traj_theta_step, coordinate_index);
    }

    // Useful for debugging
    // Serial.println("step_index: " + String(String(step_index) + "      ").substring(0, 3) + " || " + "coordinates: " + String(traj_pos_step.pos[0]) + " - " + String(traj_pos_step.pos[1]) + " - " + String(traj_pos_step.pos[2]) + " || theta vel: " + String(traj_theta_step.vel[0]) + " - " + String(traj_theta_step.vel[1]) + " - " + String(traj_theta_step.vel[2]) + " || theta acc: " + String(traj_theta_step.acc[0]) + " - " + String(traj_theta_step.acc[1]) + " - " + String(traj_theta_step.acc[2]));

    float margin = 0.1; // Add a margin to the position to account for rounding errors
    if (
      (positive_directions[0] && (traj_pos_step.pos[0] + margin < prev_pos[0] || traj_pos_step.pos[0] > end_pos[0])) ||
      (!positive_directions[0] && (traj_pos_step.pos[0] - margin > prev_pos[0] || traj_pos_step.pos[0] < end_pos[0])) ||
      (positive_directions[1] && (traj_pos_step.pos[1] + margin < prev_pos[1] || traj_pos_step.pos[1] > end_pos[1])) ||
      (!positive_directions[1] && (traj_pos_step.pos[1] - margin > prev_pos[1] || traj_pos_step.pos[1] < end_pos[1])) ||
      (positive_directions[2] && (traj_pos_step.pos[2] + margin < prev_pos[2] || traj_pos_step.pos[2] > end_pos[2])) ||
      (!positive_directions[2] && (traj_pos_step.pos[2] - margin > prev_pos[2] || traj_pos_step.pos[2] < end_pos[2])))
    {
      Serial.println("Couldn't move along path. Trajectory is unstable. We suggest you change velocity and acceleration values.");
      return;
    }

    prev_pos[0] = traj_pos_step.pos[0];
    prev_pos[1] = traj_pos_step.pos[1];
    prev_pos[2] = traj_pos_step.pos[2];
  }

  // Move the motors along the trajectory
  float vel_pos_ratio = complete_vel_time / complete_pos_time;
  float vel_time_step_delta = vel_pos_ratio * time_step_delta;
  unsigned long start_time = millis();
  for (int step_index = 0; step_index <= pos_steps; step_index++) {
    // Get time passed and time passed target to calculate delay
    unsigned long time_passed = millis() - start_time;
    unsigned long time_passed_target = step_index * (vel_time_step_delta * 1000); // Convert to milliseconds
    unsigned long time_delay = min(max(time_passed_target - time_passed, 0.0), vel_time_step_delta * 1000);

    // Delay until next step
    delay(time_delay);

    // Get step
    TrajStep traj_pos_step;
    TrajStep traj_theta_step;
    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float pos_time_step = time_step_delta * step_index;
      TrajValues traj_pos_value = traj_positions[coordinate_index];
      get_traj_step(traj_pos_value.start_pos, traj_pos_value.end_pos, pos_time_step, traj_pos_value.acc_time_step, traj_pos_value.vel_time_step, traj_pos_value.complete_time_step, traj_pos_value.start_vel, traj_pos_value.max_acc_signed, traj_pos_value.max_dec_signed, traj_pos_value.max_vel_signed, traj_pos_value.y_acc, traj_pos_step, coordinate_index);

      float percentage_done = pos_time_step / complete_pos_time;
      float vel_time_step = traj_thetas[max_theta_distance_index].complete_time_step * percentage_done;
      TrajValues traj_theta_value = traj_thetas[coordinate_index];
      get_traj_step(traj_theta_value.start_pos, traj_theta_value.end_pos, vel_time_step, traj_theta_value.acc_time_step, traj_theta_value.vel_time_step, traj_theta_value.complete_time_step, traj_theta_value.start_vel, traj_theta_value.max_acc_signed, traj_theta_value.max_dec_signed, traj_theta_value.max_vel_signed, traj_theta_value.y_acc, traj_theta_step, coordinate_index);
    }

    // Move the motors to the calculated positions with the calculated velocities
    set_position(traj_pos_step.pos[0], traj_pos_step.pos[1], traj_pos_step.pos[2], traj_theta_step.vel[0], traj_theta_step.vel[1], traj_theta_step.vel[2]);
  }
  unsigned long elapsed_time = millis() - start_time;
}

void set_position(float x, float y, float z, float vel_1, float vel_2, float vel_3) {
  // Calculate angles
  float theta_degs[3];
  calculate_motor_angles(x, y, z, theta_degs);

  // Move the motors to the calculated angles
  set_angle(1, theta_degs[0], abs(vel_1));
  set_angle(2, theta_degs[1], abs(vel_2));
  set_angle(3, theta_degs[2], abs(vel_3));

  // Set current position
  current_pos[0] = x;
  current_pos[1] = y;
  current_pos[2] = z;
}

void set_angle(int odrive_number, float theta_deg, float theta_vel) {
  if (theta_deg == NULL) {
    Serial.println("Position is out of bounds.");
    return;
  }

  // Check to see if angle will collide with spacer
  if (max_angle_deg < theta_deg || min_angle_deg > theta_deg) {
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

  // Calculate velocity in rounds per second in relation to degrees
  float vel_rounds = abs(theta_vel / 360 * gear_ratio);

  // NOTE: There are different was of setting position on ODrive: https://docs.odriverobotics.com/v/latest/ascii-protocol.html#motor-position
  // Set the motor angle
  odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, vel_rounds);

  // HACK: Save last position so we don't need to query ODrive
  last_thetas[odrive_number - 1] = real_theta_rounds;
}

// Calculate the motor angles for a given end effector position
void calculate_motor_angles(float x, float y, float z, float theta_degs[3]) {
  calculate_motor_angle(x, y, z, theta_degs[0]);
  calculate_motor_angle(x * cos(120 * PI / 180) + y * sin(120 * PI / 180), y * cos(120 * PI / 180) - x * sin(120 * PI / 180), z, theta_degs[1]); // Rotate coordinates to +120 deg
  calculate_motor_angle(x * cos(120 * PI / 180) - y * sin(120 * PI / 180), y * cos(120 * PI / 180) + x * sin(120 * PI / 180), z, theta_degs[2]); // Rotate coordinates to -120 deg
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