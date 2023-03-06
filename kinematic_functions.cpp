#include "kinematic_functions.h"

// Move the robot to a specified end effector position
void move_to_position(float x, float y, float z) {
  // Calculate the real start theta degrees
  float start_thetas[3];
  float start_degs[3];
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    Serial.println("isnan(last_thetas[drive_index]): " + String(isnan(last_thetas[drive_index])));
    Serial.println("last_thetas[drive_index]: " + String(last_thetas[drive_index]));
    Serial.println("odrive_array[drive_index].GetPosition(0): " + String(odrive_array[drive_index].GetPosition(0)));
    if (isnan(last_thetas[drive_index])) {
      Serial.println("HERE 1");
      start_thetas[drive_index] = odrive_array[drive_index].GetPosition(0);
    } else {
      Serial.println("HERE 2");
      start_thetas[drive_index] = last_thetas[drive_index];
    }

    float zero_offset_deg = zero_offset_array[drive_index];
    float horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    float theta_rad = start_thetas[drive_index] * (2 * PI) / gear_ratio;
    float theta_deg = theta_rad * 180 / PI;
    start_degs[drive_index] = abs(horizontal_offset_deg) + theta_deg;
  }

  Serial.println("start_degs: " + String(start_degs[0]) + ", " + String(start_degs[1]) + ", : " + String(start_degs[2]));

  // Calculate end theta degrees
  float end_degs[3];
  calculate_motor_angles(x, y, z, end_degs);
  Serial.println("end_degs: " + String(end_degs[0]) + ", " + String(end_degs[1]) + ", : " + String(end_degs[2]));

  // Calculate distance between start and end and get max distance
  float distances[3] = {abs(end_degs[0] - start_degs[0]), abs(end_degs[1] - start_degs[1]), abs(end_degs[2] - start_degs[2])};
  float max_distance = 0;
  int max_distance_index = 0;
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    Serial.println("distances[drive_index]: " + String(distances[drive_index]));
    if (distances[drive_index] > max_distance) {
      max_distance = distances[drive_index];
      max_distance_index = drive_index;
    }
  }

  // Calculate distance ratios
  float distance_ratios[3] = {1.0, 1.0, 1.0};
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    distance_ratios[drive_index] = distances[drive_index] / distances[max_distance_index];
  }

  // Calculate the max acceleration and velocity so the motor reached max velocity at 1/4 the distance
  // https://sciencing.com/acceleration-velocity-distance-7779124.html
  float max_deg_traj_acc_distance = max_distance / 4;
  float max_deg_traj_acc;
  float max_deg_traj_vel;
  for (int max_deg_vel_index = max_deg_vel; max_deg_vel_index > 0; max_deg_vel_index--) {
    max_deg_traj_acc = pow(max_deg_vel_index, 2) / (2 * max_deg_traj_acc_distance);

    // Serial.println("--");
    // Serial.println("max_deg_vel_index: " + String(max_deg_vel_index));
    // Serial.println("max_deg_traj_acc: " + String(max_deg_traj_acc));
    if (max_deg_traj_acc <= max_deg_acc) {
      max_deg_traj_vel = max_deg_vel_index;
      // Serial.println("max_deg_traj_vel: " + String(max_deg_traj_vel));
      break;
    }
  }
  Serial.println("max_deg_traj_acc_distance: " + String(max_deg_traj_acc_distance));
  Serial.println("max_deg_traj_acc: " + String(max_deg_traj_acc));
  Serial.println("max_deg_traj_vel: " + String(max_deg_traj_vel));

  // Get the trapezoidal trajectory values for each coordinate
  TrajValues traj_values[3];
  get_traj(start_degs, end_degs, max_deg_traj_vel, max_deg_traj_acc, distance_ratios, traj_values);

  // If max steps are less than 3, then just move the motors to the end position
  long steps = ceilf(traj_values[0].complete_time_step / time_step_delta);
  if (steps <= 3) {
    Serial.println("Moving directly to position!");

    set_angle(1, end_degs[0], 0.0);
    set_angle(2, end_degs[1], 0.0);
    set_angle(3, end_degs[2], 0.0);

    return;
  }

  // Check to see if trajectories are unstable
  float positive_directions[3] = {start_degs[0] < end_degs[0], start_degs[1] < end_degs[1], start_degs[2] < end_degs[2]};
  float prev_degs[3] = {start_degs[0], start_degs[1], start_degs[2]};
  for (int step_index = 0; step_index <= steps; step_index++) {
    TrajStep traj_step;
    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float time_step = time_step_delta * step_index;
      TrajValues traj_value = traj_values[coordinate_index];

      get_traj_step(traj_value.start_deg, traj_value.end_deg, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.start_vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
    }

    // Print out trajectory step values
    float dis[3] = {abs(traj_step.deg[0] - prev_degs[0]), abs(traj_step.deg[1] - prev_degs[1]), abs(traj_step.deg[2] - prev_degs[2])};
    // Serial.println("step_index: " + String(String(step_index) + "      ").substring(0, 3) + " || " + "deg: " + String(traj_step.deg[0]) + " - " + String(traj_step.deg[1]) + " - " + String(traj_step.deg[2]) + " || vel: " + String(traj_step.vel[0]) + " - " + String(traj_step.vel[1]) + " - " + String(traj_step.vel[2]) + " || acc: " + String(traj_step.acc[0]) + " - " + String(traj_step.acc[1]) + " - " + String(traj_step.acc[2]));
    Serial.println("step_index: " + String(String(step_index) + "      ").substring(0, 3) + " || " + "dis: " + String(dis[0]) + " - " + String(dis[1]) + " - " + String(dis[2]) + " || vel: " + String(traj_step.vel[0]) + " - " + String(traj_step.vel[1]) + " - " + String(traj_step.vel[2]) + " || acc: " + String(traj_step.acc[0]) + " - " + String(traj_step.acc[1]) + " - " + String(traj_step.acc[2]));

    // if (
    //   (positive_directions[0] && (traj_step.deg[0] < prev_degs[0] || traj_step.deg[0] > end_degs[0])) ||
    //   (!positive_directions[0] && (traj_step.deg[0] > prev_degs[0] || traj_step.deg[0] < end_degs[0])) ||
    //   (positive_directions[1] && (traj_step.deg[1] < prev_degs[1] || traj_step.deg[1] > end_degs[1])) ||
    //   (!positive_directions[1] && (traj_step.deg[1] > prev_degs[1] || traj_step.deg[1] < end_degs[1])) ||
    //   (positive_directions[2] && (traj_step.deg[2] < prev_degs[2] || traj_step.deg[2] > end_degs[2])) ||
    //   (!positive_directions[2] && (traj_step.deg[2] > prev_degs[2] || traj_step.deg[2] < end_degs[2]))) {
    //   Serial.println("Couldn't move along path. Trajectory is unstable. We suggest you change velocity and acceleration values.");
    //   return;
    // }

    prev_degs[0] = traj_step.deg[0];
    prev_degs[1] = traj_step.deg[1];
    prev_degs[2] = traj_step.deg[2];
  }

  // Move the motors along the trajectory
  float calculated_time_to_complete = steps * time_step_delta;
  Serial.println("steps: " + String(steps));
  Serial.println("time_step_delta (s): " + String(time_step_delta));
  Serial.println("calculated_time_to_complete (s): " + String(calculated_time_to_complete));
  for (int step_index = 0; step_index <= steps; step_index++) {
    TrajStep traj_step;
    for (int coordinate_index = 0; coordinate_index < 3; coordinate_index++) {
      float time_step = time_step_delta * step_index;
      TrajValues traj_value = traj_values[coordinate_index];

      get_traj_step(traj_value.start_deg, traj_value.end_deg, time_step, traj_value.acc_time_step, traj_value.vel_time_step, traj_value.complete_time_step, traj_value.start_vel, traj_value.max_acc_signed, traj_value.max_dec_signed, traj_value.max_vel_signed, traj_value.y_acc, traj_step, coordinate_index);
    }

    // Move the motors to the calculated angles
    Serial.println("--");
    set_angle(1, traj_step.deg[0], abs(traj_step.vel[0]));
    set_angle(2, traj_step.deg[1], abs(traj_step.vel[1]));
    set_angle(3, traj_step.deg[2], abs(traj_step.vel[2]));

    // Delay until next step
    delay(time_step_delta * 1000); // Convert to milliseconds
  }
}

void set_position(float x, float y, float z, float vel_1, float vel_2, float vel_3) {
  // Calculate angles
  float theta_degs[3];
  calculate_motor_angles(x, y, z, theta_degs);

  // Move the motors to the calculated angles
  set_angle(1, theta_degs[0], abs(vel_1));
  set_angle(2, theta_degs[1], abs(vel_2));
  set_angle(3, theta_degs[2], abs(vel_3));
}

void set_angle(int odrive_number, float theta_deg, float vel) {
  if (theta_deg == NULL) {
    Serial.println("Position is out of bounds.");
    return;
  }

  // Check to see if angle will collide with spacer
  if (min_angle_deg > theta_deg || min_angle_deg > theta_deg) {
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

  // Calculate velocity in rounds per second
  float vel_rounds = abs(vel / 360 * gear_ratio);
  // vel_rounds = max(vel_rounds, 0.1); // Make sure velocity is at least 0.1 rounds per second

  Serial.println("set_angle odrive_number: " + String(odrive_number) + " || theta_deg: " + String(theta_deg) + " || vel: " + String(vel) + " || real_theta_rounds: " + String(real_theta_rounds) + " || vel_rounds: " + String(vel_rounds));

  // TODO: Find out if we can se velocity using `q` (https://docs.odriverobotics.com/v/latest/ascii-protocol.html#motor-position) or figure out what feed-forward is
  // Set the motor angle
  // odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, max_deg_vel, 0.1);
  odrive_array[odrive_number - 1].SetPosition(0, real_theta_rounds, vel_rounds);

  // HACK: Save last position so we don't need to query ODrive
  last_thetas[odrive_number - 1] = real_theta_rounds;
  // Serial.println("-- " + String(odrive_number) + " || " + String(theta_deg) + " || " + String(real_theta_rounds) + " || " + String(vel_rounds));
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