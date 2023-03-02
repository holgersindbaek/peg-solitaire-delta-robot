#include "drive_functions.h"

void get_status(String value = "pos_rel") {
  // Read the position from the encoder (https://docs.odriverobotics.com/v/latest/commands.html#encoder-position-and-velocity)

  Serial.println("+----------+----------+----------+----------+");
  Serial.println("|          | ODrive 1 | ODrive 2 | ODrive 3 |");
  Serial.println("+----------+----------+----------+----------+");
  String valuesString = String("| " + value + "     ").substring(0, 11);

  for (int index = 0; index < 3; index++) {
    // odrive_serial_array[index]->println(value.value);
    // valuesString += String("| " + String(odrive_array[index].readFloat()) + "      ").substring(0, 11);
    float returned_value;
    if (value == "pos_rel") {
      returned_value = odrive_array[index].GetPosition(0);
    } else if (value == "vel_rel") {
      returned_value = odrive_array[index].GetVelocity(0);
    }

    char value_buffer[20];
    dtostrf(returned_value, 1, 3, value_buffer);
    valuesString += String("| " + String(value_buffer) + "      ").substring(0, 11);
  }

  Serial.println(valuesString + "|");
  Serial.println("+----------+----------+----------+----------+");
}

void get_position() {
  float start_thetas[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};
  float coordinates[3];
  calculate_motor_position(start_thetas, coordinates);

  Serial.println("+----------+----------+----------+----------+");
  Serial.println("|          | x        | y        | z        |");
  Serial.println("+----------+----------+----------+----------+");
  String valuesString = String("| Position      ").substring(0, 11);

  for (int index = 0; index < 3; index++) {
    char value_buffer[20];
    dtostrf(coordinates[index], 1, 3, value_buffer);
    valuesString += String("| " + String(value_buffer) + "      ").substring(0, 11);
  }

  Serial.println(valuesString + "|");
  Serial.println("+----------+----------+----------+----------+");
}

void get_angles() {
  float theta_rounds[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};

  Serial.println("+----------+----------+----------+----------+");
  Serial.println("|          | ODrive 1 | ODrive 2 | ODrive 3 |");
  Serial.println("+----------+----------+----------+----------+");
  String valuesString = String("| Theta (deg)      ").substring(0, 11);

  for (int index = 0; index < 3; index++) {
    double zero_offset_deg = zero_offset_array[index];
    double horizontal_offset_deg = horizontal_offset_angle + zero_offset_deg;
    double theta_rad = theta_rounds[index] * (2 * PI) / gear_ratio;
    double theta_deg = theta_rad * 180 / PI;
    double real_theta_deg = abs(horizontal_offset_deg) + theta_deg;

    char real_theta_deg_buffer[20];
    dtostrf(real_theta_deg, 1, 3, real_theta_deg_buffer);
    valuesString += String("| " + String(real_theta_deg_buffer) + "      ").substring(0, 11);
  }

  Serial.println(valuesString + "|");
  Serial.println("+----------+----------+----------+----------+");
}

void calibrate_drives() {
  for (int index = 0; index < 3; index++) {
    calibrate_drive(index + 1);
    delay(5000);
  }
}

void calibrate_drive(int odrive_number) {
  Serial.println("Calibrating drive " + String(odrive_number) + "...");
  int requested_states[2] = { 
    AXIS_STATE_MOTOR_CALIBRATION, 
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION
  };

  for (int state_index = 0; state_index < 2; state_index++) {
    Serial.println("ODrive " + String(odrive_number) + ": Requesting state " + requested_states[state_index]);
    if (!odrive_array[odrive_number - 1].run_state(0, requested_states[state_index], true)) {
      Serial.println("Failed");
      return;
    }
  }

  // HACK: The function `GetPosition` doesn't return the correct value until after it's been called twice
  for (int i = 0; i < 3; i++) {
    for (int drive_index = 0; drive_index < 3; drive_index++) {
      odrive_array[drive_index].GetPosition(0);
    }
  }
}

void zero_drives() {
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    delay(500);

    // Grab relative position from encoder
    float rel_pos = odrive_array[drive_index].GetPosition(0);
    delay(500);

    // Convert position to degrees with gear radius
    double pos_rounds = rel_pos / gear_ratio;
    double pos_rad = pos_rounds * 2 * PI;
    double pos_deg = (pos_rad * 180) / PI;

    // Save offset to array
    zero_offset_array[drive_index] = pos_deg;

    // Save offset to Arduino EEPROM
    EEPROM_set_double(drive_index * 100, pos_deg);

    Serial.println("ODrive " + String(drive_index + 1) + " zero position: " + String(pos_deg) + " degrees (" + String(rel_pos) + " counts)");
  }
}

void initialize_drives() {
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    Serial.println("ODrive " + String(drive_index + 1) + ": Requesting state " + AXIS_STATE_CLOSED_LOOP_CONTROL);
    if (!odrive_array[drive_index].run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 25.0f)) {
      Serial.println("Failed");
      return;
    }
  }

  // HACK: The function `GetPosition` doesn't return the correct value until after it's been called twice
  for (int i = 0; i < 3; i++) {
    for (int drive_index = 0; drive_index < 3; drive_index++) {
      odrive_array[drive_index].GetPosition(0);
    }
  }
}

void set_drives_idle() {
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    Serial.println("ODrive " + String(drive_index + 1) + ": Requesting state " + AXIS_STATE_IDLE);
    if (!odrive_array[drive_index].run_state(0, AXIS_STATE_IDLE, false, 25.0f)) {
      Serial.println("Failed");
      return;
    }
  }
}