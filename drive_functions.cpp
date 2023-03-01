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

    valuesString += String("| " + String(returned_value) + "      ").substring(0, 11);
  }

  Serial.println(valuesString + "|");
  Serial.println("+----------+----------+----------+----------+");
}

void calibrate_drives() {
  for (int index = 0; index < 3; index++) {
    calibrate_drive(index + 1);
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

void set_drives_closed_loop_control() {
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    Serial.println("ODrive " + String(drive_index + 1) + ": Requesting state " + AXIS_STATE_CLOSED_LOOP_CONTROL);
    if (!odrive_array[drive_index].run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 25.0f)) {
      Serial.println("Failed");
      return;
    }
  }
}

void zero_drives() {
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    delay(500);

    // Grab relative position from encoder
    float rel_pos = odrive_array[drive_index].GetPosition(0);
    Serial.println(rel_pos);

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