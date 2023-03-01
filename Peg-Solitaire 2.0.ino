////////////////////////////////
// Startup sequence
////////////////////////////////
// 1. Detach arms from biceps, move biceps to vertical position and run `calibrate_drives()` to calibrate all drives
// 2. Attach zeroing device to board, attach arms to biceps, move end-effector onto zero-device and run `zero_drives()` to zero all drives.
// 3. Move end-effector up, detach zeroing device from board, and run `set_drives_closed_loop_control()`. You're ready to go!

////////////////////////////////
// Main commands (all available commands can be found in the `loop()` function).
////////////////////////////////
// `move_to_position(x, y, z)` - Move end-effector to position (x, y, z).
// `play_winning_peg_solitaire()` - Play winning peg solitaire game. Remember to set up the board before you start.
// `get_status(val)` - Get position ("pos_rel") or velocity ("vel_rel") of drives.

// Include libraries
#include <math.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <EEPROM.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Functions
#include "helper_functions.h"
#include "kinematic_functions.h"
#include "drive_functions.h"
#include "peg_solitaire_functions.h"

////////////////////////////////
// Set up ODrive
////////////////////////////////
// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial &odrive_serial_1 = Serial1;
HardwareSerial &odrive_serial_2 = Serial2;
HardwareSerial &odrive_serial_3 = Serial3;
HardwareSerial *odrive_serial_array[3] = {
  &odrive_serial_1, &odrive_serial_2, &odrive_serial_3
};

// ODrive object
ODriveArduino odrive_1(odrive_serial_1);
ODriveArduino odrive_2(odrive_serial_2);
ODriveArduino odrive_3(odrive_serial_3);
ODriveArduino odrive_array[3] = {
  odrive_1, odrive_2, odrive_3
};

// Zero offsets
double zero_offset_array[3] = {
  0.0, 0.0, 0.0
};

// Robot parameters. Definitions are as follows: https://imgur.com/a/edboVco.
const double l_s = 554.27;                    // Base radius (mm) (f)
const double r_o = 160.0;                     // Bicep length (mm) (rf)
const double r_u = 220.0;                     // Forearm length (mm) (re)
const double l_m = 138.57;                    // End effector length (mm) (e)
const int gear_ratio = 8;                     // Gear ratio
const float horizontal_offset_angle = -63.2;  // Angle of the horizontal offset

// Trajectory parameters
const float time_step_delta = 1.0; // Decide which timesteps to divide the trajectory into
const float max_vel = 20.0; // Max motor velocity (rounds/s)
const float max_acc = 0.5;  // Max motor acceleration (rounds/s^2)
const float max_dec = 0.5;  // Max motor deceleration (rounds/s^2) (should be positive)

void setup() {
  // ODrive uses 115200 baud
  odrive_serial_1.begin(115200);
  odrive_serial_2.begin(115200);
  odrive_serial_3.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  // Load zero offsets from EEPROM
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    double value;
    EEPROM_get_double(drive_index * 100, value);
    zero_offset_array[drive_index] = value;
    Serial.println("Loaded zero offset for drive " + String(drive_index) + ": " + String(value));
  }

  // Set parameters for ODrives
  for (int drive_index = 0; drive_index < 3; ++drive_index) {
    odrive_serial_array[drive_index]->println("w axis0.controller.config.vel_limit " + String(max_vel));
    odrive_serial_array[drive_index]->println("w axis0.controller.config.vel_limit_tolerance " + String(2.0)); // 20% tolerance (overshot)
  }

  // HACK: The function `GetPosition` doesn't return the correct value until after it's been called twice
  for (int i = 0; i < 3; i++) {
    for (int drive_index = 0; drive_index < 3; drive_index++) {
      odrive_array[drive_index].GetPosition(0);
    }
  }

  // Welcome text in the prompt
  Serial.println("////////////////////////////////");
  Serial.println("// Ready for your commands...");
  Serial.println("////////////////////////////////");
}

void loop() {
  // Wait for incoming commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    String function_name = input.substring(0, input.indexOf('('));
    String parameter_string = input.substring(input.indexOf('(') + 1, input.indexOf(')'));
    int num_params = count_occurrences(parameter_string, ',') + 1;  // count the number of parameters in the input string
    double params[num_params]; // create an array to store the parameters
    extract_parameters(parameter_string, params, num_params); // extract the parameters from the input string
    Serial << "Running " << function_name << "...\n";

    if (function_name == "calibrate_drive") {
      int odrive_number = round(params[0]);
      calibrate_drive(odrive_number);
    } else if (function_name == "calibrate_drives") {
      calibrate_drives();
    } else if (function_name == "set_drives_idle") {
      set_drives_idle();
    } else if (function_name == "set_drives_closed_loop_control") {
      set_drives_closed_loop_control();
    } else if (function_name == "zero_drives") {
      zero_drives();
    } else if (function_name == "get_status") {
      get_status();
    } else if (function_name == "get_position") {
      get_position();
    } else if (function_name == "move_to_position") {
      double x = params[0];
      double y = params[1];
      double z = params[2];
      move_to_position(x, y, z);
    } else if (function_name == "set_angle") {
      int odrive_number = round(params[0]);
      double theta = params[1];
      set_angle(odrive_number, theta);
    } else if (function_name == "play_winning_peg_solitaire") {
      play_winning_peg_solitaire();
    }

    Serial << "Finished " << function_name << "...\n\n";
  }
}