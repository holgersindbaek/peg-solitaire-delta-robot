////////////////////////////////
// Startup sequence
////////////////////////////////
// 1. Detach suction cup from end-effector, detach arms from biceps, move biceps to vertical position and run `calibrate_drives()` to calibrate all drives
// 2. Attach zeroing device to board, attach arms to biceps, move end-effector onto zero-device and run `zero_drives()` to zero all drives. Run `get_position()` to make sure it's correct. Position should be (0, 0, -250).
// 3. Move end-effector up, detach zeroing device from board, and run `initialize_drives()`. Re-attach suction cup to end-effector. You're ready to go!

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
#include <Servo.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Functions
#include "helper_functions.h"
#include "kinematic_functions.h"
#include "drive_functions.h"
#include "suction_functions.h"
#include "peg_solitaire_functions.h"

// Setup vacuum
Servo solenoid_valve_servo;   // Solenoid valve
Servo air_pump_servo; // Air pump

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
float zero_offset_array[3] = {
  0.0, 0.0, 0.0
};

// HACK: ODrive doesn't always return the correct position, so save the last known position the motors
float last_thetas[3] = {NAN, NAN, NAN};

// Robot parameters. Definitions are as follows: https://imgur.com/a/edboVco.
const float l_s = 554.27;                    // Base radius (mm) (f)
const float r_o = 160.0;                     // Bicep length (mm) (rf)
const float r_u = 220.0;                     // Forearm length (mm) (re)
const float l_m = 138.57;                    // End effector length (mm) (e)
const int gear_ratio = 8;                    // Gear ratio
const float horizontal_offset_angle = -63.2; // Angle of the horizontal offset
const float x_max = 115;                     // Boundaries of the workspace
const float x_min = -115;
const float y_max = 115;
const float y_min = -115;
const float z_max = -160;
const float z_min = -250;
const float z_zero = -250;                   // Position of y when end-effector is zeroed (mm)
const float board_top_offset = -3;           // Offset from top of board to zero (mm)
const float suction_bottom_offset = 20;      // Offset for bottom of suction cup (mm)
const float suction_grab_offset = 14;        // Offset for making suction cub grab (mm)
const float min_angle_deg = 0.0;             // Min and max angles for the robot
const float max_angle_deg = 115.0;

// Trajectory parameters
// NOTE: To get a trajectory that isn't out of control, velocity and acceleration should be close to each other
const float throttle_factor = 1.0;                    // How much to throttle the trajectory - Used for testing purposes (0.0 - 1.0)
const float time_step_delta = 0.01;                   // Decide which timesteps to divide the trajectory into (s)
const float max_motor_vel = 8.0 * throttle_factor;    // Max trajectory velocity (rounds/s) (motor max is 9900RPM = 165RPS: https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y/edit#gid=0)
const float max_motor_acc = 8.0 * throttle_factor;    // Max motor acceleration (rounds/s^2)
const float max_motor_dec = 8.0 * throttle_factor;    // Max motor deceleration (rounds/s^2) (should be positive)
const float max_deg_vel = (max_motor_vel * 360) / gear_ratio;
const float max_deg_acc = (max_motor_acc * 360) / gear_ratio;
const float max_deg_dec = (max_motor_dec * 360) / gear_ratio;

void setup() {
  // ODrive uses 115200 baud
  odrive_serial_1.begin(115200);
  odrive_serial_2.begin(115200);
  odrive_serial_3.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  // Representing the air pump plug 10 pin, the solenoid valve plug 8 pin arduino control
  solenoid_valve_servo.attach(10);
  air_pump_servo.attach(8);

  // Load zero offsets from EEPROM
  for (int drive_index = 0; drive_index < 3; drive_index++) {
    float value;
    EEPROM_get_float(drive_index * 100, value);
    zero_offset_array[drive_index] = value;
    Serial.println("Loaded zero offset for drive " + String(drive_index) + ": " + String(value));
  }

  // Set parameters for ODrives
  for (int drive_index = 0; drive_index < 3; ++drive_index) {
    odrive_serial_array[drive_index]->println("w axis0.controller.config.vel_limit " + String(max_motor_vel));
    odrive_serial_array[drive_index]->println("w axis0.controller.config.vel_limit_tolerance " + String(2.0)); // 100%  overshot allowance
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

  Serial.println("max_motor_vel " + String(max_motor_vel));
  Serial.println("max_deg_vel " + String(max_deg_vel));
  Serial.println("max_deg_acc " + String(max_deg_acc));
}

void loop() {
  // Wait for incoming commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    String function_name = input.substring(0, input.indexOf('('));
    String parameter_string = input.substring(input.indexOf('(') + 1, input.indexOf(')'));
    int num_params = count_occurrences(parameter_string, ',') + 1;  // count the number of parameters in the input string
    float params[num_params]; // create an array to store the parameters
    extract_parameters(parameter_string, params, num_params); // extract the parameters from the input string
    Serial << "Running " << function_name << "...\n";

    if (function_name == "calibrate_drives") {
      calibrate_drives();
    } else if (function_name == "calibrate_drive") {
      int odrive_number = round(params[0]);
      calibrate_drive(odrive_number);
    } else if (function_name == "zero_drives") {
      zero_drives();
    } else if (function_name == "initialize_drives") {
      initialize_drives();
    } else if (function_name == "set_drives_idle") {
      set_drives_idle();
    } else if (function_name == "get_status") {
      get_status();
    } else if (function_name == "get_position") {
      get_position();
    } else if (function_name == "get_angles") {
      get_angles();
    } else if (function_name == "save_angles") {
      save_angles();
    } else if (function_name == "run_complete_calibration_sequence") {
      run_complete_calibration_sequence(); // Used for testing purposes
    } else if (function_name == "move_to_position") {
      float x = params[0];
      float y = params[1];
      float z = params[2];
      move_to_position(x, y, z);
    } else if (function_name == "set_position") {
      float x = params[0];
      float y = params[1];
      float z = params[2];
      set_position(x, y, z);
    } else if (function_name == "set_angle") {
      int odrive_number = round(params[0]);
      float theta = params[1];
      set_angle(odrive_number, theta);
    } else if (function_name == "set_suction_state") {
      int state = params[0];
      set_suction_state(state);
    } else if (function_name == "play_winning_peg_solitaire") {
      play_winning_peg_solitaire();
    } else if (function_name == "play_random_peg_solitaire") {
      play_random_peg_solitaire();
    }

    Serial << "Finished " << function_name << "...\n\n";
  }

  // // Set position of end-effector if current position is set
  // if (current_position[0] != -1.0 && current_position[1] != -1.0 && current_position[2] != -1.0) {
  //   set_position(current_position[0], current_position[1], current_position[2]);
  // }
}