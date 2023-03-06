#ifndef DRIVE_FUNCTIONS_H
#define DRIVE_FUNCTIONS_H
#include <Arduino.h>
#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include "helper_functions.h"
#include "kinematic_functions.h"

  extern ODriveArduino odrive_array[];
  extern HardwareSerial *odrive_serial_array[];
  extern float zero_offset_array[];
  extern const float horizontal_offset_angle;
  extern const int gear_ratio;

  void get_status(String value = "pos_rel");
  void get_position();
  void get_angles();
  void save_angles();
  void calibrate_drive(int odrive_number);
  void calibrate_drives();
  void zero_drives();
  void initialize_drives();
  void set_drives_idle();
  void run_complete_calibration_sequence();

#endif