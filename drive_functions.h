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
  extern double zero_offset_array[];
  extern const int gear_ratio;

  void get_status(String value = "pos_rel");
  void get_position();
  void calibrate_drive(int odrive_number);
  void calibrate_drives();
  void set_drives_idle();
  void set_drives_closed_loop_control();
  void zero_drives();

#endif