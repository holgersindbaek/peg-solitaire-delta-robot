#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H
#include <Arduino.h>
#include <EEPROM.h>

  int count_occurrences(String str, char target);
  void extract_parameters(String input_string, double* params, int num_params);
  void EEPROM_set_double(int address, double value);
  void EEPROM_get_double(int address, double& value);
  String readSerialString(HardwareSerial serial);

#endif