#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H
#include <Arduino.h>
#include <EEPROM.h>

  int count_occurrences(String str, char target);
  void extract_parameters(String input_string, float* params, int num_params);
  void EEPROM_set_float(int address, float value);
  void EEPROM_get_float(int address, float& value);
  String readSerialString(HardwareSerial serial);

#endif