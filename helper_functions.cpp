#include "helper_functions.h"

int count_occurrences(String str, char target) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == target) {
      count++;
    }
  }
  return count;
}

void extract_parameters(String input_string, double* params, int num_params) {
  int start = 0;
  int end = input_string.indexOf(',');
  int index = 0;
  while (index < num_params - 1 && end != -1) {
    params[index] = atof(input_string.substring(start, end).c_str()); // Extract the parameter and convert it to a double
    start = end + 1;
    end = input_string.indexOf(',', start);
    index++;
  }
  params[index] = atof(input_string.substring(start).c_str()); // Extract the last parameter
}

void EEPROM_set_double(int address, double value) {
  for(byte i = 0; i < sizeof(value); i++){
    EEPROM.write(address + i, reinterpret_cast<byte *>(&value)[i]);
  }
}

void EEPROM_get_double(int address, double &value) {
  for(byte i = 0; i < sizeof(value); i++){
    reinterpret_cast<byte*>(&value)[i]=EEPROM.read(address+i);
  }
}

String readSerialString(HardwareSerial serial) {
    String str = "";
    for (;;) {
        char c = serial.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}