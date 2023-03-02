#ifndef SUCTION_FUNCTIONS_H
#define SUCTION_FUNCTIONS_H
#include <Arduino.h>
#include <Servo.h>

  extern Servo solenoid_valve_servo;
  extern Servo air_pump_servo;

  void set_suction_state(int state);

#endif