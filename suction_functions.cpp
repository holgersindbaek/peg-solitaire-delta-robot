#include "suction_functions.h"

void set_suction_state(int state) {
  if (state == 0) { // off
    solenoid_valve_servo.write(0);
    air_pump_servo.write(0);
  } else if (state == 1) { // pickup
    solenoid_valve_servo.write(180);
    air_pump_servo.write(0);
  } else if (state == 2) { // drop
    solenoid_valve_servo.write(0);
    air_pump_servo.write(180);
  }
}