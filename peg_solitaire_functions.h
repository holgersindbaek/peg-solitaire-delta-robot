#ifndef PEG_SOLITAIRE_FUNCTIONS_H
#define PEG_SOLITAIRE_FUNCTIONS_H
#include <Arduino.h>
#include "kinematic_functions.h"
#include "suction_functions.h"

  extern ODriveArduino odrive_array[];
  extern bool robot_enabled;
  extern const float z_zero;
  extern const float board_top_offset;
  extern const float suction_bottom_offset;
  extern const float suction_grab_offset;

  void play_winning_peg_solitaire();
  void peg_coordinate(int row, int col, float &x, float &y);
  void drop_coordinate(int step, float &x, float &y);
#endif