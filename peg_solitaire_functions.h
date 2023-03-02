#ifndef PEG_SOLITAIRE_FUNCTIONS_H
#define PEG_SOLITAIRE_FUNCTIONS_H
#include <Arduino.h>
#include "kinematic_functions.h"
#include "suction_functions.h"

  extern const float z_zero;
  extern const float board_top_offset;
  extern const float suction_bottom_offset;
  extern const float suction_grab_offset;

  void play_winning_peg_solitaire();
  void play_random_peg_solitaire();
  void peg_coordinate(int row, int col, double &x, double &y);
  bool findValidMove(int& from_row, int& from_col, int& to_row, int& to_col);
#endif