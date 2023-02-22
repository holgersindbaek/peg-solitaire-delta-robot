#ifndef PEG_SOLITAIRE_FUNCTIONS_H
#define PEG_SOLITAIRE_FUNCTIONS_H
#include <Arduino.h>

  void play_winning_peg_solitaire();
  void play_random_peg_solitaire();
  bool findValidMove(int& from_row, int& from_col, int& to_row, int& to_col);
#endif