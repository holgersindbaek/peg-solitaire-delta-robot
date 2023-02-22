#include "peg_solitaire_functions.h"
#include "drive_functions.h"

// Board position constants
const int num_rows = 7;
const int num_cols = 7;
const double row_spacing = 28.0;  // Distance between rows (mm)
const double col_spacing = 28.0;  // Distance between columns (mm)
const double peg_height = 50.0;   // Height of pegs above board (mm)
const double hole_depth = -10.0;  // Depth of holes below board (mm)

// Peg Solitaire game state
bool board[num_rows][num_cols] = {
  {false, false, true, true, true, false, false},
  {false, false, true, true, true, false, false},
  {true, true, true, true, true, true, true},
  {true, true, true, false, true, true, true},
  {true, true, true, true, true, true, true},
  {false, false, true, true, true, false, false},
  {false, false, true, true, true, false, false}
};
int num_pegs = 32;

void play_winning_peg_solitaire() {

}

void play_random_peg_solitaire() {
  while (num_pegs > 1) {
    // Find a valid move
    int from_row, from_col, to_row, to_col;
    if (!findValidMove(from_row, from_col, to_row, to_col)) {
      // No valid move found, game over
      break;
    }
    
    // // Move the peg to the new location
    // move_to_position(from_row, from_col, peg_height);
    // delay(500);
    // move_to_position(from_row, from_col, hole_depth);
    // delay(500);
    // board[from_row][from_col] = false;
    // num_pegs--;
    // move_to_position(to_row, to_col, hole_depth);
    // delay(500);
    // move_to_position(to_row, to_col, peg_height);
    // delay(500);
    // board[to_row][to_col] = true;
    // num_pegs++;
    // move_to_position(to_row, to_col, hole_depth);
    // delay(500);
  }
}

// Find a valid move on the Peg Solitaire board
bool findValidMove(int& from_row, int& from_col, int& to_row, int& to_col) {
  // Check all possible moves
  for (int row = 0; row < num_rows; row++) {
    for (int col = 0; col < num_cols; col++) {
      if (board[row][col]) {
        // Check up
        if (row >= 2 && board[row-1][col] && board[row-2][col]) {
          from_row = row;
          from_col = col;
          to_row = row-2;
          to_col = col;
          return true;
        }

        // Check down
        if (row <= num_rows-3 && board[row+1][col] && board[row+2][col]) {
          from_row = row;
          from_col = col;
          to_row = row+2;
          to_col = col;
          return true;
        }

        // Check left
        if (col >= 2 && board[row][col-1] && board[row][col-2]) {
          from_row = row;
          from_col = col;
          to_row = row;
          to_col = col-2;
          return true;
        }

        // Check right
        if (col <= num_cols-3 && board[row][col+1] && board[row][col+2]) {
          from_row = row;
          from_col = col;
          to_row = row;
          to_col = col+2;
          return true;
        }
      }
    }
  }

  // No valid move found
  return false;
}