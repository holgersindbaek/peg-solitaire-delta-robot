#include "peg_solitaire_functions.h"

// Board position constants
const int num_rows = 7;
const int num_cols = 7;
const double row_spacing = 27.5;  // Distance between rows (mm)
const double col_spacing = 27.5;  // Distance between columns (mm)
const double peg_height = 14.0;   // Height of pegs above board (mm)
const double hole_depth = -7.0;  // Depth of holes below board (mm)
const double safety_height = 10.0;
const double ball_height = 20.5;
const double z_movement = z_zero + board_top_offset + suction_bottom_offset + peg_height + safety_height;
const double z_grabbing = z_zero + board_top_offset + suction_grab_offset + peg_height;
const double z_ball_movement = z_movement + ball_height;

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
  // Winning sequence for peg solitaire - starting row-col to ending row-col
  int winning_sequence[32][2][2] = {
      {{4, 6}, {4, 4}},
      {{2, 5}, {4, 5}},
      {{3, 7}, {3, 5}},
      {{5, 7}, {3, 7}},
      {{4, 4}, {4, 2}},
      {{1, 5}, {3, 5}},
      {{5, 5}, {5, 7}},
      {{7, 5}, {5, 5}},
      {{3, 4}, {3, 6}},
      {{3, 7}, {3, 5}},
      {{1, 4}, {3, 4}},
      {{3, 4}, {3, 6}},
      {{5, 4}, {3, 4}},
      {{7, 4}, {5, 4}},
      {{4, 6}, {4, 4}},
      {{5, 4}, {5, 6}},
      {{5, 7}, {5, 5}},
      {{3, 3}, {3, 5}},
      {{3, 6}, {3, 4}},
      {{1, 3}, {3, 3}},
      {{4, 3}, {2, 3}},
      {{5, 2}, {5, 4}},
      {{7, 3}, {5, 3}},
      {{3, 1}, {3, 3}},
      {{3, 4}, {3, 2}},
      {{5, 1}, {3, 1}},
      {{3, 1}, {3, 3}},
      {{2, 3}, {4, 3}},
      {{4, 3}, {6, 3}},
      {{3, 5}, {3, 3}},
      {{6, 3}, {4, 3}},
      {{4, 2}, {4, 4}},
  };

  // Move to starting position
  move_to_position(0, 0, z_movement);
  delay(1000);

  for (int step = 0; step < 32; step++) {
    Serial.println("step: " + String(step));
    int from_row = winning_sequence[step][0][0];
    int from_col = winning_sequence[step][0][1];
    int to_row = winning_sequence[step][1][0];
    int to_col = winning_sequence[step][1][1];

    // Move suction cup to peg, go down, grab peg and go up
    double from_x, from_y;
    peg_coordinate(from_row, from_col, from_x, from_y);
    move_to_position(from_x, from_y, z_movement);
    // delay(1000);
    move_to_position(from_x, from_y, z_grabbing);
    set_suction_state(1);
    // delay(1000);
    move_to_position(from_x, from_y, z_ball_movement);
    // delay(1000);

    // Move ball from center to other hole, back and move suction cup up again
    double to_x, to_y;
    peg_coordinate(to_row, to_col, to_x, to_y);
    move_to_position(to_x, to_y, z_ball_movement);
    // delay(1000);
    Serial.println("substep 5");
    move_to_position(to_x, to_y, z_grabbing);
    set_suction_state(2);
    // delay(1000);
    Serial.println("substep 6");
    move_to_position(to_x, to_y, z_movement);
    // delay(1000);
    Serial.println("substep 7");
  }
}

void play_random_peg_solitaire() {
  move_to_position(1, 1, z_grabbing);
  // double x, y;
  // move_to_position(0, 0, y_grabbing);
  // delay(1000);
  // peg_coordinate(0, 3, x, y);
  // move_to_position(x, y, y_grabbing - 10);
  // delay(5000);
  // move_to_position(0, 0, y_grabbing);
  // delay(1000);
  // peg_coordinate(6, 3, x, y);
  // move_to_position(x, y, y_grabbing - 10);
  // delay(5000);
  // move_to_position(0, 0, y_grabbing);
  // delay(1000);
  // peg_coordinate(3, 0, x, y);
  // move_to_position(x, y, y_grabbing - 10);
  // delay(5000);
  // move_to_position(0, 0, y_grabbing);
  // delay(1000);
  // peg_coordinate(3, 6, x, y);
  // move_to_position(x, y, y_grabbing - 10);
  // delay(5000);
  // move_to_position(0, 0, y_grabbing);
  // while (num_pegs > 1) {
  //   // Find a valid move
  //   int from_row, from_col, to_row, to_col;
  //   if (!findValidMove(from_row, from_col, to_row, to_col)) {
  //     // No valid move found, game over
  //     break;
  //   }
    
  //   // // Move the peg to the new location
  //   // move_to_position(from_row, from_col, peg_height);
  //   // delay(500);
  //   // move_to_position(from_row, from_col, hole_depth);
  //   // delay(500);
  //   // board[from_row][from_col] = false;
  //   // num_pegs--;
  //   // move_to_position(to_row, to_col, hole_depth);
  //   // delay(500);
  //   // move_to_position(to_row, to_col, peg_height);
  //   // delay(500);
  //   // board[to_row][to_col] = true;
  //   // num_pegs++;
  //   // move_to_position(to_row, to_col, hole_depth);
  //   // delay(500);
  // }
}

void peg_coordinate(int row, int col, double &x, double &y) {
  // Return if position doens't exist on board
  if (row < 0 || row >= num_rows || col < 0 || col >= num_cols) {
    Serial.println("Wrong row or board position.");
    return;
  }

  // Adjust offset because of inaccurate board zero
  // x = x + 1;

  // Calculate x and y coordinates
  // float x_pos = ((col - 1) - (num_cols - 1) / 2.0) * col_spacing;
  // float y_pos = ((row - 1) - (num_rows - 1) / 2.0) * row_spacing;
  int center_col_row = 4;
  float x_pos = (col - center_col_row) * col_spacing;
  float y_pos = (row - center_col_row) * row_spacing;
  Serial.println("col: " + String(col));
  Serial.println("row: " + String(row));
  Serial.println("x_pos: " + String(x_pos));
  Serial.println("y_pos: " + String(y_pos));

  // Rotate coordinates in relation to board rotation
  float degrees = 58.5; // Measured rotation of board
  x = x_pos * cos(degrees * PI / 180) + y_pos * sin(degrees * PI / 180);
  y = y_pos * cos(degrees * PI / 180) - x_pos * sin(degrees * PI / 180);
  // Serial.println("x: " + String(x));
  // Serial.println("y: " + String(y));
}

// // Find a valid move on the Peg Solitaire board
// bool findValidMove(int& from_row, int& from_col, int& to_row, int& to_col) {
//   // Check all possible moves
//   for (int row = 0; row < num_rows; row++) {
//     for (int col = 0; col < num_cols; col++) {
//       if (board[row][col]) {
//         // Check up
//         if (row >= 2 && board[row-1][col] && board[row-2][col]) {
//           from_row = row;
//           from_col = col;
//           to_row = row-2;
//           to_col = col;
//           return true;
//         }

//         // Check down
//         if (row <= num_rows-3 && board[row+1][col] && board[row+2][col]) {
//           from_row = row;
//           from_col = col;
//           to_row = row+2;
//           to_col = col;
//           return true;
//         }

//         // Check left
//         if (col >= 2 && board[row][col-1] && board[row][col-2]) {
//           from_row = row;
//           from_col = col;
//           to_row = row;
//           to_col = col-2;
//           return true;
//         }

//         // Check right
//         if (col <= num_cols-3 && board[row][col+1] && board[row][col+2]) {
//           from_row = row;
//           from_col = col;
//           to_row = row;
//           to_col = col+2;
//           return true;
//         }
//       }
//     }
//   }

//   // No valid move found
//   return false;
// }