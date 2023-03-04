#include "peg_solitaire_functions.h"

// Board position constants
const float board_offset_degrees = 31.5; // Measured rotation of board
const float board_drop_radius = 110.0; // Measured radius of drop line
const int num_rows = 7;
const int num_cols = 7;
const float row_spacing = 27.5; // Distance between rows (mm)
const float col_spacing = 27.5; // Distance between columns (mm)
const float peg_height = 10.0;  // Height of pegs above board (mm)
const float safety_height = 10.0;
const float ball_height_suction = 15;
const float z_movement = z_zero + board_top_offset + suction_bottom_offset + peg_height + safety_height;
const float z_grabbing = z_zero + board_top_offset + suction_grab_offset + peg_height;
const float z_dropping = z_grabbing + 5;
const float z_ball_movement = z_movement + ball_height_suction;

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
  int winning_sequence[32][3][2] = {
    {{4, 6}, {4, 4}, {4, 5}},
    {{2, 5}, {4, 5}, {3, 5}},
    {{3, 7}, {3, 5}, {3, 6}},
    {{5, 7}, {3, 7}, {4, 7}},
    {{4, 5}, {2, 5}, {3, 5}},
    {{1, 5}, {3, 5}, {2, 5}},
    {{5, 5}, {5, 7}, {5, 6}},
    {{7, 5}, {5, 5}, {6, 5}},
    {{3, 4}, {3, 6}, {3, 5}},
    {{3, 7}, {3, 5}, {3, 6}},
    {{1, 4}, {3, 4}, {2, 4}},
    {{3, 4}, {3, 6}, {3, 5}},
    {{5, 4}, {3, 4}, {4, 4}},
    {{7, 4}, {5, 4}, {6, 4}},
    {{5, 4}, {5, 6}, {5, 5}},
    {{5, 7}, {5, 5}, {5, 6}},
    {{3, 3}, {3, 5}, {3, 4}},
    {{3, 6}, {3, 4}, {3, 5}},
    {{1, 3}, {3, 3}, {2, 3}},
    {{4, 3}, {2, 3}, {3, 3}},
    {{5, 2}, {5, 4}, {5, 3}},
    {{7, 3}, {5, 3}, {6, 3}},
    {{3, 1}, {3, 3}, {3, 2}},
    {{3, 4}, {3, 2}, {3, 3}},
    {{5, 1}, {3, 1}, {4, 1}},
    {{3, 1}, {3, 3}, {3, 2}},
    {{2, 3}, {4, 3}, {3, 3}},
    {{4, 3}, {6, 3}, {5, 3}},
    {{5, 5}, {5, 3}, {5, 4}},
    {{6, 3}, {4, 3}, {5, 3}},
    {{4, 2}, {4, 4}, {4, 3}}
  };

  // Move to starting position
  float start_thetas[3] = {odrive_array[0].GetPosition(0), odrive_array[1].GetPosition(0), odrive_array[2].GetPosition(0)};
  float start_points[3];
  calculate_motor_position(start_thetas, start_points);
  move_to_position(start_points[0], start_points[1], z_ball_movement);
  move_to_position(0, 0, z_ball_movement);

  // Run through winning sequence
  for (int step = 0; step < 32; step++) {
    Serial.println("step: " + String(step));
    
    // Move suction cup to peg, go down, grab peg and go up
    int from_row = winning_sequence[step][0][0];
    int from_col = winning_sequence[step][0][1];
    float from_x, from_y;
    peg_coordinate(from_row, from_col, from_x, from_y);
    move_to_position(from_x , from_y, z_movement);
    set_suction_state(1);
    move_to_position(from_x, from_y, z_grabbing);
    move_to_position(from_x, from_y, z_ball_movement);

    // Move ball from center to other hole, back and move suction cup up again
    int to_row = winning_sequence[step][1][0];
    int to_col = winning_sequence[step][1][1];
    float to_x, to_y;
    peg_coordinate(to_row, to_col, to_x, to_y);
    move_to_position(to_x, to_y, z_ball_movement);
    move_to_position(to_x, to_y, z_dropping);
    set_suction_state(2);
    move_to_position(to_x, to_y, z_movement);

    // Discard ball
    int remove_row = winning_sequence[step][2][0];
    int remove_col = winning_sequence[step][2][1];
    float remove_x, remove_y;
    peg_coordinate(remove_row, remove_col, remove_x, remove_y);
    move_to_position(remove_x, remove_y, z_movement);
    set_suction_state(1);
    move_to_position(remove_x, remove_y, z_grabbing);
    move_to_position(remove_x, remove_y, z_ball_movement);
    float drop_start_x, drop_start_y, drop_end_x, drop_end_y;
    drop_coordinate(step + 2, drop_start_x, drop_start_y);
    drop_coordinate(step, drop_end_x, drop_end_y);
    move_to_position(drop_start_x, drop_start_y, z_ball_movement);
    move_to_position(drop_end_x, drop_end_y, z_dropping);
    set_suction_state(2);
    move_to_position(drop_end_x, drop_end_y, z_movement);
  }

  // Go back to starting position
  move_to_position(4, 4, z_ball_movement);
}

void play_random_peg_solitaire() {
  Serial.println("Errors 1: " + String(odrive_array[0].GetActiveError(0)) + " || " + String(odrive_array[0].GetDisarmReason(0)));
  Serial.println("Errors 2: " + String(odrive_array[1].GetActiveError(0)) + " || " + String(odrive_array[1].GetDisarmReason(0)));
  Serial.println("Errors 3: " + String(odrive_array[2].GetActiveError(0)) + " || " + String(odrive_array[2].GetDisarmReason(0)));

  // int from_row = 4;
  // int from_col = 6;
  // float from_x, from_y;
  // peg_coordinate(from_row, from_col, from_x, from_y);
  // move_to_position(from_x, from_y, z_grabbing);

  // float discard_x, discard_y;
  // peg_coordinate(4, 1, discard_x, discard_y);
  // move_to_position(0, -110, z_ball_movement);
  // delay(5000);
  // move_to_position(0, -110, z_grabbing-8);
  // delay(5000);
  // move_to_position(row_spacing, -110, z_grabbing-8);
  // move_to_position(row_spacing, -111, z_movement);

  // float x, y;
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

void peg_coordinate(int row, int col, float &x, float &y) {
  // Return if position doens't exist on board
  if (row < 0 || row > num_rows || col < 0 || col > num_cols) {
    Serial.println("Wrong row or board position.");
    return;
  }

  // Calculate x and y coordinates
  int center_col_row = 4;
  float x_pos = (col - center_col_row) * col_spacing;
  float y_pos = (row - center_col_row) * row_spacing;

  // Rotate coordinates in relation to board rotation
  x = x_pos * cos(board_offset_degrees * PI / 180) - y_pos * sin(board_offset_degrees * PI / 180);
  y = y_pos * cos(board_offset_degrees * PI / 180) + x_pos * sin(board_offset_degrees * PI / 180);
}

void drop_coordinate(int step, float &x, float &y) {
  float degree_step = 360 / 36;
  float degrees = step * degree_step;

  // Calculate drop placement of ball
  float x_pos = -(board_drop_radius * cos(degrees * PI / 180));
  float y_pos = -(board_drop_radius * sin(degrees * PI / 180));

  Serial.println(x_pos);
  Serial.println(y_pos);

  // Rotate coordinates in relation to board rotation
  x = x_pos * cos(board_offset_degrees * PI / 180) - y_pos * sin(board_offset_degrees * PI / 180);
  y = y_pos * cos(board_offset_degrees * PI / 180) + x_pos * sin(board_offset_degrees * PI / 180);
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