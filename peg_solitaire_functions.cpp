#include "peg_solitaire_functions.h"

// Board position constants
const float board_offset_degrees = 31.5; // Measured rotation of board
const float board_drop_radius = 110.0; // Measured radius of drop line
const int num_rows = 7;
const int num_cols = 7;
const float row_spacing = 27.5; // Distance between rows (mm)
const float col_spacing = 27.5; // Distance between columns (mm)
const float peg_height = 10.0;  // Height of pegs above board (mm)
const float safety_height = 30.0;
const float ball_height_suction = 15;
const float z_movement = z_zero + board_top_offset + suction_bottom_offset + peg_height + safety_height;
const float z_grabbing = z_zero + board_top_offset + suction_grab_offset + peg_height;
const float z_dropping = z_grabbing + 5;
const float z_ball_movement = z_movement + ball_height_suction;
const float z_discard = z_grabbing + 28;

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
  move_to_position(0, 0, z_ball_movement);
  delay(3000);

  // Run through winning sequence
  for (int step = 0; step < 31; step++) {
    Serial.println("step: " + String(step));

    // Move suction cup to peg, go down, grab peg and go up
    int from_row = winning_sequence[step][0][0];
    int from_col = winning_sequence[step][0][1];
    float from_x, from_y;
    peg_coordinate(from_row, from_col, from_x, from_y);
    move_to_position(from_x , from_y, z_movement);
    set_suction_state(1);
    move_to_position(from_x, from_y, z_grabbing, 0.1);
    delay(250);
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
    move_to_position(remove_x, remove_y, z_grabbing, 0.1);
    move_to_position(remove_x, remove_y, z_ball_movement);
    float drop_start_x, drop_start_y, drop_end_x, drop_end_y;
    drop_coordinate(step + 3, drop_start_x, drop_start_y);
    drop_coordinate(step, drop_end_x, drop_end_y);
    move_to_position(drop_start_x, drop_start_y, z_ball_movement);
    move_to_position(drop_start_x, drop_start_y, z_discard);
    move_to_position(drop_end_x, drop_end_y, z_dropping, 0.1);
    set_suction_state(2);
    move_to_position(drop_end_x, drop_end_y, z_ball_movement);
  }

  // Go back to starting position
  delay(1000);
  move_to_position(4, 4, z_ball_movement);
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

  // Rotate coordinates in relation to board rotation
  x = x_pos * cos(board_offset_degrees * PI / 180) - y_pos * sin(board_offset_degrees * PI / 180);
  y = y_pos * cos(board_offset_degrees * PI / 180) + x_pos * sin(board_offset_degrees * PI / 180);
}