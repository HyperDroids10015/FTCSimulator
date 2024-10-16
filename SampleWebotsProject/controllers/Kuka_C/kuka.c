/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Start with a predefined behavior and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/keyboard.h>
#include <webots/joystick.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void high_level_go_to(double x, double y, double a) {
  base_goto_set_target(x, y, a);
  while (!base_goto_reached()) {
    base_goto_run();
    step();
  }
  base_reset();
}

static void high_level_grip_pixel(bool grip) {
  static double h_per_step = 0.002;
  static double offset = 0.01;  // security margin

  double x = 0.0;
  double z = 0.0127;
  double y = 0.189;

  if (!grip)
    z += offset;

  // prepare
  arm_set_sub_arm_rotation(ARM5, M_PI_2);
  arm_ik(x, y, 0.20);
  if (grip)
    gripper_release();
  passive_wait(1.0);

  // move the arm down
  double h;
  for (h = 0.2; h > z; h -= h_per_step) {
    arm_ik(x, y, h);
    step();
  }

  passive_wait(1.0);

  // grip or ungrip
  if (grip)
    gripper_set_gap(0.012265);
  else
    gripper_release();
  passive_wait(1.0);
/*
  // move the arm up
  for (h = z; h < 0.2; h += h_per_step) {
    arm_ik(x, y, h);
    step();
  }
  arm_set_orientation(ARM_FRONT);
*/
}

static void high_level_stock(int o, bool stock) {
  arm_set_height(ARM_BACK_PLATE_HIGH);
  arm_set_orientation(o);
  passive_wait(4.5);
  if (stock)
    gripper_release();
  else
    gripper_set_gap(0.04);
  passive_wait(1.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(3.0);
}

static void automatic_behavior() {
  int GOTO_SPM = 0;

  double goto_info[1][3] = {{0.0, 1.2, 0.0}};

  high_level_go_to(goto_info[GOTO_SPM][0], goto_info[GOTO_SPM][1], goto_info[GOTO_SPM][2]);
  high_level_grip_pixel(true);
//  high_level_stock(ARM_FRONT, true);
  // end behavior
  arm_reset();
  high_level_go_to(0.0, 0.0, 0.0);

  /*
  arm_set_height(ARM_BACK_PLATE_LOW);
  passive_wait(3.0);
  gripper_release();
  passive_wait(1.0);
  arm_reset();
  base_strafe_right();
  passive_wait(5.0);
  gripper_grip();
  base_reset();
  passive_wait(1.0);
  base_turn_right();
  passive_wait(1.0);
  base_reset();
  gripper_release();
  arm_set_height(ARM_BACK_PLATE_LOW);
  passive_wait(3.0);
  gripper_grip();
  passive_wait(1.0);
  arm_set_height(ARM_RESET);
  passive_wait(2.0);
  arm_set_height(ARM_FRONT_PLATE);
  arm_set_orientation(ARM_RIGHT);
  passive_wait(4.0);
  arm_set_height(ARM_FRONT_FLOOR);
  passive_wait(2.0);
  gripper_release();
  passive_wait(1.0);
  arm_set_height(ARM_FRONT_PLATE);
  passive_wait(2.0);
  arm_set_height(ARM_RESET);
  passive_wait(2.0);
  arm_reset();
  gripper_grip();
  passive_wait(2.0);
*/
}

static void display_helper_message() {
  printf("\n \nKeyboard commands:\n");
  printf(" Arrows:         Move the robot\n");
  printf(" Page Up/Down:   Rotate the robot\n");
  printf(" +/-:            (Un)grip\n");
  printf(" Shift + arrows: Handle the arm\n");
  printf(" Space:          Reset\n");
  if (wb_joystick_is_connected()) {
    printf("\n\nFound joystick: %s\n",wb_joystick_get_model());
    int n_axes = wb_joystick_get_number_of_axes();
    int n_povs = wb_joystick_get_number_of_povs();
    printf("Number of axes: %d\n",n_axes);
    printf("Number of point of views (POV): %d\n",n_povs);
    while(false) {
      int i;
      printf("Axes:");
      for (i=0; i<n_axes; i++) printf(" %d",wb_joystick_get_axis_value(i));
      printf("POVs:");
      for (i=0; i<n_povs; i++) printf(" %d",wb_joystick_get_pov_value(i));
      printf("Buttons:");
      i = wb_joystick_get_pressed_button();
      while (i>=0) {
        printf(" %d",i);
        i = wb_joystick_get_pressed_button();
      }
      printf("\n");
      passive_wait(0.1);
    }

  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();
  WbDeviceTag kinect_color = wb_robot_get_device("kinect color");
  WbDeviceTag kinect_range = wb_robot_get_device("kinect range");
  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);
  passive_wait(2.0);

  if (argc > 1 && strcmp(argv[1], "AUTO") == 0)
    automatic_behavior();

  wb_keyboard_enable(TIME_STEP);
  wb_joystick_enable(TIME_STEP);
  passive_wait(2.0);

  display_helper_message();

  int pc = 0;

  while (true) {
    step();

    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          base_forwards_increment();
          break;
        case WB_KEYBOARD_DOWN:
          base_backwards_increment();
          break;
        case WB_KEYBOARD_LEFT:
          base_strafe_left_increment();
          break;
        case WB_KEYBOARD_RIGHT:
          base_strafe_right_increment();
          break;
        case WB_KEYBOARD_PAGEUP:
          base_turn_left_increment();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          base_turn_right_increment();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case '+':
        case 388:
        case 65585:
        case 65579:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
