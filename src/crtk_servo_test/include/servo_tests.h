/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2018  Andrew Lewis, Yun-Hsuan Su, Blake Hannaford, 
 * and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * 
 *
 *
 *  \date Oct 29, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#include "defines.h"
#include "crtk_robot_state.h"
#include "crtk_robot.h"
// #include "raven.h"

#ifndef _SERVO_TESTS_
#define _SERVO_TESTS_



int servo_testing(CRTK_robot*, time_t);

// 1 Motion measured query testing
//    1-1 (measured_js functionality) Move all joints in series manually.
//      Pass: Check that each joint has moved.
//      Pass: Check that each joint velocity has a non-zero value.
//    1-2 (measured_cp functionality) Move tool to the right manually.
//      Pass: Check that the correct axis has been mostly moved in.
int test_1(CRTK_robot *, time_t);


// 2-1 Relative (command: servo_cr) Axis motion Test
// (functionality) move along X axis for 2 secs (both arms)
// 		Pass: Check raven state
// (functionality) move along Y axis for 2 secs (both arms)
// 		Pass: Check raven state
// (functionality) move along Z axis for 2 secs (both arms)
// 		Pass: Check raven state
int test_2_1(CRTK_robot *, time_t);


// 2-2 Relative (command: servo_cr) Cube tracing Test
// (functionality) Trace a cube
//    Pass: Ask user!
int test_2_2(CRTK_robot *, time_t);
 
// 2-3 Relative (command: servo_cr) Orientation axis test
// (functionality) rotate about X,Y,Z axis for 1 secs (30 deg)
// Pass: ask user!
int test_2_3(CRTK_robot *, time_t);

// 2-4 Relative (command: servo_cr for grasper) Grasper test
// (functionality) clapping with grasper for 2 sec (max = 30 deg)
// Pass: ask user!
int test_2_4(CRTK_robot *, time_t);

// 3-1 Absolute (command: servo_cp) Axis motion Test
// (functionality) move along X axis for 2 cm (both arms)
// 		Pass: Ask user
// (functionality) move along Z axis for 2 cm (both arms)
// 		Pass: Ask user
int test_3_1(CRTK_robot *, time_t);

// 3-2 Absolute (command: servo_cp) Axis rotation Test
// (functionality) rotate along X axis for 45 degrees (both arms)
//    Pass: Ask user
// (functionality) rotate along Z axis for 45 degrees (both arms)
//    Pass: Ask user
int test_3_2(CRTK_robot *, time_t);


// 3-3 Go home (command: servo_cp) 
// (functionality) move back to home pose (both arms)
//    Pass: Ask user
int test_3_3(CRTK_robot *, time_t);


// 4-1 Relative joint test (command: servo_jr) 
// (functionality) move 10 degrees in the shoulder and tool joints
//    Pass: Ask user
int test_4_1(CRTK_robot *, time_t);


// 4-2 Go home (command: servo_jr) 
// (functionality) move back to home pose (both arms)
//    Pass: Ask user
int test_4_2(CRTK_robot *, time_t);

// 5-1 Absolute joint test (command: servo_jp) 
// (functionality) move 10 degrees in the shoulder and tool joints
//    Pass: Ask user
int test_5_1(CRTK_robot *, time_t);


// 5-2 Go home (command: servo_jp) 
// (functionality) move back to home pose (both arms)
//    Pass: Ask user
int test_5_2(CRTK_robot *, time_t);
#endif