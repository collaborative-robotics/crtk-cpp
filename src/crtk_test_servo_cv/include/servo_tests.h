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
 *  \date Aug 14, 2019
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#include <crtk_lib_cpp/defines.h>
#include <crtk_lib_cpp/crtk_robot_state.h>
#include <crtk_lib_cpp/crtk_robot.h>


#ifndef _SERVO_TESTS_
#define _SERVO_TESTS_


// This function loops through all the crtk tests
int servo_testing(CRTK_robot*, time_t);


// 7-1 Relative (command: servo_cv) Axis motion Test
// (functionality) move along X axis for 2 secs (both arms)
// 		Pass: Check raven state
// (functionality) move along Y axis for 2 secs (both arms)
// 		Pass: Check raven state
// (functionality) move along Z axis for 2 secs (both arms)
// 		Pass: Check raven state
int test_7_1(CRTK_robot *, time_t);


// 7-2 Relative (command: servo_cv) Cube tracing Test
// (functionality) Trace a cube
//    Pass: Ask user!
int test_7_2(CRTK_robot *, time_t);
 
// 7-3 Relative (command: servo_cv) Orientation axis test
// (functionality) rotate about X,Y,Z axis for 1 secs (30 deg)
// Pass: ask user!
int test_7_3(CRTK_robot *, time_t);

#endif