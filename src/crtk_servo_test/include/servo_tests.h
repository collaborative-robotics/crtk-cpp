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
int test_2(CRTK_robot *, time_t);

// // II.    {disabled, homed} + enable [prompt for button press] → {enabled / p_dn}
// // IV-2.  {enabled, busy} + pause → {paused / p_up}
// int test_2(Raven, CRTK_robot_state, time_t);

// // VI-2.    {paused, p_up} + disable → {disabled / e-stop}
// // VIII-2.    {disabled, homed} + unhome → {disabled, ~homed / e-stop} 
// int test_3(Raven, CRTK_robot_state, time_t);

// // IV-1.    {enabled, homing} + pause → {disabled / e-stop}
// // VIII-1.  {disabled, ~homed} + unhome → {disabled, ~homed / e-stop}
// int test_4(Raven, CRTK_robot_state, time_t);

// // III-1.    {enabled, homing} + disable → {disabled / e-stop}
// // III-2.    {enabled, busy} + disable → {disabled / e-stop}
// int test_5(Raven, CRTK_robot_state, time_t);

// // VIII-3.    {enabled, homing} + unhome → {disabled, ~homed / e-stop}
// // VIII-4.    {enabled, busy} + unhome → {disabled, ~homed / e-stop}
// int test_6(Raven, CRTK_robot_state, time_t);

// // VIII-6.    {paused, homed} + unhome → {disabled, ~homed / e-stop}
// int test_7(Raven, CRTK_robot_state, time_t);

// // V-3.    {disabled, ~homed} + home [prompt for button press] → {enabled, homing / init}
// // V-1.    {enabled, homed} + home [prompt for button press] → {enabled, homing / init}
// // V-2.    {paused, homed} + home [prompt for button press] → {enabled, homing / init}
// int test_8(Raven, CRTK_robot_state, time_t);     
#endif