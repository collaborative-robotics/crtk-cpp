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



#ifndef _TEST_FUNCS_H_
#define _TEST_FUNCS_H_

#include "defines.h"
#include "crtk_robot_state.h"
#include "crtk_robot.h"
#include <ctime>


int crtk_state_check(CRTK_robot_state_enum, CRTK_robot_state_enum, int);
int check_joint_motion_and_vel(CRTK_robot*, float, float, long, int);
int step_success(int, int*);
int done_sum(int*);
int check_movement_direction(CRTK_motion* , CRTK_axis , float , int, long);
float axis_value(tf::Vector3, CRTK_axis);
int check_movement_distance(CRTK_motion*,tf::Transform, CRTK_axis, float);
int check_movement_rotation(CRTK_motion*, float, int, long, tf::Transform);
char rand_cube_dir(char *, tf::Vector3 *, CRTK_axis *);
#endif