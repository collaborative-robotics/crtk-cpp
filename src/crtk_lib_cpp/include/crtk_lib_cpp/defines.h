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


#ifndef _DEFINES_H_
#define _DEFINES_H_

#include "tf/tf.h"
#include <cmath> 
#define MM_TO_M   * 0.001
#define LOOP_RATE 999        // Hz (TODO increase to 1000Hz? NOOOOOOO! will fail for _cp)


#define MAX_JOINTS 15 


#define DEG_TO_RAD * M_PI/180
#define RAD_TO_DEG * 180/M_PI

#define STEP_TRANS_LIMIT  2 MM_TO_M     // change this if loop rate is changed
#define STEP_ROT_LIMIT    3 DEG_TO_RAD  // change this if loop rate is changed

enum CRTK_axis {CRTK_X, CRTK_Y, CRTK_Z};
enum CRTK_input {CRTK_servo, CRTK_interp, CRTK_move, CRTK_out};
enum CRTK_robot_command {CRTK_ENABLE, CRTK_DISABLE, CRTK_PAUSE, CRTK_RESUME, CRTK_UNHOME, CRTK_HOME};
enum CRTK_robot_state_enum {CRTK_ENABLED, CRTK_DISABLED, CRTK_PAUSED, CRTK_FAULT};



#endif
