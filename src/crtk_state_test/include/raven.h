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
 * crtk_state.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 18, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#ifndef _RAVEN_H_
#define _RAVEN_H_

#define RL_E_STOP 0
#define RL_INIT 1
#define RL_PEDAL_UP 2
#define RL_PEDAL_DN 3

#define SM_PEDAL_UP 0
#define SM_PEDAL_DN 1

#include <raven_2/raven_state.h>


class raven{
  private:
  	char runlevel;
  	char surgeon_mode;
  	char robot_fault;
  	char robot_homed;
  	// TODO: to get robot_fault and robot_homed from raven_state

  public:
  ros::Subscriber sub;
  raven();
  bool init_ros(ros::NodeHandle);
  char get_runlevel();
  char get_surgeon_mode();
  char get_robot_fault();
  char get_robot_homed();

  void raven_state_cb(raven_2::raven_state);
};

#endif