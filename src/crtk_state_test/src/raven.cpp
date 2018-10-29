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

#include <ros/ros.h>
#include <raven.h>

raven::raven(){
  runlevel      = '?';
  surgeon_mode  = '?';
  robot_fault   = '?';
  robot_homed   = '?';
}

bool raven::init_ros(ros::NodeHandle n){
  sub = n.subscribe("ravenstate", 1, &raven::raven_state_cb,this);
  return true;
}

char raven::get_runlevel(){
	return runlevel;
}

char raven::get_surgeon_mode(){
	return surgeon_mode;     
}

char raven::get_robot_fault(){
  return robot_fault;
}

char raven::get_robot_homed(){
  return robot_homed;
}

void raven::raven_state_cb(raven_2::raven_state msg){
  static int count = 0;

  runlevel = msg.runlevel;
  if (runlevel == RL_PEDAL_DN)
    surgeon_mode = 1;
  else if (runlevel == RL_PEDAL_UP)
    surgeon_mode == 0;

  robot_homed = msg.sublevel;

  // TODO
  // get robot fault
  if(count%100 == 0){
    // ROS_INFO("runlevel = %i",runlevel);
    // ROS_INFO("surgeon mode = %i",surgeon_mode);
  }
  ++count;
}