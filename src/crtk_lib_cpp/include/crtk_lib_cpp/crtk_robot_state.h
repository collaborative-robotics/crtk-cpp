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
 * crtk_robot_state.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 18, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#ifndef CRTK_ROBOT_STATE_H_
#define CRTK_ROBOT_STATE_H_
#include "defines.h"
#include "ros/ros.h"
#include <crtk_msgs/operating_state.h>
#include <crtk_msgs/StringStamped.h>


class CRTK_robot_state
{
 public:

  // methods
  CRTK_robot_state();
  CRTK_robot_state(ros::NodeHandle n);

  ~CRTK_robot_state(){};


  bool set_homing();
  bool set_busy(bool new_state);
  // bool set_ready(bool new_state);
  bool set_homed(bool new_state);
  bool ready_logic();

  bool get_disabled();
  bool get_enabled();
  bool get_paused();
  bool get_fault();
  bool get_homing();
  bool get_busy();
  bool get_ready();
  bool get_homed();
  bool get_connected();

  CRTK_robot_state_enum get_state();

  bool set_disabled_state();
  bool set_enabled_state();
  bool set_paused_state();
  bool set_fault_state();
  bool set_connected(bool);

  bool init_ros(ros::NodeHandle);
  void operating_state_cb(crtk_msgs::operating_state msg);
  void crtk_command_pb(CRTK_robot_command);

  char state_char();
  std::string state_string();

  ros::Publisher pub;
  ros::Subscriber sub;

private:
  bool is_disabled;
  bool is_enabled;
  bool is_paused;
  bool is_fault;
  bool is_homing;
  bool is_busy;
  bool is_ready;
  bool is_homed;

  bool has_connected;

};

#endif /* CRTK_STATE_H_ */
