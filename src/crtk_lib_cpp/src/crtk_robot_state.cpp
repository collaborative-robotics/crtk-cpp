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

#include "ros/ros.h"
#include "crtk_robot_state.h"

#include <sstream>

CRTK_robot_state::CRTK_robot_state(){
  is_disabled   = 0;
  is_enabled    = 0;
  is_paused     = 0;
  is_fault      = 0;
  is_homing     = 0;
  is_busy       = 0;
  is_ready      = 0;
  is_homed      = 0;

  has_connected = 0;
}

CRTK_robot_state::CRTK_robot_state(ros::NodeHandle n){
  is_disabled   = 0;
  is_enabled    = 0;
  is_paused     = 0;
  is_fault      = 0;
  is_homing     = 0;
  is_busy       = 0;
  is_ready      = 0;
  is_homed      = 0;

  has_connected = 0;
  init_ros(n);
}


/**
 * @brief      initializes ros pubs and subs
 *
 * @param[in]  n     ros nodehandle
 *
 * @return     success
 */
bool CRTK_robot_state::init_ros(ros::NodeHandle n){

  string topic;

  // Read ROS Parameter Values from yaml file
  if(!n.getParam("/robot_namespace", robot_name))
    ROS_ERROR("Cannot read robot_namespace from the param.yaml file.");

  // Set Publisher and Subscribers under the namespace from from the parameter list
  topic = robot_name + "state_command";
  pub = n.advertise<crtk_msgs::StringStamped>(topic, 1);

  topic = robot_name + "operating_state";
  sub = n.subscribe(topic, 1, &CRTK_robot_state::operating_state_cb,this);

  return true;
}


/**
 * @brief      updates local copy of robot's state based on the messages from the robot
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_robot_state::operating_state_cb(crtk_msgs::operating_state msg){

  std::string state = msg.state;

  if (state =="DISABLED"){
    set_disabled_state();
  }
  else if (state =="ENABLED"){
    set_enabled_state();
  }

  else if (state =="PAUSED"){
    set_paused_state();
  }
  else if (state =="FAULT"){
    set_fault_state();
  }
  else{
    set_fault_state();
  }


  set_homed(msg.is_homed);
  set_busy(msg.is_busy);
  set_homing();
  // set_ready(msg.is_ready);
  ready_logic();

  set_connected(1);

  static int count = 0;
  ++count;

}



/**
 * @brief      send crtk robot state transition command
 *
 * @param[in]  command  The command
 */
void CRTK_robot_state::crtk_command_pb(CRTK_robot_command command){

  static int count = 0;
  static crtk_msgs::StringStamped msg_command;

  //robot command supports ("ENABLE", "DISABLE", "PAUSE", "RESUME", "NULL")
  switch(command)
  {
    case CRTK_ENABLE:
      msg_command.string = "enable";
      ROS_INFO("Sent ENABLE: May need to press start button.");
      break;

    case CRTK_DISABLE:
      msg_command.string = "disable";
      ROS_INFO("Sent DISABLE.");
      break;

    case CRTK_PAUSE:
      msg_command.string = "pause";
      ROS_INFO("Sent PAUSE.");
      break;

    case CRTK_RESUME:
      msg_command.string = "resume";
      ROS_INFO("Sent RESUME.");
      break;

    case CRTK_UNHOME:
      msg_command.string = "unhome";
      ROS_INFO("Sent UNHOME.");
      break;

    case CRTK_HOME:
      msg_command.string = "home";
      ROS_INFO("Sent HOME: May need to press start button."); 
      break;

    default:
      msg_command.string = "NULL";
      ROS_INFO("Sent NULL.");
      break;
  }
  msg_command.header.stamp = msg_command.header.stamp.now();
  pub.publish(msg_command);
  ++count;

}



/**
 * @brief      Gets the robot state.
 *
 * @return     The state.
 */
CRTK_robot_state_enum CRTK_robot_state::get_state(){
  if(is_disabled){
    return CRTK_DISABLED;
  }
  else if(is_enabled){
    return CRTK_ENABLED;
  }
  else if(is_paused){
    return CRTK_PAUSED;
  }
  else{
    return CRTK_FAULT;
  }
}



/**
 * @brief      Sets robot to is_homing.
 *
 * @return     is_homing flag
 */
bool CRTK_robot_state::set_homing(){
  is_homing = is_busy && !is_homed;
  return is_homing;
}



/**
 * @brief      Sets robot is_busy flag.
 *
 * @param[in]  new_state  The value
 *
 * @return     is_busy flag
 */
bool CRTK_robot_state::set_busy(bool new_state){
  is_busy = new_state;
  return is_busy;
}



/**
 * @brief      Sets robot is_homed flag.
 *
 * @param[in]  new_state  The value
 *
 * @return     is_homed flag
 */
bool CRTK_robot_state::set_homed(bool new_state){
  is_homed = new_state;
  return is_homed;
}



/**
 * @brief      Decide robot ready status based on other robot status flags
 *
 * @return     is_ready flag
 */
bool CRTK_robot_state::ready_logic(){
  is_ready = !is_busy && is_homed && is_enabled;
  return is_ready;
}



/**
 * @brief      Sets the robot to disabled state.
 *
 * @return     0
 */
bool CRTK_robot_state::set_disabled_state(){
  is_disabled   = 1;
  is_enabled    = 0;
  is_paused     = 0;
  is_fault      = 0;

  return 0;
}



/**
 * @brief      Sets the robot to enabled state.
 *
 * @return     0
 */
bool CRTK_robot_state::set_enabled_state(){
  is_disabled   = 0;
  is_enabled    = 1;
  is_paused     = 0;
  is_fault      = 0;

  return 0;
}



/**
 * @brief      Sets the robot to paused state.
 *
 * @return     0
 */
bool CRTK_robot_state::set_paused_state(){
  is_disabled   = 0;
  is_enabled    = 0;
  is_paused     = 1;
  is_fault      = 0;

  return 0;
}



/**
 * @brief      Sets the robot to fault state.
 *
 * @return     0
 */
bool CRTK_robot_state::set_fault_state(){
  is_disabled   = 0;
  is_enabled    = 0;
  is_paused     = 0;
  is_fault      = 1;

  return 0;
}



/**
 * @brief      Sets the has_connected flag for the robot
 *
 * @param[in]  val   The value
 *
 * @return     0
 */
bool CRTK_robot_state::set_connected(bool val){
  if(val == 0 || val == 1)
    has_connected = val;
}



/**
 * @brief      Maps the robot state to single char
 *
 * @return     char version of the robot state
 */
char CRTK_robot_state::state_char(){
  char out;

  if(is_disabled)       out = 'D';
  else if (is_enabled)  out = 'E';
  else if (is_paused)   out = 'P';
  else if (is_fault)    out = 'F';
  else out = 'N'; //no connection?

  return out;
}



/**
 * @brief      Gets the disabled flag.
 *
 * @return     The disabled flag.
 */
bool CRTK_robot_state::get_disabled(){
  return is_disabled;
}


/**
 * @brief      Gets the enabled flag.
 *
 * @return     The enabled flag.
 */
bool CRTK_robot_state::get_enabled(){
  return is_enabled;
}


/**
 * @brief      Gets the paused flag.
 *
 * @return     The paused flag.
 */
bool CRTK_robot_state::get_paused(){
  return is_paused;
}



/**
 * @brief      Gets the fault flag.
 *
 * @return     The fault flag.
 */
bool CRTK_robot_state::get_fault(){
  return is_fault;
}



/**
 * @brief      Gets the homing flag.
 *
 * @return     The homing flag.
 */
bool CRTK_robot_state::get_homing(){
  return is_homing;
}



/**
 * @brief      Gets the busy flag.
 *
 * @return     The busy flag.
 */
bool CRTK_robot_state::get_busy(){
  return is_busy;
}



/**
 * @brief      Gets the ready flag.
 *
 * @return     The ready flag.
 */
bool CRTK_robot_state::get_ready(){
  return is_ready;
}



/**
 * @brief      Gets the homed flag.
 *
 * @return     The homed flag.
 */
bool CRTK_robot_state::get_homed(){
  return is_homed;
}



/**
 * @brief      Gets the connected flag.
 *
 * @return     The connected flag.
 */
bool CRTK_robot_state::get_connected(){
  return has_connected;
}
