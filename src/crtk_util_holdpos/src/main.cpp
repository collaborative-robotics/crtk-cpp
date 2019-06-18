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
 * \brief gets the home position and sends it back as servo_cr
 * 
 * \param ns  the namespace of the target robot 
 *
 *
 * \date June 18, 2019
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#ifndef MAIN_
#define MAIN_


#include "main.h"




/**
 * @brief      The main function that hold the robot arm in current pose (using crtk servo_cp command)
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0
 */
int main(int argc, char **argv)
{


  //start ros node
  ros::init(argc, argv, "crtk_util_holdpos");
  static ros::NodeHandle n("~");
  ros::Rate loop_rate(LOOP_RATE);


  std::string space;
  n.getParam("ns", space);
  ROS_INFO("targeting robot named : %s", space.c_str());

  sub_operating_state = n.subscribe("/"+space+"/operating_state", 1, operating_state_cb);
  sub_measured_cp = n.subscribe("/"+space+"/measured_cp", 1, crtk_measured_cp_arm_cb);
  pub_servo_cp = n.advertise<geometry_msgs::TransformStamped>("/"+space+"/servo_cp", 1);
  pub_state_command = n.advertise<crtk_msgs::StringStamped>("/"+space+"/state_command", 1);

  //loop variables
  static int hold = 0, start = 0;
  static tf::Transform hold_pos;


  ROS_INFO("Starting loop. Press 'd' to hold and 'e' to let go.");
  while (ros::ok()){

    //check keyboard
    int key_in = getkey();

    //wait for 'd' to start
    if(key_in == 'd'){
      // grab current position
      hold_pos = current_pos;
      //check that the robot is enabled (and enable if not)
      if (robot_state == 'E' && !is_busy){
        hold = 1;
        start = 1;
      }
      else{
        hold = enable_if_safe(); //tell the user to wait and press d after enabled
      }
    } 
    else if (key_in == 'e'){
      hold = 0;
      ROS_INFO("Letting go!");
    }

    if (hold == 1 && robot_state == 'E' && !is_busy){
      if (start) ROS_INFO("I'm just gonna hold right here");
      start = 0;

      //send pos: call publisher with position
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = msg.header.stamp.now();
      tf::transformTFToMsg(hold_pos,msg.transform);

      pub_servo_cp.publish(msg);
    }



    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}



/**
 *  \fn int getkey()
 *
 *  \brief gets keyboard character for switch case's of console_process()
 *
 *  \return returns keyboard character
 *
 *  \ingroup IO
 *
 *  \return character int
 */
int getkey() {
  int character;
  termios orig_term_attr;
  termios new_term_attr;

  /* set the terminal to raw mode */
  tcgetattr(fileno(stdin), &orig_term_attr);
  memcpy(&new_term_attr, &orig_term_attr, sizeof(termios));
  new_term_attr.c_lflag &= ~(ECHO | ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  /* read a character from the stdin stream without blocking */
  /*   returns EOF (-1) if no character is available */
  character = fgetc(stdin);

  /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return character;
}


/**
 * @brief      arm1 callback function for measured_cp
 *
 * @param[in]  msg   The message
 */
void crtk_measured_cp_arm_cb(geometry_msgs::TransformStamped msg){
  tf::Transform in;
  tf::transformMsgToTF(msg.transform, in);
  
  current_pos = in;
}


/**
 * @brief      updates local copy of robot's state based on the messages from the robot
 *
 * @param[in]  msg   The message from ROS
 */
void operating_state_cb(crtk_msgs::operating_state msg){

  std::string state = msg.state;

  if (state =="DISABLED"){
    robot_state = 'D';
  }
  else if (state =="ENABLED"){
    robot_state = 'E';
  }

  else if (state =="PAUSED"){
    robot_state = 'P';
  }
  else if (state =="FAULT"){
    robot_state = 'F';
  }
  else{
    robot_state = 'f';
  }


  is_homed = msg.is_homed;

  is_busy  = msg.is_busy;

}

/**
 * @brief      transitions to Enabled if the robot can do that safely
 *
 * @return     1 if robot is enabled and not busy
 */
char enable_if_safe(){

CRTK_robot_command command;
  if (!is_homed){
    ROS_INFO("Please home the robot and press 'd' again");
    return 0;
  }
  else if(robot_state == 'P'){ 
    ROS_INFO("Resuming robot, please wait and press 'd' again");
    command = CRTK_RESUME;
    crtk_command_pb(command);
    return 0;
  } 
  else if (robot_state == 'D' ){ 
    ROS_INFO("Enabling robot, please wait and press 'd' again");
    command = CRTK_ENABLE;
    crtk_command_pb(command);
    return 0;
  } 
  else if (robot_state == 'E'){ 
    if(!is_busy)
      return 1; //do nothing - already enabled, not busy
    else{
      ROS_INFO("Robot is already busy, please wait and press 'd' again");
      return 0;
    }
  } 
  else if (robot_state == 'F'){ 
    ROS_INFO("Robot in fault state, please clear fault and press 'd' again");
    return  -1;
  } 
  return 0;
}

/**
 * @brief      send crtk robot state transition command
 *
 * @param[in]  command  The command
 */
void crtk_command_pb(CRTK_robot_command command){

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
  pub_state_command.publish(msg_command);
  ++count;

}

#endif

