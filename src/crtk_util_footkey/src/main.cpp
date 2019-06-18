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
 * \date April 11, 2019
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#ifndef MAIN_
#define MAIN_




#include "ros/ros.h"
#include <crtk_msgs/StringStamped.h>
#include "main.h"
#include <sstream>
#include <iostream>
#include <string>

#include <cstdio>
#include <iomanip>
#include <termios.h>  // needed for terminal settings in getkey()

using namespace std;


#define LOOP_RATE 1000

ros::Publisher command_pub;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


/**
 * @brief      The main function that executes keyboard alternative for foot pedal up and down
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0
 */
int main(int argc, char **argv)
{


  //start ros node
  ros::init(argc, argv, "crtk_util_footkey");
  static ros::NodeHandle n; 
  ros::Rate loop_rate(LOOP_RATE); 

  ROS_INFO("!~~~~~~~~~~~~ Starting keyboard node ~~~~~~~~~~~");
  ROS_INFO("Press 'e' for pedal up, 'd' for pedal down!");

  //init crtk command publishing
  command_pub = n.advertise<crtk_msgs::StringStamped>("crtk_footkey/state_command", 1);

  int count = 0;
  int foot = 0;

  ROS_INFO("Please launch stand alone roscore.");
  while (ros::ok()){

    //check keyboard
    foot = foot_pedal();
    //pub if foot up or down
    if (foot != 0) pub_foot(foot);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}


/**
 * @brief      checks terminal for 'e' or 'd' 
 *
 * @return     -1 for pedal down (d)
 *              0 for null or unsupported entry
 *              1 for pedal up
 *        
 */
int foot_pedal(){
  int key_in;

  key_in = getkey();

  if(key_in == 'd') return -1; 
  else if(key_in == 'e') return 1;
  else return 0;

}


/**
 * @brief      publishes CRTK commands for the given foot command
 *
 * @param[in]  foot  The foot state
 *
 * @return     1 if success
 **/
int pub_foot(int foot){
  static crtk_msgs::StringStamped msg_command;

  if(foot == 1){
    //pub pedal up
    msg_command.data = "pause";
    ROS_INFO("Sent pause.");  
  }

  else if(foot == -1){
    //pub pedal down
    msg_command.data = "resume";
    ROS_INFO("Sent resume.");  

  } else return 0;

  msg_command.header.stamp = msg_command.header.stamp.now();
  command_pub.publish(msg_command);
  return 1;
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


#endif
