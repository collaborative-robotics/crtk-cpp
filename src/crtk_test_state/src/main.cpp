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

#ifndef MAIN_
#define MAIN_

#include "ros/ros.h"
#include <crtk_msgs/operating_state.h>
#include <crtk_msgs/StringStamped.h>

#include "state_tests.h"
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
using namespace std;

int is_raven = 0;

/**
 * The main function creates a robot state and initiates it before running a loop for the
 * state testing.
 */


/**
 * @brief      The main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0
 */
int main(int argc, char **argv)
{

  time_t current_time;
  

  ros::init(argc, argv, "crtk_test_state");
  static ros::NodeHandle n("~"); 

  std::string r_space;
  if(!n.getParam("r_space", r_space))
    ROS_ERROR("No Robot namespace provided in command line!");
  
  if(r_space == "arm1" || r_space == "arm2")
    is_raven = 1;

  CRTK_robot_state robot_state(n,r_space);

  int count = 0;
  ros::Rate loop_rate(10); // \TODO increase loop rate?

  ROS_INFO("Please launch stand alone roscore.");
  while (ros::ok()){

    current_time = time(NULL);
    state_testing(robot_state, current_time);


    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}




#endif
