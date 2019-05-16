 /*
 Raven 2 Control - Control software for the Raven II robot
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
 * servo_tests.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *              
 *
 * \date Oct 29, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#include "servo_tests.h"
#include <ros/ros.h>
#include <crtk_msgs/operating_state.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_lib_cpp/crtk_robot_state.h>
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include "test_funcs.h"

using namespace std;

tf::Vector3 vec_x(1,0,0);
tf::Vector3 vec_y(0,1,0);
tf::Vector3 vec_z(0,0,1);

static int start_test = 1;

int servo_testing( CRTK_robot* robot, time_t current_time){
  static time_t start_time = current_time;
  static int current_test = 0;
  static int finished = 0;
  static int errors = 0;
  int test_status;
  int num_of_tests = 1;


  // wait for crtk state message to be published before testing
  if (current_test == 0 && robot->state.get_connected() == 1) {
    current_test = start_test;
  }

  // wait for a beat
  if (current_time - start_time < 2){
        return 0;
  }
  else if (current_test == 0 && !finished){
    ROS_INFO("Robot not connected. %i", robot->state.get_connected()) ;
    return 0;
  }

  // don't do anything after all tests have finished
  if (finished) {
    return 0;
  }

  // start testing!!
  switch(current_test){
    case 1:
    {
      test_status = test_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_1 passed: %i", test_status);
      }
      break;
    }
    default:
    {
      if (finished == 0 && errors != 0){
        // After all tests, send estop command!
        CRTK_robot_command command = CRTK_DISABLE;
        robot->state.crtk_command_pb(command);
        ROS_ERROR("We failed some things.");
        finished = 1;
      }
      else if(finished == 0 && errors == 0){
        // After all tests, send estop command!
        CRTK_robot_command command = CRTK_DISABLE;
        robot->state.crtk_command_pb(command);
        ROS_INFO("We finished everything. Good job!!");
        finished = 1;
      }
    }

  }

  if(current_test > num_of_tests && errors == 0) {
    ROS_INFO("Servo motion testing success!!!");
  }

  return errors;
}


// 1 Motion measured query testing
//    1-1 (measured_js functionality) Move all joints in series manually.
//      Pass: Check that each joint has moved.
//      Pass: Check that each joint velocity has a non-zero value.
//    1-2 (measured_cp functionality) Move tool to the right manually.
//      Pass: Check that the correct axis has been mostly moved in.

int test_1(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out=0;
  std::string start;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_1-1 ======================= ");
      ROS_INFO("Please move each joint of the robot at least %f radians at at least %f rad/s", pos_thresh, vel_thresh);
      ROS_INFO("You have 60 seconds to complete this test. Good luck.");        
      current_step++;
      break;
    }

    case 2:
    {
      // (2) check joints
      out = check_joint_motion_and_vel(robot, pos_thresh, vel_thresh, current_time, (int)60);
      out = step_success(out, &current_step);
      break;
    }

    case 3:
    {
      ROS_INFO("======================= Starting test_1-2 ======================= ");
      ROS_INFO("Press 'Enter' when robot stops moving.");
      current_step ++;
      break;
    }
    case 4:
    {
      // (4) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 5:
    {
      ROS_INFO("Please move left tool 50mm in X -- back.");
      ROS_INFO("You have 30 seconds to complete this test. Good luck.");        
      current_step++;
      break;
    }
    case 6:
    {
      // (6) check arm1 motion in X
      float dist = 0.05; // 50 mm
      out = check_movement_direction(&robot->arm[0], CRTK_X, dist, 30, current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 7:
    {
      ROS_INFO("Please move left tool 50mm in Y -- left.");
      ROS_INFO("You have 30 seconds to complete this test. Good luck.");        
      current_step++;
      break;
    }
    case 8:
    {
      // (8) check arm1 motion in X
      float dist = 0.05; // 50 mm
      out = check_movement_direction(&robot->arm[0], CRTK_Y, dist, 30, current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 9:
    {
      ROS_INFO("Please move right tool 50mm in X -- back.");
      ROS_INFO("You have 30 seconds to complete this test. Good luck.");        
      current_step++;
      break;
    }
    case 10:
    {
      // (10) check arm1 motion in X
      float dist = 0.05; // 50 mm
      out = check_movement_direction(&robot->arm[1], CRTK_X, dist, 30, current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 11:
    {
      ROS_INFO("Please move right tool 50mm in Y -- left.");
      ROS_INFO("You have 30 seconds to complete this test. Good luck.");        
      current_step++;
      break;
    }
    case 12:
    {
      // (12) check arm1 motion in X
      float dist = 0.05; // 50 mm
      out = check_movement_direction(&robot->arm[1], CRTK_Y, dist, 30, current_time);
      out = step_success(out, &current_step);
      if(out == 1)
        return 1;
      break;
    }
    // default:
    // {
    //   if(out == 1)
    //     return 1;
    //   else
    //     return -100;
    //   break;
    // }


  }
  if(out < 0) return out;
  return 0;
}