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
  int num_of_tests = 2;


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
      // simple servo_jr test
      test_status = test_5_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_5_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_5_1 passed: %i", test_status);
      }
      break;
    }
    case 2:   
    {
      // simple servo_jr test
      test_status = test_5_2(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_5_2 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_5_2 passed: %i", test_status);
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


// 5-1 Absolute joint test (command: servo_jp) 
// (functionality) move 10 degrees in the shoulder and tool joints
//    Pass: Ask user
int test_5_1(CRTK_robot *robot, time_t current_time)
{
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  int joint_index = 0;
  std::string start, s1,s2;
  float angle = 45 DEG_TO_RAD; // 10 degrees total
  float distance = 0.03;   // 3 cm in total

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO(" ==================== Starting test_5-1 shoulder jp test ==================== ");
      ROS_INFO("Start and home robot if not already.");
      ROS_INFO("(Press 'Enter' when done.)"); 
      current_step ++;
      break;
    }
    case 2:
    {
      // (2) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 3:    case 9:    case 15:    case 21:    case 27:    case 33:
    {
      static int started  = 0;
      if(!started){
      // (3) send resume command to enable robot
        ROS_INFO("CRTK_RESUME command sent.");
        ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
        CRTK_robot_command command = CRTK_RESUME;
        robot->state.crtk_command_pb(command); 
        pause_start = current_time;
        started = 1;
      }
      else if(current_time - pause_start > 1){
        started = 0;
        current_step++;
      }
      break;
    }
    case 4:    case 10:     case 16:    case 22:    case 28:     case 34:  
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        bool arm_check = current_step == 4 || current_step == 16 || current_step == 28;
        int curr_arm = (arm_check) ? 0 : 1;

        s1 = (arm_check) ? "left" : "right";
        if(current_step <= 10)        s2 = "shoulder";
        else if(current_step <= 22)   s2 = "tool roll";
        else                          s2 = "tool grasper";

        ROS_INFO("Moving %s arm in the %s joint ...",s1.c_str(), s2.c_str()); 
        robot->arm[curr_arm].start_motion(current_time);
        ROS_INFO("Start moving robot!");
        current_step ++;
      }
      break;
    }
    case 5:    case 11:     case 17:    case 23:      case 29:     case 35:
    {
      // (5) send motion command to move robot (for 2 secs)
      bool arm_check = current_step == 5 || current_step == 17 || current_step == 29;
      int curr_arm = (arm_check) ? 0 : 1;

      if(current_step <= 11)        joint_index = 0;
      else if(current_step <= 23)   joint_index = 4;
      else                          joint_index = 6;

      if(joint_index == 2)
        out = robot->arm[curr_arm].go_to_jpos(1,joint_index, distance, current_time);
      else
        out = robot->arm[curr_arm].go_to_jpos(1,joint_index, angle, current_time);

      out = step_success(out, &current_step);
      break;
    }
    case 6:    case 12:    case 18:    case 24:    case 30:     case 36:   
    {
      // (6) do a dance
      out = 1; 
      out = step_success(out, &current_step);
      break;
    }
    case 7:    case 13:    case 19:     case 25:    case 31:    case 37:
    {
      // (7) ask human if it moved (back)?
      if(current_step <= 13)        s2 = "shoulder";
      else if(current_step <= 25)   s2 = "tool insertion";
      else                          s2 = "tool grasper";
      ROS_INFO("Did the %s move aout %f degrees? (Y/N)", s2.c_str(),angle RAD_TO_DEG);
      current_step++;
      break;
    }
    case 8:    case 14:    case 20:     case 26:     case 32:    case 38:
    {
      // (8) take user input yes or no
      getline(std::cin,start);
      if(start == "Y" || start == "y"){
        out = 1;
        out = step_success(out, &current_step);
      }
      else if(start == "N" || start == "n"){
        out = -1;
        out = step_success(out, &current_step);
      }
      else{
        current_step --;
      }
      if(current_step == 39 && out == 1)
        return 1; // at the end of test
      break;
    }
  }
  if(out < 0) return out;
  return 0;

}


// 5-2 Go home (command: servo_jp) 
// (functionality) move back to home pose (both arms)
//    Pass: Ask user
int test_5_2(CRTK_robot *robot, time_t current_time)
{
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s1,s2;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float angle = 45 DEG_TO_RAD; // 45 degrees total
  float completion_percentage_thres = 0.85;  // 0.95;
  static float home[MAX_JOINTS];

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO(" ==================== Starting test_5-2 the voyage home ==================== ");
      ROS_INFO("Start and home robot if not already.");
      ROS_INFO("(Press 'Enter' when done.)"); 
      current_step ++;
      break;
    }
    case 2:
    {
      // (2) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 3:    case 9:    
    {
      static int started  = 0;
      if(!started){
      // (3) send resume command to enable robot
        ROS_INFO("CRTK_RESUME command sent.");
        ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
        CRTK_robot_command command = CRTK_RESUME;
        robot->state.crtk_command_pb(command); 
        pause_start = current_time;
        started = 1;
      }
      else if(current_time - pause_start > 1){
        started = 0;
        current_step++;
      }
      break;
    }
    case 4:    case 10:    
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        int curr_arm = (current_step == 4) ? 0 : 1;
        s1 = (current_step == 4) ? "left" : "right";
        ROS_INFO("Taking %s arm home ...",s1.c_str()); 
        robot->arm[curr_arm].start_motion(current_time);
        robot->arm[curr_arm].get_home_jpos(home);
        ROS_INFO("Start moving robot!");
        current_step ++;
      }
      break;
    }
    case 5:    case 11:    
    {
      // (5) send motion command to move robot (for 2 secs)
      int curr_arm = (current_step == 5) ? 0 : 1;
      robot->arm[curr_arm].get_home_jpos(home);
      out = robot->arm[curr_arm].go_to_jpos(1, home, current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 6:    case 12:    
    {
      // (6) do a dance
      out = 1; //check_movement_rotation(&robot->arm[curr_arm], angle, 1, current_time, start_pos);
      out = step_success(out, &current_step);
      break;
    }
    case 7:    case 13:    
    {
      // (7) ask human if it moved (back)?
      // CRTK_robot_command command = CRTK_PAUSE;
      // robot->state.crtk_command_pb(command); 
      ROS_INFO("Did the end effector go home!? (Y/N)");
      current_step++;
      break;
    }
    case 8:    case 14:    
    {
      // (8) take user input yes or no
      getline(std::cin,start);
      if(start == "Y" || start == "y"){
        out = 1;
        out = step_success(out, &current_step);
      }
      else if(start == "N" || start == "n"){
        out = -1;
        out = step_success(out, &current_step);
      }
      else{
        current_step --;
      }
      if(current_step == 15 && out == 1)
        return 1; // at the end of test
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}