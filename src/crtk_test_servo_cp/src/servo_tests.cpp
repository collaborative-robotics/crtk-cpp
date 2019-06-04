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


/**
 * @brief      This function loops through all the crtk tests
 *
 * @param      robot         The robot object
 * @param[in]  current_time  The current time
 *
 * @return     errors
 */
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
      //cartesian cp
      test_status = test_3_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_3_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_3_1 passed: %i", test_status);
      }
      break;
    }
    case 2:
    {
      //rotation cp
      test_status = test_3_2(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_3_2 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_3_2 passed: %i", test_status);
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

/**
 * @brief      The test function 6: 3-1 Absolute (command: servo_cp) Axis motion Test
 *            (functionality) move along X axis for 2 cm (both arms)
 *                 Pass: ask user!
 *            (functionality) move along Z axis for 2 cm (both arms)
 *                 Pass: ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_3_1(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s1,s2;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float dist = 0.025; // 25 mm total
  float completion_percentage_thres = 0.85;  // 0.95;
  static tf::Transform start_pos;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_3-1 ======================= ");
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
    case 3:    case 9:    case 15:    case 21:
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
    case 4:    case 10:    case 16:   case 22:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        s1 = (current_step == 4 || current_step == 10) ? "left" : "right";
        s2 = (current_step == 4 || current_step == 16) ? "-Z"    : "X";
        int curr_arm = (current_step == 4 || current_step == 10) ? 0 : 1;
        ROS_INFO("Moving %s arm along %s for 2 cm ...",s1.c_str(),s2.c_str()); 
        start_pos = robot->arm[curr_arm].get_measured_cp();
        robot->arm[curr_arm].start_motion(current_time);
        ROS_INFO("Start moving robot!");
        current_step ++;
      }
      break;
    }
    case 5:    case 11:    case 17:   case 23:
    {
      // (5) send motion command to move robot (for 2 secs)
      int curr_arm = (current_step == 5 || current_step == 11) ? 0 : 1;
      tf::Vector3 move_vec = (current_step == 5 || current_step == 17) ? -vec_z : vec_x;
      out = robot->arm[curr_arm].send_servo_cp_distance(move_vec,dist,current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 6:    case 12:    case 18:   case 24:
    {
      // (6) check if it moved in the correct direction (msg)
      int curr_arm = (current_step == 6 || current_step == 12) ? 0 : 1;
      CRTK_axis curr_axis = (current_step == 6 || current_step == 18) ? CRTK_Z : CRTK_X;
      float curr_sign = (current_step == 6 || current_step == 18) ? -1.0 : 1.0;
      float check_thres = curr_sign * dist * completion_percentage_thres;
      out = check_movement_distance(&robot->arm[curr_arm], start_pos, curr_axis,check_thres);
      out = step_success(out, &current_step);
      break;
    }
    case 7:    case 13:    case 19:   case 25:
    {
      // (7) ask human if it moved (back)?
      // CRTK_robot_command command = CRTK_PAUSE;
      // robot->state.crtk_command_pb(command); 
      if(current_step == 7 || current_step == 19)
        ROS_INFO("Did the robot move down toward the table? (Y/N)");
      else
        ROS_INFO("Did the robot move away along the table plane? (Y/N)");
      current_step++;
      break;
    }
    case 8:    case 14:    case 20:    case 26:
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
      if(current_step == 27 && out == 1)
        return 1; // at the end of test 2
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}



/**
 * @brief      The test function 7: 3-2 Absolute (command: servo_cp) Axis rotation Test
 *            (functionality) rotate along X axis for 45 degrees (both arms)
 *                 Pass: ask user!
 *            (functionality) rotate along Z axis for 45 degrees (both arms)
 *                 Pass: ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_3_2(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s1,s2;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float angle = 45 DEG_TO_RAD; // 45 degrees total
  float completion_percentage_thres = 0.85;  // 0.95;
  static tf::Transform start_pos;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO(" ==================== Starting test_3-2 cp_rotation ==================== ");
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
    case 3:    case 9:    case 15:    case 21:
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
    case 4:    case 10:    case 16:   case 22:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        s1 = (current_step == 4 || current_step == 10) ? "left" : "right";
        s2 = (current_step == 4 || current_step == 16) ? "-Z"    : "X";
        int curr_arm = (current_step == 4 || current_step == 10) ? 0 : 1;
        ROS_INFO("Rotating %s arm along %s for 45 degrees ...",s1.c_str(),s2.c_str()); 
        start_pos = robot->arm[curr_arm].get_measured_cp();
        robot->arm[curr_arm].start_motion(current_time);
        ROS_INFO("Start moving robot!");
        current_step ++;
      }
      break;
    }
    case 5:    case 11:    case 17:   case 23:
    {
      // (5) send motion command to move robot (for 2 secs)
      int curr_arm = (current_step == 5 || current_step == 11) ? 0 : 1;
      tf::Vector3 move_vec = (current_step == 5 || current_step == 17) ? -vec_z : vec_x;
      out = robot->arm[curr_arm].send_servo_cp_rot_angle(move_vec, angle, current_time);
      out = step_success(out, &current_step);
      break;
    }
    case 6:    case 12:    case 18:   case 24:
    {
      // (6) check if it moved in the correct direction (msg)
      int curr_arm = (current_step == 6 || current_step == 12) ? 0 : 1;
      CRTK_axis curr_axis = (current_step == 6 || current_step == 18) ? CRTK_Z : CRTK_X;
      float curr_sign = (current_step == 6 || current_step == 18) ? -1.0 : 1.0;
      float check_thres = curr_sign * angle * completion_percentage_thres;
      out = 1; //check_movement_rotation(&robot->arm[curr_arm], angle, 1, current_time, start_pos);
      out = step_success(out, &current_step);
      break;
    }
    case 7:    case 13:    case 19:   case 25:
    {
      // (7) ask human if it moved (back)?
      // CRTK_robot_command command = CRTK_PAUSE;
      // robot->state.crtk_command_pb(command); 
      if(current_step == 7 || current_step == 19)
        ROS_INFO("Did the end effector rotate normal to the table? (Y/N)");
      else
        ROS_INFO("Did the robot rotate away parallel to the vertical plane? (Y/N)");
      current_step++;
      break;
    }
    case 8:    case 14:    case 20:    case 26:
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
      if(current_step == 27 && out == 1)
        return 1; // at the end of test 2
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}


