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
 * \date Aug 14, 2019
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
      // simple servo_jv test
      test_status = test_6_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_6_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_6_1 passed: %i", test_status);
      }
      break;
    }
    case 2:   
    {
      // simple servo_jv test
      test_status = test_6_2(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_6_2 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_6_2 passed: %i", test_status);
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
 * @brief      The test function 9: 6-1 Simple joint velocity (command: servo_jv) 
 *            (functionality) move 10 degrees in the shoulder and tool joints
 *                 Pass: ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_6_1(CRTK_robot *robot, time_t current_time)
{
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  static int joint_index = 0;
  static std::string s;
  std::string start;
  float angle = 20 DEG_TO_RAD; // 10 degrees total
  float distance = 0.03;   // 3 cm in total
  float max_omega = 20 DEG_TO_RAD; // per second 
  float max_pris = 0.03;           // meters per second 
  static float average_jv, extreme_jv, current_jv;
  static int iteration, extreme_iteration, delay_iteration;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO(" ==================== Starting test_6-1 shoulder jv test ==================== ");
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
    case 3:    case 9:    case 15: 
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
    case 4:    case 10:     case 16: 
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){

        if(current_step == 4)
          joint_index = 0;     
        else if(current_step == 10)
          joint_index = 4;
        else
          joint_index = 6;

        if(joint_index >= robot->get_max_joints())
          joint_index = std::rand() % robot->get_max_joints();
        std::stringstream gstream; 
        gstream << joint_index; 
        s = gstream.str();                 

        ROS_INFO("Moving robot arm in joint %s ...",s.c_str()); 
        robot->arm.start_motion(current_time);
        ROS_INFO("Start moving robot!");
        average_jv = 0.0;
        extreme_jv = 0.0;
        iteration = 0;
        extreme_iteration = 0;
        delay_iteration = 0;
        current_step ++;
      }
      break;
    }
    case 5:    case 11:     case 17:
    {
      // (5) send motion command to move robot (for 2 secs)

      if(robot->arm.is_prismatic(joint_index))
      {
        out = robot->arm.go_to_jpos(2,joint_index, distance, current_time, max_omega, max_pris);
      }
      else
      {
        out = robot->arm.go_to_jpos(2,joint_index, angle, current_time, max_omega, max_pris);
      }
      current_jv = robot->arm.get_measured_js_vel(joint_index);
      iteration = iteration + 1;
      average_jv = (average_jv * iteration + current_jv)/(iteration + 1);
      if(fabs(robot->arm.get_measured_js_vel(joint_index)) > fabs(extreme_jv))
      {
        extreme_iteration = iteration;
        extreme_jv = current_jv;
      }
      out = step_success(out, &current_step);
      break;
    }
    case 6:    case 12:    case 18:
    {
      // (6) do a dance
      if(robot->arm.get_measured_js_vel(joint_index) == 0 || delay_iteration > 3*LOOP_RATE)
        out = step_success(1, &current_step);
      else
      {
        out = 0;
        delay_iteration ++;
      }
      
      break;
    }
    case 7:    case 13:    case 19: 
    {
      // (7) ask human if it moved (back)?
      ROS_INFO("Process complete: avg v = %f (out of total iterations: %d)", average_jv, iteration);
      ROS_INFO("                  max v = %f (happened at iteration: %d)", extreme_jv, extreme_iteration);
      if(delay_iteration > LOOP_RATE)
        ROS_INFO("                  time delay = over 3 secs.. (not very good)");
      else
        ROS_INFO("                  time delay = %f secs (%d iterations)", (1.0*delay_iteration)/(1.0*LOOP_RATE), delay_iteration);

      if(robot->arm.is_prismatic(joint_index))
      {
        ROS_INFO("                  achieved ratio = (%f)/(%f) = %f% (max/desired v)\n",extreme_jv, max_pris, 100*extreme_jv/max_pris);
        ROS_INFO("Did joint %s move about %f mm? (Y/N)", s.c_str(),distance M_TO_MM);
      }
      else
      {
        ROS_INFO("                  achieved ratio = (%f)/(%f) = %f% (max/desired v)\n",extreme_jv, max_omega, 100*extreme_jv/max_omega);
        ROS_INFO("Did joint %s move about %f degrees? (Y/N)", s.c_str(),angle RAD_TO_DEG);
      }
      current_step++;
      break;
    }
    case 8:    case 14:    case 20: 
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
      if(current_step == 21 && out == 1)
        return 1; // at the end of test
      break;
    }
  }
  if(out < 0) return out;
  return 0;

}



/**
 * @brief      The test function 9: 6-2 Go home (command: servo_jv) 
 *            (functionality) move back to home pose (both arms)
 *                 Pass: ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_6_2(CRTK_robot *robot, time_t current_time)
{
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float angle = 45 DEG_TO_RAD; // 45 degrees total
  float completion_percentage_thres = 0.85;  // 0.95;
  static float home[MAX_JOINTS];
  float max_omega = 20 DEG_TO_RAD; // per second 
  float max_pris = 0.03;           // meters per second 

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO(" ==================== Starting test_6-2 the voyage home ==================== ");
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
        ROS_INFO("Taking robot arm home ..."); 
        robot->arm.start_motion(current_time);
        robot->arm.get_home_jpos(home);
        ROS_INFO("Start moving robot!");
        current_step ++;
      }
      break;
    }
    case 5: 
    {
      // (5) send motion command to move robot (for 2 secs)
      robot->arm.get_home_jpos(home);
      out = robot->arm.go_to_jpos(2, home, current_time, max_omega, max_pris);
      out = step_success(out, &current_step);
      break;
    }
    case 6: 
    {
      // (6) do a dance
      out = 1; //check_movement_rotation(&robot->arm[curr_arm], angle, 1, current_time, start_pos);
      out = step_success(out, &current_step);
      break;
    }
    case 7: 
    {
      // (7) ask human if it moved (back)?
      // CRTK_robot_command command = CRTK_PAUSE;
      // robot->state.crtk_command_pb(command); 
      ROS_INFO("Did the end effector go home!? (Y/N)");
      current_step++;
      break;
    }
    case 8: 
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
      if(current_step == 9 && out == 1)
        return 1; // at the end of test
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}

