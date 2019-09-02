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
 * \date   Aug 14, 2019
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
  int num_of_tests = 3;


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
      test_status = test_7_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_7_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_7_1 passed: %i", test_status);
      }
      break;
    }

    case 2:
    {
      test_status = test_7_2(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_7_2 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_7_2 passed: %i", test_status);
      }
      break;
    }

    case 3:
    {
      test_status = test_7_3(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_7_3 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_7_3 passed: %i", test_status);
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
 * @brief      The test function 7: 7-1 Relative (command: servo_cv) Axis motion Test
 *             (functionality) move along X axis for 2 secs (both arms)
 *                 Pass: Check raven state
 *             (functionality) move along Y axis for 2 secs (both arms)
 *                 Pass: Check raven state
 *             (functionality) move along Z axis for 2 secs (both arms)
 *                 Pass: Check raven state
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_7_1(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float dist = 0.02; // 20 mm total
  float completion_percentage_thres = 0.85;  // 0.95;
  float duration_time = 2; // sec

  static tf::Transform start_pos;
  static float current_cv, extreme_cv, average_cv;
  static int iteration, extreme_iteration, delay_iteration;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_7-1 ======================= ");
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
      // (3) send resume command to enable robot
      ROS_INFO("CRTK_RESUME command sent.");
      ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
      CRTK_robot_command command = CRTK_RESUME;
      robot->state.crtk_command_pb(command); 
      current_step++;
      break;
    }
    case 4:    case 10:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        s = (current_step == 4) ? "-Z"    : "X";
        ROS_INFO("Moving robot arm along %s for 2 seconds...",s.c_str()); 
        start_pos = robot->arm.get_measured_cp();
        robot->arm.start_motion(current_time);
        average_cv = 0.0;
        extreme_cv = 0.0;
        iteration = 0;
        extreme_iteration = 0;
        delay_iteration = 0;
        current_step ++;
      }
      break;
    }
    case 5:    case 11:
    {
      // (5) send motion command to move robot (for 2 secs)
      tf::Vector3 move_vec = (current_step == 5) ? -vec_z : vec_x;
      int move_idx = (current_step == 5) ? 2 : 0;
      out = robot->arm.send_servo_cv_time(move_vec,dist,duration_time,current_time);
      out = step_success(out, &current_step);

      current_cv = robot->arm.get_measured_cv().getOrigin()[move_idx];
      iteration = iteration + 1;
      average_cv = (average_cv * iteration + current_cv)/(iteration + 1);
      if(fabs(robot->arm.get_measured_cv().getOrigin()[move_idx]) > fabs(extreme_cv))
      {
        extreme_iteration = iteration;
        extreme_cv = current_cv;
      }
      break;
    }
    case 6:    case 12:
    {
      // (6) check if it moved in the correct direction (msg)
      CRTK_axis curr_axis = (current_step == 6) ? CRTK_Z : CRTK_X;
      float curr_sign = (current_step == 6) ? -1.0 : 1.0;
      float check_thres = curr_sign * dist * completion_percentage_thres;
      out = check_movement_distance(&robot->arm, start_pos, curr_axis,check_thres);

      int move_idx = (current_step == 6) ? 2 : 0;
      if(robot->arm.get_measured_cv().getOrigin()[move_idx] == 0 || delay_iteration > 3*LOOP_RATE)
        out = step_success(out, &current_step);
      else
      {
        out = 0;
        delay_iteration ++;
      }
      break;
    }
    case 7:    case 13:
    {
      // (7) ask human if it moved (back)?
      // CRTK_robot_command command = CRTK_PAUSE;
      // robot->state.crtk_command_pb(command); 
      ROS_INFO("Process complete: avg v = %f (out of total iterations: %d)", average_cv, iteration);
      ROS_INFO("                  max v = %f (happened at iteration: %d)", extreme_cv, extreme_iteration);
      if(delay_iteration > LOOP_RATE)
        ROS_INFO("                  time delay = over 3 secs.. (not very good)");
      else
        ROS_INFO("                  time delay = %f secs (%d iterations)", (1.0*delay_iteration)/(1.0*LOOP_RATE), delay_iteration);

      ROS_INFO("                  achieved ratio = (%f)/(%f) = %f%% (max/desired v)\n",extreme_cv, (dist/duration_time), 100*extreme_cv/(dist/duration_time));

      if(current_step == 7)
        ROS_INFO("Did the robot move down toward the table? (Y/N)");
      else
        ROS_INFO("Did the robot move away along the table plane? (Y/N)");
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
        return 1; // at the end of test 2
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}


/**
 * @brief      The test function 7: 7-2 Relative (command: servo_cv) Cube tracing Test
 *             (functionality) (functionality) Trace a cube
 *                 Pass: Ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_7_2(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s;

  float dist = 0.01; // 10 mm total
  int duration = 1;
  float completion_percentage_thres = 0.85;  // 0.95;
  static tf::Transform start_pos;

  static char curr_vertex = 0b110; //start arm 0 in front left upper
  static tf::Vector3 move_vec;
  static CRTK_axis prev_axis = CRTK_Z;

  static char edge_count = 0;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_7-2 ======================= ");
      ROS_INFO("Start and home robot if not already.");
      ROS_INFO("(Press 'Enter' when done.)"); 
      ROS_INFO("In this test, the arms should randomly trace a cube.");
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
    case 3:
    {
      // (3) send resume command to enable robot
      ROS_INFO("CRTK_RESUME command sent.");
      ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
      CRTK_robot_command command = CRTK_RESUME;
      robot->state.crtk_command_pb(command); 
      robot->arm.start_motion(current_time);
      current_step++;
      break;
    }
    case 4:
    {
      // (4) send motion command to move left robot arm down (for 2 secs)
      if (robot->state.get_enabled()){
        out = robot->arm.send_servo_cv_time(-vec_z,dist,duration,current_time);
        out = step_success(out, &current_step);
      }
      break;
    }
    case 5:
    {
      // (6) record start pos
      ROS_INFO("Start randomly tracing a cube.");
      current_step ++;
      break;
    }
    case 6:
    {
      rand_cube_dir(&curr_vertex, &move_vec, &prev_axis);
      robot->arm.start_motion(current_time);
      ROS_INFO("\t step 7 length ->  %f", move_vec.length());

      edge_count++;
      current_step++;
      break;
    }
    case 7:
    {
      out = 0;
      if(!out) out = robot->arm.send_servo_cv_time(move_vec,dist,duration,current_time);
      else           robot->arm.send_servo_cv(tf::Transform());

      out = step_success(out, &current_step);
      if(out && edge_count < 11) current_step = 7; 
      break;
    }
    case 8:
    {
      ROS_INFO("Did the robot make a nice cube? (Y/N)");
      current_step++;
      break;
    }

    case 9:
    {
      // (9) take user input yes or no
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
      if(out == 1)
        return 1; // at the end of test 2
      break;
    }
  }
  if(out < 0) return out;
  return 0;
}


/**
 * @brief      The test function 7: 7-3 Relative (command: servo_cv) Orientation axis test
 *            (functionality) simple rotation commands test
 *                 Pass: ask user!
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current time
 *
 * @return     success 1, fail otherwise
 */
int test_7_3(CRTK_robot * robot, time_t current_time){
  static int current_step = 1;
  int duration = 1, out = 0;
  float step_angle = 2*0.000262;
  static tf::Vector3 motion_vec;
  std::string start;
  static float average_cv, extreme_cv, current_cv;
  static int iteration, extreme_iteration, delay_iteration;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_7-3 ======================= ");
      ROS_INFO("Start and home robot if not already.");
      ROS_INFO("(Press 'Enter' when done.)"); 
      ROS_INFO("In this test, the arms should subsequently rotate around X,Y,Z axes for %i secs each.",duration);
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
    case 3:  
    {
      // (3) send resume command to enable robot
      ROS_INFO("CRTK_RESUME command sent.");
      ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
      CRTK_robot_command command = CRTK_RESUME;
      robot->state.crtk_command_pb(command); 
      current_step++;
      break;
    }
    case 4:  case 9:  case 14:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        robot->arm.start_motion(current_time);
        average_cv = 0.0;
        extreme_cv = 0.0;
        iteration = 0;
        extreme_iteration = 0;
        delay_iteration = 0;
        current_step ++;
      }
      break;
    }
    case 5:  case 10:   case 15:
    {
      // (5) send motion command to move robot (for 2 secs)
      if(current_step == 5)
        motion_vec = vec_x;
      else if(current_step == 10)
        motion_vec = vec_y;
      else
        motion_vec = vec_z;

      tf::Transform trans = tf::Transform(tf::Quaternion(motion_vec,step_angle));
      out = robot->arm.send_servo_cv(trans);

      current_cv = robot->arm.get_measured_cv().getRotation().getAngleShortestPath();
      iteration = iteration + 1;
      average_cv = (average_cv * iteration + current_cv)/(iteration + 1);
      if(fabs(robot->arm.get_measured_cv().getRotation().getAngleShortestPath()) > fabs(extreme_cv))
      {
        extreme_iteration = iteration;
        extreme_cv = current_cv;
      }

      if(current_time-robot->arm.get_start_time() > duration){
        robot->arm.start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);
      }
      break;
    }
    case 6:  case 11:   case 16:  case 8:  case 13:
    {
      // (6) send nothing 
      // (8) send nothing 
      // if(current_time-robot->arm.get_start_time() > duration){
      //   robot->arm.start_motion(current_time);
      //   current_step ++;
      //   ROS_INFO("moving to step %i",current_step);
      // }
      if(robot->arm.get_measured_cv().getRotation().getAngleShortestPath() == 0 || delay_iteration > 3*LOOP_RATE)
      {
        out = step_success(1, &current_step);
        ROS_INFO("moving to step %i",current_step);
      }
      else
      {
        out = 0;
        delay_iteration ++;
      }
      break;
    }
    case 7:  case 12:   case 17:
    {
      // (7) send motion command to move robot (for 2 secs)
      tf::Transform trans = tf::Transform(tf::Quaternion(-motion_vec,step_angle));
      out = robot->arm.send_servo_cv(trans);
      if(current_time-robot->arm.get_start_time() > duration){
        robot->arm.start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);

        ROS_INFO("Process complete: avg v = %f (out of total iterations: %d)", average_cv, iteration);
        ROS_INFO("                  max v = %f (happened at iteration: %d)", extreme_cv, extreme_iteration);
        if(delay_iteration > LOOP_RATE)
          ROS_INFO("                  time delay = over 3 secs.. (not very good)");
        else
          ROS_INFO("                  time delay = %f secs (%d iterations)", (1.0*delay_iteration)/(1.0*LOOP_RATE), delay_iteration);

        ROS_INFO("                  achieved ratio = (%f)/(%f) = %f%% (max/desired v)\n",extreme_cv, (step_angle*LOOP_RATE), 100*extreme_cv/(step_angle*LOOP_RATE));
      }
      break;
    }
    case 18:
    {
      ROS_INFO("Did the arm subsequently rotate around X,Y,Z axes for %i secs each? (Y/N)",duration);
      current_step++;
      break;
    }
    case 19:
    {
      // (8) take user input yes or no
      getline(std::cin,start);
      if(start == "Y" || start == "y"){
        step_success(1, &current_step);
        return 1;
      }
      else if(start == "N" || start == "n"){
        step_success(-1, &current_step);
        return -1;
      }
      else{
        current_step --;
      }
      break;
    }
  }
  return 0;
}