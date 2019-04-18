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
#include "ros/ros.h"
#include <crtk_msgs/operating_state.h>
#include <crtk_msgs/StringStamped.h>
#include "crtk_robot_state.h"
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>
#include "test_funcs.h"

using namespace std;

tf::Vector3 vec_x(1,0,0);
tf::Vector3 vec_y(0,1,0);
tf::Vector3 vec_z(0,0,1);

int servo_testing( CRTK_robot* robot, time_t current_time){
  static time_t start_time = current_time;
  static int current_test = 0;
  static int finished = 0;
  static int errors = 0;
  int test_status;
  int num_of_tests = 6;


  // wait for crtk state message to be published before testing
  if (current_test == 0 && robot->state.get_connected() == 1) {
    current_test = 6;
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
    case 2:
    {
      test_status = test_2_1(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_2_1 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_2_1 passed: %i", test_status);
      }
      break;
    }

    case 3:
    {
      test_status = test_2_2(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_2_2 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_2_2 passed: %i", test_status);
      }
      break;
    }

    case 4:
    {
      test_status = test_2_3(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_2_3 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_2_3 passed: %i", test_status);
      }
      break;
    }

    case 5:
    {
      test_status = test_2_4(robot, current_time);
      if (test_status < 0) {
        errors += 1;
        current_test ++;
        ROS_ERROR("test_2_4 fail: %i", test_status);
      }
      else if (test_status > 0) {
        current_test ++;
        ROS_INFO("test_2_4 passed: %i", test_status);
      }
      break;
    }
    case 6:
    {
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


// 2-1 Relative (command: servo_cr) Axis motion Test
// (functionality) move along X axis for 2 secs (both arms)
//    Pass: Check raven state
// (functionality) move along Y axis for 2 secs (both arms)
//    Pass: Check raven state
// (functionality) move along Z axis for 2 secs (both arms)
//    Pass: Check raven state
int test_2_1(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s1,s2;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float dist = 0.02; // 20 mm total
  float completion_percentage_thres = 0.85;  // 0.95;
  static tf::Transform start_pos;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_2-1 ======================= ");
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
      // (3) send resume command to enable robot
      ROS_INFO("CRTK_RESUME command sent.");
      ROS_INFO("Waiting for robot to enter CRTK_ENABLED state..."); 
      CRTK_robot_command command = CRTK_RESUME;
      robot->state.crtk_command_pb(command); 
      current_step++;
      break;
    }
    case 4:    case 10:    case 16:   case 22:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        s1 = (current_step == 4 || current_step == 10) ? "left" : "right";
        s2 = (current_step == 4 || current_step == 16) ? "-Z"    : "X";
        int curr_arm = (current_step == 4 || current_step == 10) ? 0 : 1;
        ROS_INFO("Moving %s arm along %s for 2 seconds...",s1.c_str(),s2.c_str()); 
        start_pos = robot->arm[curr_arm].get_measured_cp();
        robot->arm[curr_arm].start_motion(current_time);
        current_step ++;
      }
      break;
    }
    case 5:    case 11:    case 17:   case 23:
    {
      // (5) send motion command to move robot (for 2 secs)
      int curr_arm = (current_step == 5 || current_step == 11) ? 0 : 1;
      tf::Vector3 move_vec = (current_step == 5 || current_step == 17) ? -vec_z : vec_x;
      out = robot->arm[curr_arm].send_servo_cr_time(move_vec,dist,2,current_time);
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

// 2-2 Relative (command: servo_cr) Cube tracing Test
// (functionality) Trace a cube
//    Pass: Ask user!

int test_2_2(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0, out1 = 0, out2 = 0;
  std::string start, s1,s2;

  float dist = 0.01; // 10 mm total
  int duration = 1;
  float completion_percentage_thres = 0.85;  // 0.95;
  static tf::Transform start_pos1, start_pos2;

  static char curr_vertex0 = 0b110; //start arm 0 in front left upper
  static char curr_vertex1 = 0b110; //start arm 1 in front left upper
  static tf::Vector3 move_vec0, move_vec1;
  static CRTK_axis prev_axis0 = CRTK_Z;
  static CRTK_axis prev_axis1 = CRTK_Z; 

  static char edge_count = 0;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_2-2 ======================= ");
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
      robot->arm[0].start_motion(current_time);
      current_step++;
      break;
    }
    case 4:
    {
      // (4) send motion command to move left robot arm down (for 2 secs)
      if (robot->state.get_enabled()){
        out = robot->arm[0].send_servo_cr_time(-vec_z,dist,duration,current_time);
        out = step_success(out, &current_step);
        if(out) robot->arm[1].start_motion(current_time);
      }
      break;
    }
    case 5:
    {
      // (5) send motion command to move right robot arm down (for 2 secs)
      if (robot->state.get_enabled()){
        out = robot->arm[1].send_servo_cr_time(-vec_z,dist,duration,current_time);
        out = step_success(out, &current_step);
      }
      break;
    }
    case 6:
    {
      // (6) record start pos
      ROS_INFO("Start randomly tracing a cube.");
      current_step ++;
      break;
    }
    case 7:
    {
      rand_cube_dir(&curr_vertex0, &move_vec0, &prev_axis0);
      rand_cube_dir(&curr_vertex1, &move_vec1, &prev_axis1);
      robot->arm[0].start_motion(current_time);
      robot->arm[1].start_motion(current_time);
      ROS_INFO("\t step 7 length -> (left) %f, (right) %f", move_vec0.length(),move_vec1.length());

      edge_count++;
      current_step++;
      break;
    }
    case 8:
    {
      out1 = 0;
      out2 = 0;
      if(!out1) out1 = robot->arm[0].send_servo_cr_time(move_vec0,dist,duration,current_time);
      else             robot->arm[0].send_servo_cr(tf::Transform());
      if(!out2) out2 = robot->arm[1].send_servo_cr_time(move_vec1,dist,duration,current_time);
      else             robot->arm[1].send_servo_cr(tf::Transform());

      out = step_success(out1 && out2, &current_step);
      if(out && edge_count < 11) current_step = 7; 
      break;
    }
    case 9:
    {
      ROS_INFO("Did the robot make a nice cube? (Y/N)");
      current_step++;
      break;
    }

    case 10:
    {
      // (10) take user input yes or no
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


// 2-3 Relative (command: servo_cr) Orientation axis test
// (functionality) simple rotation commands test
// Pass: ask user!
int test_2_3(CRTK_robot * robot, time_t current_time){
  static int current_step = 1;
  static int curr_arm = 0;
  int duration = 1, out = 0;
  float step_angle = 2*0.000262;
  static tf::Vector3 motion_vec;
  std::string start;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_2-3 ======================= ");
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
    case 4:  case 9:  case 14:  case 19:  case 24:  case 29:
    {
      // (4) check if crtk == enabled
      curr_arm = (current_step % 2 == 0) ? 0:1;
      if (robot->state.get_enabled()){
        robot->arm[curr_arm].start_motion(current_time);
        current_step ++;
      }
      break;
    }
    case 5:  case 10:   case 15:  case 20:  case 25:  case 30:
    {
      // (5) send motion command to move robot (for 2 secs)
      if(current_step <= 10)
        motion_vec = vec_x;
      else if(current_step <= 20)
        motion_vec = vec_y;
      else
        motion_vec = vec_z;

      tf::Transform trans = tf::Transform(tf::Quaternion(motion_vec,step_angle));
      out = robot->arm[curr_arm].send_servo_cr(trans);
      if(current_time-robot->arm[curr_arm].get_start_time() > duration){
        robot->arm[curr_arm].start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);
      }
      break;
    }
    case 6:  case 11:   case 16:  case 21: case 26: case 31:  case 8:  case 13:   case 18:  case 23: case 28: case 33:
    {
      // (6) send nothing 
      // (8) send nothing 
      // if(current_time-robot->arm[curr_arm].get_start_time() > duration){
      //   robot->arm[curr_arm].start_motion(current_time);
      //   current_step ++;
      //   ROS_INFO("moving to step %i",current_step);
      // }
      current_step ++;
      ROS_INFO("moving to step %i",current_step);
      break;
    }
    case 7:  case 12:   case 17:  case 22: case 27: case 32:
    {
      // (7) send motion command to move robot (for 2 secs)
      tf::Transform trans = tf::Transform(tf::Quaternion(-motion_vec,step_angle));
      out = robot->arm[curr_arm].send_servo_cr(trans);
      if(current_time-robot->arm[curr_arm].get_start_time() > duration){
        robot->arm[curr_arm].start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);
      }
      break;
    }
    case 34:
    {
      ROS_INFO("Did the arms subsequently rotate around X,Y,Z axes for %i secs each? (Y/N)",duration);
      current_step++;
      break;
    }
    case 35:
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

// 2-4 Relative (command: servo_cr for grasper) Grasper test
// (functionality) clapping with grasper for 2 sec (max = 30 deg)
// Pass: ask user!
int test_2_4(CRTK_robot * robot, time_t current_time){
  static int current_step = 1;
  static int direction = -1;
  int duration = 1, out = 0;
  float step_angle = 0.0005;
  std::string start;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_2-4 ======================= ");
      ROS_INFO("Start and home robot if not already.");
      ROS_INFO("(Press 'Enter' when done.)"); 
      ROS_INFO("In this test, the graspers should clap several times for around %i seconds each direction.",duration);
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
    case 4:  case 9:  case 14:  case 19:  case 24:  case 29:
    {
      // (4) check if crtk == enabled
      if (robot->state.get_enabled()){
        robot->arm[0].start_motion(current_time);
        robot->arm[1].start_motion(current_time);
        current_step ++;
      }
      break;
    }
    case 5:  case 10:   case 15:  case 20:  case 25:  case 30:
    {
      // (5) send motion command to move robot (for 1 sec)
      direction = (current_step % 2 == 0) ? 1:-1;
      robot->arm[0].send_servo_jr_grasp(direction*step_angle);
      robot->arm[1].send_servo_jr_grasp(direction*step_angle);
      if(current_time-robot->arm[0].get_start_time() > duration){
        robot->arm[0].start_motion(current_time);
        robot->arm[1].start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);
      }
      break;
    }
    case 6:  case 11:   case 16:  case 21: case 26: case 31:  case 8:  case 13:   case 18:  case 23: case 28: case 33:
    {
      // (6) send nothing 
      // (8) send nothing 
      current_step ++;
      ROS_INFO("moving to step %i",current_step);
      break;
    }
    case 7:  case 12:   case 17:  case 22: case 27: case 32:
    {
      // (7) send motion command to move robot (for 2 secs)
      robot->arm[0].send_servo_jr_grasp(-direction*step_angle);
      robot->arm[1].send_servo_jr_grasp(-direction*step_angle);
      if(current_time-robot->arm[0].get_start_time() > duration){
        robot->arm[0].start_motion(current_time);
        robot->arm[1].start_motion(current_time);
        current_step ++;
        ROS_INFO("moving to step %i",current_step);
      }
      break;
    }
    case 34:
    {
      ROS_INFO("Did the graspers clap several times for around %i seconds each direction? (Y/N)",duration);
      current_step++;
      break;
    }
    case 35:
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


// 3-1 Absolute (command: servo_cp) Axis motion Test
// (functionality) move along X axis for 2 cm (both arms)
//    Pass: Check raven state
// (functionality) move along Y axis for 2 cm (both arms)
//    Pass: Check raven state
// (functionality) move along Z axis for 2 cm (both arms)
//    Pass: Check raven state
int test_3_1(CRTK_robot *robot, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  int out = 0;
  std::string start, s1,s2;
  float pos_thresh = 10 DEG_TO_RAD;
  float vel_thresh = 10 DEG_TO_RAD;
  float dist = 0.035; // 20 mm total
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
      else if(current_time - pause_start > 2){
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
