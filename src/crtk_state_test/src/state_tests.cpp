#include "state_tests.h"
#include "ros/ros.h"
#include <crtk_msgs/robot_state.h>
#include <crtk_msgs/robot_command.h>
#include "crtk_robot_state.h"
#include "raven.h"
#include <sstream>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;

int state_testing(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static time_t start_time = current_time;
  static int current_test = 0;
  static int finished = 0;
  static int errors = 0;
  int test_status;
  int num_of_tests = 8;


  // wait for crtk state message to be published before testing
  if (current_test == 0 && robot_state.get_connected() == 1) {
    current_test = 1;
  }

  // wait for a beat
  if (current_time - start_time < 2){
        return 0;
  }
  else if (current_test == 0 && !finished){
    ROS_INFO("Robot not connected.");
  }

  // don't do anything after all tests have finished
  if (finished) {
    return 0;
  }

  // start testing!!
  switch(current_test){
    case 1:
    {
      // I.    {disabled, ~homed} + enable [prompt for button press] → {enabled / init}
      test_status = test_1(raven,robot_state, current_time);
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
        // II.    {disabled, homed} + enable [prompt for button press] → {enabled / p_dn}
        // IV-2.  {enabled, moving} + pause → {paused / p_up}
        test_status = test_2(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_2 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_2 passed: %i", test_status);
        }
        break;
    }

    case 3:
    {
        // VI-2.    {paused, p_up} + disable → {disabled / e-stop}
        // VIII-2.    {disabled, homed} + unhome → {disabled, ~homed / e-stop} 
        test_status = test_3(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_3 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_3 passed: %i", test_status);
        }
        break;
    }

    case 4:
    {
        // IV-1.    {enabled, homing} + pause → {disabled / e-stop}
        // VIII-1.  {disabled, ~homed} + unhome → {disabled, ~homed / e-stop}
        test_status = test_4(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_4 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_4 passed: %i", test_status);
        }
        break;
    }

    case 5:
    {
        // III-1.    {enabled, homing} + disable → {disabled / e-stop}
        // III-2.    {enabled, moving} + disable → {disabled / e-stop}

        test_status = test_5(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_5 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_5 passed: %i", test_status);
        }
        break;
    }

    case 6:
    {
        // VIII-3.    {enabled, homing} + unhome → {disabled, ~homed / e-stop}
        // VIII-4.    {enabled, moving} + unhome → {disabled, ~homed / e-stop}

        test_status = test_6(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_6 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_6 passed: %i", test_status);
        }
        break;
    }


    case 7:
    {
        // VIII-6.    {paused, homed} + unhome → {disabled, ~homed / e-stop}

        test_status = test_7(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_7 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_7 passed: %i", test_status);
        }
        break;
    }

    case 8:
    {
        // V-3.    {disabled, ~homed} + home [prompt for button press] → {enabled, homing / init}
        // V-1.    {enabled, homed} + home [prompt for button press] → {enabled, homing / init}
        // V-2.    {paused, homed} + home [prompt for button press] → {enabled, homing / init}
        test_status = test_8(raven,robot_state, current_time);
        if (test_status < 0) {
          errors += 1;
          current_test ++;
          ROS_ERROR("test_8 fail: %i", test_status);
        }
        else if (test_status > 0) {
          current_test ++;
          ROS_INFO("test_8 passed: %i", test_status);
        }
        break;
    }

    default:
    {
      if (finished == 0 && errors != 0){
        // After all tests, send estop command!
        CRTK_robot_command command = CRTK_DISABLE;
        robot_state.crtk_command_pb(command);
        ROS_ERROR("We failed some things.");
        finished = 1;
      }
      else if(finished == 0 && errors == 0){
        // After all tests, send estop command!
        CRTK_robot_command command = CRTK_DISABLE;
        robot_state.crtk_command_pb(command);
        ROS_INFO("We finished everything. Good job!!");
        finished = 1;
      }
    }

  }

  if(current_test > num_of_tests && errors == 0) {
    ROS_INFO("State testing success!!!");
  }

  return errors;
}


// I.    {disabled, ~homed} + enable [prompt for button press] → {enabled / init}
int test_1(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;

  switch(current_step)
  {
    case 1:
    {
      ROS_INFO("======================= Starting test_1 ======================= ");
      // (1) check disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -1;
      }
      break;
    }

    case 2:
    {
      // (2) check ~homed
      if (!robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -2;
      }
      break;
    }

    case 3:
    {
      // (3) send enable command
      CRTK_robot_command command = CRTK_ENABLE;
      robot_state.crtk_command_pb(command);
      current_step ++;
      break;
    }

    case 4:
    {
      // (4) prompt button press
      // ROS_INFO("test_1 (step 4): Please press silver button!");
      pause_start = current_time;
      current_step ++;
      break;
    }

    case 5:
    {
      // (5) wait for a bit
      if (current_time - pause_start >= 6){
        current_step ++;
      }
      break;
    }

    case 6:
    {
      // (6) check if crtk == enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -6;
      }
      break;
    }

    case 7:
    {
      // (7) check if raven == init
      if (raven.get_runlevel() == RL_INIT){
        current_step ++;
        return 1; // all steps passed
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}

// II.    {paused, homed} + resume [prompt for button press] → {enabled / p_dn}
// IV-2.  {enabled, moving} + pause → {paused / p_up} (starting at case 7)
int test_2(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";
  static int start_flag = 0;

  switch(current_step)
  {
    case 0:
    {
      if(!start_flag){
        ROS_INFO("======================= Starting test_2 ======================= ");
        ROS_INFO("Please home Raven and press 'Enter'.");
        start_flag = 1;
      }
      // cin.clear();
      getline(std::cin,start);
      if(start == ""){
        current_step = 1;
      }
      break;
    }

    case 1:
    {
      // (1) check paused
      if (robot_state.get_paused()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -1;
      }
      break;
    }

    case 2:
    {
      // (2) check homed
      if (robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -2;
      }
      break;
    }

    case 3:
    {
      // (3) send resume command
      CRTK_robot_command command = CRTK_RESUME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }

    case 4:
    {
      // (4) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }

    case 5:
    {
      // (5) check if crtk == enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -5;
      }
      break;
    }

    case 6:
    {
      // (6) check if raven == p_dn
      if (raven.get_runlevel() == RL_PEDAL_DN){
        current_step ++;
      }
      else{
        current_step = -100;
        return -6;
      }
      break;
    }

    case 7:
    {
      // (7) send pause command
      CRTK_robot_command command = CRTK_PAUSE;
      robot_state.crtk_command_pb(command);
      current_step ++;
      break;
    }

    case 8:
    {
      // (8) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }

    case 9:
    {
      // (9) check if crtk == paused
      if (robot_state.get_paused()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -9;
      }
      break;
    }

    case 10:
    {
      // (10) check if raven == p_up
      if (raven.get_runlevel() == RL_PEDAL_UP){
        current_step ++;
        return 1; // all steps passed
      }
      else{
        current_step = -100;
        return -10;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}


// VI-2.    {paused, p_up} + disable → {disabled / e-stop}
// VIII-2.    {disabled, homed} + unhome → {disabled, ~homed / e-stop} (starting at case 8)
int test_3(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 1;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    case 1:
    {
      // (1) check paused
      ROS_INFO("======================= Starting test_3 ======================= ");
      if (robot_state.get_paused()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -1;
      }
      break;
    }

    case 2:
    {
      // (2) check pedal_up
      if (raven.get_runlevel() == RL_PEDAL_UP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -2;
      }
      break;
    }

    case 3:
    {
      // (3) send disable command
      CRTK_robot_command command = CRTK_DISABLE;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }

    case 4:
    {
      // (4) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }

    case 5:
    {
      // (5) check if crtk == disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -5;
      }
      break;
    }

    case 6:
    {
      // (6) check if raven == estop
      if (raven.get_runlevel() == RL_E_STOP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -6;
      }
      break;
    }

    case 7:
    {
      // (7) check if crtk == is_homed
      if (robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }

    case 8:
    {
      // (8) send unhome command
      CRTK_robot_command command = CRTK_UNHOME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      break;
    }

    case 9:
    {
      // (9) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }

    case 10:
    {
      // (10) check if raven == unhomed
      if (!raven.get_robot_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -10;
      }
      break;
    }
    case 11:
    {
      // (11) check if crtk == !is_homed
      if (!robot_state.get_homed()){
        current_step ++;
        return 1; // all steps passed
      }
      else{
        current_step = -100;
        return -11;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}


// IV-1.    {enabled, homing} + pause → {disabled / e-stop}
// VIII-1.  {disabled, ~homed} + unhome → {disabled, ~homed / e-stop}
int test_4(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    case 0:
    {
      ROS_INFO("======================= Starting test_4 ======================= ");
      ROS_INFO("Please terminate the robot software and restart it.");
      ROS_INFO("Press 'Enter' when you're done!");
      current_step ++;
      break;
    }
    case 1:
    {
      // (1) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 2:
    {
      // (2) wait for initialization
      CRTK_robot_command command = CRTK_ENABLE;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Now press the silver button to initialize.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 3:
    {
      // (3) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -3;
      }
      break;
    }
    case 4:
    {
      // (4) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 5:
    {
      // (5) send pause command
      CRTK_robot_command command = CRTK_PAUSE;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 6:
    {
      // (6) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }
    case 7:
    {
      // (7) check estop
      if (raven.get_runlevel() == RL_E_STOP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }
    case 8:
    {
      // (8) check if crtk == disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -8;
      }
      break;
    }
    case 9:
    {
      // (9) check if crtk == not is_homed
      if (!robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -9;
      }
      break;
    }

    case 10:
    {
      // (10) send unhome command
      CRTK_robot_command command = CRTK_UNHOME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      break;
    }

    case 11:
    {
      // (11) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }

    case 12:
    {
      // (12) check if raven == unhomed
      if (!raven.get_robot_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -12;
      }
      break;
    }
    case 13:
    {
      // (13) check if crtk == !is_homed
      if (!robot_state.get_homed()){
        current_step ++;
        return 1; // all steps passed
      }
      else{
        current_step = -100;
        return -13;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}


// III-1.    {enabled, homing} + disable → {disabled / e-stop}
// III-2.    {enabled, moving} + disable → {disabled / e-stop}
int test_5(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    case 0:
    {
      ROS_INFO("======================= Starting test_5 ======================= ");
      ROS_INFO("Please terminate the robot software and restart it.");
      ROS_INFO("Press 'Enter' when you're done!");
      current_step ++;
      break;
    }
    case 1:
    {
      // (1) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 2:
    {
      // (2) wait for initialization
      CRTK_robot_command command = CRTK_ENABLE;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Now press the silver button to initialize.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 3:
    {
      // (3) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -3;
      }
      break;
    }
    case 4:
    {
      // (4) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 5:
    {
      // (5) send disable command
      CRTK_robot_command command = CRTK_DISABLE;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 6:
    {
      // (6) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }
    case 7:
    {
      // (7) check estop
      if (raven.get_runlevel() == RL_E_STOP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }
    case 8:
    {
      // (8) check if crtk == disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -8;
      }
      break;
    }
    case 9:
    {
      // (9) check if crtk == not is_homed
      if (!robot_state.get_homed()){
        current_step ++;
        return 1;
      }
      else{
        current_step = -100;
        return -9;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}

// VIII-3.    {enabled, homing} + unhome → {disabled, ~homed / e-stop}
// VIII-4.    {enabled, moving} + unhome → {disabled, ~homed / e-stop}
int test_6(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    case 0:
    {
      ROS_INFO("======================= Starting test_6 ======================= ");
      ROS_INFO("Please terminate the robot software and restart it.");
      ROS_INFO("Press 'Enter' when you're done!");
      current_step ++;
      break;
    }
    case 1:
    {
      // (1) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 2:
    {
      // (2) wait for initialization
      CRTK_robot_command command = CRTK_ENABLE;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Now press the silver button to initialize.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 3:
    {
      // (3) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -3;
      }
      break;
    }
    case 4:
    {
      // (4) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 5:
    {
      // (5) send disable command
      CRTK_robot_command command = CRTK_UNHOME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 6:
    {
      // (6) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }
    case 7:
    {
      // (7) check estop
      if (raven.get_runlevel() == RL_E_STOP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }
    case 8:
    {
      // (8) check if crtk == disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -8;
      }
      break;
    }
    case 9:
    {
      // (9) check if crtk == not is_homed
      if (!robot_state.get_homed()){
        current_step ++;
        return 1;
      }
      else{
        current_step = -100;
        return -9;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}

// VIII-6.    {paused, homed} + unhome → {disabled, ~homed / e-stop}
int test_7(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    case 0:
    {
      ROS_INFO("======================= Starting test_7 ======================= ");
      ROS_INFO("Please terminate the robot software and restart it.");
      ROS_INFO("Press 'Enter' when you're done!");
      current_step ++;
      break;
    }
    case 1:
    {
      // (1) wait for 'Enter' key press
      getline(std::cin,start);
      if(start == ""){
        current_step ++;
      }
      break;
    }
    case 2:
    {
      // (2) wait for initialization
      CRTK_robot_command command = CRTK_ENABLE;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Now press the silver button to initialize.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 3:
    {
      // (3) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -3;
        return -3;
      }
      break;
    }
    case 4:
    {
      // (4) wait for robot to finish initializing
      if (robot_state.get_paused()){
        current_step ++;
        ROS_INFO("Detected completion of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 30){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -4;
      }
      break;
    }
    case 5:
    {
      // (5) send disable command
      CRTK_robot_command command = CRTK_UNHOME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 6:
    {
      // (6) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }
    case 7:
    {
      // (7) check estop
      if (raven.get_runlevel() == RL_E_STOP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }
    case 8:
    {
      // (8) check if crtk == disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -8;
      }
      break;
    }
    case 9:
    {
      // (9) check if crtk == not is_homed
      if (!robot_state.get_homed()){
        current_step ++;
        return 1;
      }
      else{
        current_step = -100;
        return -9;
      }
      break;
    }

    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}


// V-3.    {disabled, ~homed} + home [prompt for button press] → {enabled, homing / init}
// V-2.    {paused, homed} + home [prompt for button press] → {enabled, homing / init}
// V-1.    {enabled, homed} + home [prompt for button press] → {enabled, homing / init}
int test_8(raven raven, CRTK_robot_state robot_state, time_t current_time){
  static int current_step = 0;
  static time_t pause_start;
  string start = "0";

  switch(current_step)
  {
    // for V-3
    case 0:
    {
      ROS_INFO("======================= Starting test_8 ======================= ");
      current_step ++;
      break;
    }
    case 1:
    {
      // (1) check crtk disabled
      if (robot_state.get_disabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -1;
      }
      break;
    }
    case 2:
    {
      // (2) check crtk unhomed
      if (!robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -2;
      }
      break;
    }
    case 3:
    {
      // (3) send home command
      CRTK_robot_command command = CRTK_HOME;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Press and release E-stop. Then press the silver button.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 4:
    {
      // (4) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -4;
        return -4;
      }
      break;
    }
    case 5:
    {
      // (5) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 6:
    {
      // (6) check init
      if (raven.get_runlevel() == RL_INIT){
        current_step ++;
      }
      else{
        current_step = -100;
        return -6;
      }
      break;
    }
    case 7:
    {
      // (7) check crtk enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -7;
      }
      break;
    }
    case 8:
    {
      // (8) check crtk homing
      if (robot_state.get_homing()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -8;
      }
      break;
    }
    case 9:
    {
      // (9) wait for robot to finish initializing
      if (robot_state.get_paused()){
        current_step ++;
        ROS_INFO("Detected completion of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 30){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -9;
      }
      break;
    }
    case 10:
    {
      // (10) check p_up
      if (raven.get_runlevel() == RL_PEDAL_UP){
        current_step ++;
      }
      else{
        current_step = -100;
        return -10;
      }
      break;
    }
    case 11:
    {
      // (11) check if crtk == paused
      if (robot_state.get_paused()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -11;
      }
      break;
    }
    case 12:
    {
      // (12) check if crtk == is_homed
      if (robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -12;
      }
      break;
    }

    // start testing V-2
    case 13:
    {
      // (13) send home command
      CRTK_robot_command command = CRTK_HOME;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Press and release E-stop. Then press the silver button.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 14:
    {
      // (14) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -14;
        return -14;
      }
      break;
    }
    case 15:
    {
      // (15) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 16:
    {
      // (16) check init
      if (raven.get_runlevel() == RL_INIT){
        current_step ++;
      }
      else{
        current_step = -100;
        return -16;
      }
      break;
    }
    case 17:
    {
      // (17) check crtk enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -17;
      }
      break;
    }
    case 18:
    {
      // (18) check crtk homing
      if (robot_state.get_homing()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -18;
      }
      break;
    }
    case 19:
    {
      // (19) wait for robot to finish initializing
      if (robot_state.get_paused()){
        current_step ++;
        ROS_INFO("Detected completion of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 30){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -19;
      }
      break;
    }
    case 20:
    {
      // (20) check crtk homing
      if (robot_state.get_homed()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -20;
      }
      break;
    }


    // start testing V-1
    case 21:
    {
      // (21) send resume command
      CRTK_robot_command command = CRTK_RESUME;
      robot_state.crtk_command_pb(command);
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 22:
    {
      // (22) wait for a bit
      if (current_time - pause_start >= 1){
        current_step ++;
      }
      break;
    }
    case 23:
    {
      // (23) check crtk enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -23;
      }
      break;
    }
    case 24:
    {
      // (24) send home command
      CRTK_robot_command command = CRTK_HOME;
      robot_state.crtk_command_pb(command);
      ROS_INFO("Press and release E-stop. Then press the silver button.");
      current_step ++;
      pause_start = current_time;
      break;
    }
    case 25:
    {
      // (14) wait for robot to start initializing
      if (robot_state.get_homing()){
        current_step ++;
        ROS_INFO("Detected start of robot homing.");
        pause_start = current_time;
      }
      else if(current_time - pause_start > 10){
        ROS_ERROR("Testing timeout...");
        current_step = -25;
        return -25;
      }
      break;
    }
    case 26:
    {
      // (26) wait for a bit
      if (current_time - pause_start >= 3){
        current_step ++;
      }
      break;
    }
    case 27:
    {
      // (27) check init
      if (raven.get_runlevel() == RL_INIT){
        current_step ++;
      }
      else{
        current_step = -100;
        return -27;
      }
      break;
    }
    case 28:
    {
      // (28) check crtk enabled
      if (robot_state.get_enabled()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -28;
      }
      break;
    }
    case 29:
    {
      // (29) check crtk homing
      if (robot_state.get_homing()){
        current_step ++;
      }
      else{
        current_step = -100;
        return -29;
      }
      break;
    }
    case 30:
    {
      // (30) wait for robot to finish initializing
      if (robot_state.get_paused()){
        current_step ++;
        ROS_INFO("Detected completion of robot homing.");
        pause_start = current_time;
        return 1;
      }
      else if(current_time - pause_start > 30){
        ROS_ERROR("Testing timeout...");
        current_step = -100;
        return -30;
      }
      break;
    }


    default:
    {
      return -100;
      break;
    }

  }
  return 0;
}