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
 * crtk_motion.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 29, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */



#ifndef CRTK_MOTION_H_
#define CRTK_MOTION_H_
#include "defines.h"
#include <ctime>
#include "ros/ros.h"
#include "tf/tf.h"
#include <crtk_msgs/operating_state.h>

// extern const int MAX_JOINTS;

class CRTK_motion{
public:
  CRTK_motion();
  ~CRTK_motion(){};
  tf::Transform get_measured_cp();
  void set_measured_cp(tf::Transform);

  float get_measured_js_pos(int);
  int get_measured_js_pos(float*, int);
  int set_measured_js_pos(int, float);
  int set_measured_js_pos(float*, int);

  float get_measured_js_vel(int);
  int get_measured_js_vel(float*, int);
  int set_measured_js_vel(int, float);
  int set_measured_js_vel(float*, int);

  float get_measured_js_eff(int);
  int get_measured_js_eff(float*, int);
  int set_measured_js_eff(int, float);
  int set_measured_js_eff(float*, int);

  char send_servo_cr_time(tf::Vector3,float,float,time_t);
  char send_servo_cr_rot_time(tf::Vector3,float,float,time_t);
  char send_servo_cr(tf::Transform);
  char send_servo_jr_grasp(float);
  void reset_servo_cr_updated();
  void reset_servo_jr_grasp_updated();
  char get_servo_jr_grasp_updated();
  char get_servo_cr_updated();
  tf::Transform get_servo_cr_command();
  float get_servo_jr_grasp_command();

  time_t get_start_time();

  char start_motion( time_t curr_time);

private:
  tf::Transform measured_cp;
  tf::Transform measured_cv;
  tf::Transform measured_cf; // Not supported by Raven
  tf::Transform goal_cp;
  tf::Transform setpoint_cp;
  float measured_js_pos[MAX_JOINTS];
  float measured_js_vel[MAX_JOINTS];
  float measured_js_eff[MAX_JOINTS];

  tf::Transform servo_cr_command;
  float servo_jr_grasp_command;
  char servo_cr_updated;
  char servo_jr_grasp_updated;

  time_t motion_start_time;
};

#endif
