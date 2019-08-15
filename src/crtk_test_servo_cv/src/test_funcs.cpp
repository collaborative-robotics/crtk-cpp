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
 * 
 *
 *
 *  \date Aug 14, 2019
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#include "test_funcs.h"
#include <cmath>


/**
 * @brief      Checks if the robot transitioned to the desired state
 *
 * @param[in]  desired       The desired robot state
 * @param[in]  actual        The actual robot state
 * @param[in]  current_step  The current step
 *
 * @return     success 1, fail -1
 */
int crtk_state_check(CRTK_robot_state_enum desired, CRTK_robot_state_enum actual, int current_step){
  if(actual == desired){
    return 1;
  }else {
    return -1;
  }
}

/**
 * @brief      This function checks each robot joint to move beyond the pos and vel threshold
 *             assuming that we're testing MAX_JOINTS number of joints
 *
 * @param      robot         The robot class object
 * @param[in]  pos_thresh    The position thresh
 * @param[in]  vel_thresh    The velocity thresh
 * @param[in]  current_time  The current time
 * @param[in]  check_time    The expected time to finish the motion
 *
 * @return     success 1, fail -1
 */
int check_joint_motion_and_vel(CRTK_robot* robot, float pos_thresh, float vel_thresh, long current_time, int check_time){
  static int start = 1;
  double scale;
  static time_t start_time;
  static float start_pos[MAX_JOINTS];
  static float curr_pos[MAX_JOINTS],curr_vel[MAX_JOINTS]; 
  static int pos_done[MAX_JOINTS],vel_done[MAX_JOINTS];
  static float max_vel[MAX_JOINTS]; 

  if (start){
    robot->arm.get_measured_js_pos(start_pos, MAX_JOINTS);
    start_time = current_time;

    ROS_INFO(" pos_thresh = %f",fabs(pos_thresh));
    ROS_INFO(" vel_thresh = %f",fabs(vel_thresh));
    for(int i=0;i<MAX_JOINTS;i++){
      pos_done[i] = 0;
      vel_done[i] = 0;
      max_vel[i] = 0;
    }
    start = 0;
  }

  robot->arm.get_measured_js_pos(curr_pos, MAX_JOINTS);
  robot->arm.get_measured_js_vel(curr_vel, MAX_JOINTS);

  //check each joint for motion and velocity greater than threshold
  for(int i = 0; i<MAX_JOINTS; i++){
    if(i == 2)  scale = 0.1;
    else        scale = 1.0;

    if(fabs(start_pos[i] - curr_pos[i]) > fabs(pos_thresh*scale)) pos_done[i] = 1;
    if(fabs(curr_vel[i]) > vel_thresh*scale) vel_done[i] = 1;

    if(fabs(curr_vel[i]) >= fabs(max_vel[i])) max_vel[i] = curr_vel[i];
  }

 static int count = 0;
  if(count % 1500 == 0){
    ROS_INFO("(pos done)-- %i, %i, %i, %i, %i, %i, %i", pos_done[0],pos_done[1],pos_done[2], pos_done[3],pos_done[4],pos_done[5],pos_done[6]);
    ROS_INFO("(vel done)-- %i, %i, %i, %i, %i, %i, %i", vel_done[0],vel_done[1],vel_done[2], vel_done[3],vel_done[4],vel_done[5],vel_done[6]);
    ROS_INFO(" ");

  }
  count ++;

  if((done_sum(pos_done) == MAX_JOINTS) && (done_sum(vel_done) == MAX_JOINTS)){
    //success!

    ROS_INFO("max_vel-- %f, %f, %f, %f, %f, %f, %f", max_vel[0],max_vel[1],max_vel[2],max_vel[3],max_vel[4],max_vel[5],max_vel[6]);
    ROS_INFO("pos_done %i\tvel_done %i",done_sum(pos_done),done_sum(vel_done));
    ROS_INFO(" ");
    start = 1;
    return 1;
  }
  //if no, check time 
  else if(current_time - start_time > check_time){
    ROS_ERROR("Joint motion and velocity check timeout on step.");
    if (done_sum(pos_done) != MAX_JOINTS) ROS_INFO("robot arm didn't move far enough");
    if (done_sum(vel_done) != MAX_JOINTS) ROS_INFO("robot arm didn't move fast enough");


    ROS_INFO("max_vel-- %f, %f, %f, %f, %f, %f, %f", max_vel[0],max_vel[1],max_vel[2],max_vel[3],max_vel[4],max_vel[5],max_vel[6]);
    ROS_INFO("pos_done %i\tvel_done %i",done_sum(pos_done),done_sum(vel_done));
    ROS_INFO(" ");
    start = 1;
    return -1;
  }
  return 0;


}



/**
 * @brief      Checks robot completion status
 *
 * @param[in]  status        The completion status
 * @param      current_step  The current step
 *
 * @return     success > 0, fail otherwise
 */
int step_success(int status, int* current_step){
  int out = *current_step;
  if (status == -1){
    ROS_ERROR("step fail: %i,\tout: %i", *current_step, out);
    *current_step = -100;
    return -out;
  }
  else if (status == 1){
    ROS_INFO("step %i success.", *current_step);
    *current_step = *current_step + 1;
    return 1;
  }
  else if(status == 0){
    return 0;
  }
  else{
    ROS_ERROR("What does this mean?????");
    return -20;
  }
}



/**
 * @brief      This function checks if all joints of a robot pass the joint_motion_and_vel test
 *
 * @param      in    input completion flag array for all joints
 *
 * @return     The number of joints pass the test
 */
int done_sum(int in[MAX_JOINTS]){
  int done_sum = 0;
  for(int i=0; i<MAX_JOINTS;i++){
    done_sum += in[i];
  }
  return done_sum;
}


/**
 * @brief      Checks if the robot moved in the specified direction for a desired distance
 *             We are doing the check one arm at a time, not parallel
 *
 * @param      arm           The arm index
 * @param[in]  axis          The axis
 * @param[in]  dist          The distance
 * @param[in]  check_time    The check time
 * @param[in]  current_time  The current time
 *
 * @return     success > 0, fail otherwise
 */
int check_movement_direction(CRTK_motion* arm, CRTK_axis axis, float dist, int check_time, long current_time){
  static int start = 1;
  static float start_pos, max_dist;
  static time_t start_time;

  float curr_pos, curr_dist;

  if(dist == 0){
      ROS_ERROR("Zero distance specified.");
      return -1;
  }

  if(start){
    start_pos = axis_value(arm->get_measured_cp().getOrigin(), axis);
    start_time = current_time;
    start = 0;
    max_dist = 0;
  }

  curr_pos = axis_value(arm->get_measured_cp().getOrigin(), axis);
  curr_dist = curr_pos - start_pos;

  // save maxa distance
  if(fabs(curr_dist)>fabs(max_dist)){
    max_dist = curr_dist;
  }

  static int count = 0;
  count ++;
  if(count%500 == 0){
    ROS_INFO("curr_dist = %f",curr_dist);
    count = 0;
  }


  // check movement along axis
  if((dist > 0 && curr_dist > dist) || (dist < 0 && curr_dist < dist)){
    ROS_INFO("success!");
    start = 1;
    return 1;
  }

  // check for timeout
  if(current_time - start_time > check_time){
    ROS_ERROR("Check movement timeout.");
    start = 1;
    return -1;
  }
  return 0;
}


/**
 * @brief      Check for any rotation not around any particular axis
 *
 * @param      arm           The arm index
 * @param[in]  angle         The angle
 * @param[in]  check_time    The check time
 * @param[in]  current_time  The current time
 *
 * @return     success > 0, fail otherwise
 */
int check_movement_rotation(CRTK_motion* arm, float angle, int check_time, long current_time, tf::Transform start_pos){
  static int start = 1;
  static tf::Quaternion start_ori = start_pos.getRotation();
  static float max_angle;
  static time_t start_time;

  tf::Quaternion curr_ori;
  float curr_angle;

  if(angle == 0){
      ROS_ERROR("Zero angle specified.");
      return -1;
  }

  if(start){
    start_time = current_time;
    start = 0;
    max_angle = 0;
    ROS_INFO("start == 1");
  }

  curr_ori = arm->get_measured_cp().getRotation();

  ROS_INFO("start rotation (after): %f, %f, %f, %f",start_pos.getRotation().x(),start_pos.getRotation().y(),start_pos.getRotation().z(),start_pos.getRotation().w());
  ROS_INFO("curr rotation (after):  %f, %f, %f, %f",curr_ori.x(),curr_ori.y(),curr_ori.z(),curr_ori.w());
  curr_angle = fabs(2*curr_ori.angle(start_ori));
  ROS_INFO("angle rotated = %f",curr_angle);

  // save maxa distance
  if(fabs(curr_angle)>fabs(max_angle)){
    max_angle = curr_angle;
  }

  static int count = 0;
  count ++;
  if(count%500 == 0){
    ROS_INFO("angle rotated = %f",curr_angle);
    count = 0;
  }


  // check movement along axis
  if(fabs(curr_angle) > fabs(angle)){
    ROS_INFO("success!");
    start = 1;
    return 1;
  }

  // check for timeout
  if(current_time - start_time > check_time){
    ROS_ERROR("Check movement timeout.");
    start = 1;
    return -1;
  }
  return 0;
}



/**
 * @brief      returns the value of the "axis" entry of a Vector3
 *
 * @param[in]  vec   The vector
 * @param[in]  axis  The axis
 *
 * @return     the vector entry value
 */
float axis_value(tf::Vector3 vec, CRTK_axis axis){
  if(axis == CRTK_X){
    return vec.x();
  }
  else if(axis == CRTK_Y){
    return vec.y();
  }
  else if(axis == CRTK_Z){
    return vec.z();
  }
  else{
    ROS_ERROR("Unknown axis.");
    return 0;
  }
}



/**
 * @brief      Checks if the robot moves along the specified Cartesian direction for a desired distance
 *
 * @param      arm        The arm index
 * @param[in]  start_pos  The start position
 * @param[in]  axis       The axis
 * @param[in]  dist       The distance
 *
 * @return     success > 0, fail otherwise
 */
int check_movement_distance(CRTK_motion* arm,tf::Transform start_pos, CRTK_axis axis, float dist){
  tf::Transform curr_pos = arm->get_measured_cp();
  float start_val = axis_value(start_pos.getOrigin(),axis);
  float curr_val = axis_value(curr_pos.getOrigin(),axis);

  if ((curr_val - start_val)*dist <= 0){ // opposite direction
    ROS_INFO("Movement is in the wrong direction.\n(curr pos (%f) - start pos (%f) = %f (expected distance: %f)",
      curr_val, start_val, curr_val - start_val, dist);
    return -1;
  }  
  else if(fabs(curr_val - start_val) > fabs(1.2*dist)){
    ROS_INFO("Caution - moved too far.\n(curr pos (%f) - start pos (%f) = %f (expected distance: %f)",
      curr_val, start_val, curr_val - start_val, dist);
    return 1;    
  }
  else if(fabs(curr_val - start_val) > fabs(dist)){
    return 1;
  }
  else{
    ROS_INFO("Didn't move far enough. \n(curr pos (%f) - start pos (%f) = %f (expected distance: %f)",
      curr_val, start_val, curr_val - start_val, dist);
    return -1;
  }
}

enum cube_dir{cube_x, cube_y, cube_z};
char front = 0b100;
char left  = 0b010;
char lower = 0b001;



/**
 * @brief      Randomly chooses the next motion direction for the robot in cube tracing example
 *
 * @param      curr_vertex  The curr vertex
 * @param      move_vec     The move vector
 * @param      prev_axis    The previous axis
 *
 * @return     success
 */
char rand_cube_dir(char *curr_vertex, tf::Vector3 *move_vec, CRTK_axis *prev_axis){
  char choice = *prev_axis;
  while ((CRTK_axis)choice == *prev_axis){
    choice = std::rand() % 3; //random int 0-2
  }

  ROS_INFO("\t \t Randomly Picked %i", choice);

  switch((cube_dir)choice){
    case (cube_x):
    {
      ROS_INFO("Picked X!, %i", *curr_vertex);
      *prev_axis = CRTK_X ;

      if(*curr_vertex & front){
        *move_vec = tf::Vector3(1,0,0);
        *curr_vertex &= ~front;

      } else {
        *move_vec = tf::Vector3(-1,0,0);
        *curr_vertex |= front;
      }
      break;
    }    
    case (cube_y):
    {
      ROS_INFO("Picked Y!, %i", *curr_vertex);
      *prev_axis = CRTK_Y ;

      if(*curr_vertex & left){
        *move_vec = tf::Vector3(0,1,0);
        *curr_vertex &= ~left;

      } else {
        *move_vec = tf::Vector3(0,-1,0);
        *curr_vertex |= left;
      }
      break;
    }    
    case (cube_z):
    {
      ROS_INFO("Picked Z!, %i", *curr_vertex);
      *prev_axis = CRTK_Z;

      if(*curr_vertex & lower){
        *move_vec = tf::Vector3(0,0,1);
        *curr_vertex &= ~lower;

      } else {
        *move_vec = tf::Vector3(0,0,-1);
        *curr_vertex |= lower;
      }
      break;
    }
    default:
    {
      ROS_ERROR("unknown cube dir");
      break;
    }
  }
  return 1;
}