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
 * crtk_motion.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 29, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#include "crtk_motion.h"

CRTK_motion::CRTK_motion(){
  servo_cr_updated = 0;
}

tf::Transform CRTK_motion::get_measured_cp(){
  return measured_cp;
}

void CRTK_motion::set_measured_cp(tf::Transform trans){
  measured_cp = tf::Transform(trans);
}

float CRTK_motion::get_measured_js_pos(int index){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Index out of range.");
    return -1;
  }
  return measured_js_pos[index];
}

int CRTK_motion::get_measured_js_pos(float out[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }

  for(int i=0; i<length; i++)
    out[i] = measured_js_pos[i];
  // static int count = 0;
  // count ++;
  // if(count% 25 == 1){
  //   ROS_INFO("out = %f, measured = %f",out[0],measured_js_pos[0]);  
  // }

  return 1;
}

int CRTK_motion::set_measured_js_pos(int index, float value){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Wrong index of joints.");
    return -1;
  }
  measured_js_pos[index] = value;
  return 1;
}

int CRTK_motion::set_measured_js_pos(float js_value[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }
  for(int i=0; i<length; i++)
    measured_js_pos[i] = js_value[i];

  return 1;
}

//vel
float CRTK_motion::get_measured_js_vel(int index){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Index out of range.");
    return -1;
  }
  return measured_js_vel[index];
}

int CRTK_motion::get_measured_js_vel(float out[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }

  for(int i=0; i<length; i++)
    out[i] = measured_js_vel[i];

  return 1;
}

int CRTK_motion::set_measured_js_vel(int index, float value){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Wrong index of joints.");
    return -1;
  }
  measured_js_vel[index] = value;
  return 1;
}

int CRTK_motion::set_measured_js_vel(float js_value[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }
  for(int i=0; i<length; i++)
    measured_js_vel[i] = js_value[i];

  return 1;
}
//eff
float CRTK_motion::get_measured_js_eff(int index){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Index out of range.");
    return -1;
  }
  return measured_js_eff[index];
}

int CRTK_motion::get_measured_js_eff(float out[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }

  for(int i=0; i<length; i++)
    out[i] = measured_js_eff[i];

  return 1;
}

int CRTK_motion::set_measured_js_eff(int index, float value){
  if(index<0 || index>MAX_JOINTS){
    ROS_ERROR("Wrong index of joints.");
    return -1;
  }
  measured_js_eff[index] = value;
  return 1;
}

int CRTK_motion::set_measured_js_eff(float js_value[MAX_JOINTS], int length){
  if(length > MAX_JOINTS){
    ROS_ERROR("Wrong number of joints.");
    return -1;
  }
  for(int i=0; i<length; i++)
    measured_js_eff[i] = js_value[i];

  return 1;
}

/**
 * @brief      Starts a motion.
 *
 * @param[in]  curr_time  The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::start_motion( time_t curr_time){
  motion_start_time = curr_time;
}

/**
 * @brief      Sends a servo carriage return time. (Must call start_motion function first)
 *
 * @param[in]  vec         The vector
 * @param[in]  total_dist  The total distance
 * @param[in]  duration    The duration
 * @param[in]  curr_time   The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::send_servo_cr_time(tf::Vector3 vec, float total_dist, float duration, time_t curr_time){
  // static char start = 1;
  char out=0;
  float step = total_dist/(duration*LOOP_RATE);

  if(duration <= 0 || total_dist <= 0){
    ROS_ERROR("Duration and distance should be positive.");
    return -1;    
  }
  if(step > 0.001){
    ROS_ERROR("Step size is too big.");
    return -1;
  }

  // send command
  if(!vec.normalized()){
    vec = vec.normalize();
    ROS_INFO("Servo_cr direction not normalized. (set to normalized)");
  }
  out = send_servo_cr(tf::Transform(tf::Quaternion(0,0,0,0),vec*step));

  // check time
  if(curr_time - motion_start_time > duration){
    ROS_INFO("%f sec movement complete.",duration);
    
    //at end of time, send 0 command
    tf::Transform ident = tf::Transform(tf::Quaternion(0,0,0,0));
    // ident.setIdentity();
    out = send_servo_cr(ident);
    return 1;
  }
  return out;
}  

/**
 * @brief      Sends a servo carriage return time. (Must call start_motion
 *             function first)
 *
 * @param[in]  vec          The rotation vector
 * @param[in]  total_angle  The total angle (radian)
 * @param[in]  duration     The duration
 * @param[in]  curr_time    The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::send_servo_cr_rot_time(tf::Vector3 vec, float total_angle, float duration, time_t curr_time){
  // static char start = 1;
  char out=0;
  float step = total_angle/(duration*LOOP_RATE);

  if(duration <= 0){
    ROS_ERROR("Duration should be positive.");
    return -1;    
  }
  if(step > 0.001){
    ROS_ERROR("Step size is too big.");
    return -1;
  }

  // send command
  if(!vec.normalized()){
    vec = vec.normalize();
    ROS_INFO("Servo_cr rot direction not normalized. (set to normalized)");
  }

  tf::Quaternion out_qua = tf::Quaternion(vec,step);
  out = send_servo_cr(tf::Transform(out_qua));
  // ROS_INFO("Send quaternion:%f,%f,%f,%f step %f", out_qua.x(),out_qua.y(),out_qua.z(),out_qua.w(),step);

  // check time
  if(curr_time - motion_start_time > duration){
    ROS_INFO("%f sec movement complete.",duration);
    
    //at end of time, send 0 command
    tf::Transform ident = tf::Transform(tf::Quaternion(0,0,0,0));
    // ident.setIdentity();
    out = send_servo_cr(ident);
    return 1;
  }
  return out;
}  

char CRTK_motion::send_servo_cr(tf::Transform trans){
  // check command
  tf::Vector3 vec = trans.getOrigin();
  float ang = trans.getRotation().getAngle();
  if(vec.length() > STEP_TRANS_LIMIT || ang > STEP_ROT_LIMIT){
    ROS_ERROR("Servo_cr step limit exceeded. Motion not sent.");
    reset_servo_cr_updated();
    return -1;
  }
  // send command
  servo_cr_updated = 1;
  servo_cr_command = trans;

  return 0;
}

void CRTK_motion::reset_servo_cr_updated(){
  servo_cr_command = tf::Transform(tf::Quaternion(0,0,0,0),tf::Vector3(0,0,0));
  servo_cr_updated = 0;
}

char CRTK_motion::get_servo_cr_updated(){
  // servo_cr_updated = 1;
  // ROS_INFO("get says: %i", servo_cr_updated);
  return servo_cr_updated;
}

tf::Transform CRTK_motion::get_servo_cr_command(){
  return tf::Transform(servo_cr_command);
}