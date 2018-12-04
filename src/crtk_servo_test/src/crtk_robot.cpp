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
 * crtk_robot.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 29, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#include "crtk_robot.h"

CRTK_robot::CRTK_robot(ros::NodeHandle n):state(n){
  init_ros(n);
}

bool CRTK_robot::init_ros(ros::NodeHandle n){

  sub_measured_cp1 = n.subscribe("arm1/measured_cp", 1, &CRTK_robot::crtk_measured_cp_arm1_cb,this);
  sub_measured_cp2 = n.subscribe("arm2/measured_cp", 1, &CRTK_robot::crtk_measured_cp_arm2_cb,this);
  sub_measured_js1 = n.subscribe("arm1/measured_js", 1, &CRTK_robot::crtk_measured_js_arm1_cb,this);
  sub_measured_js2 = n.subscribe("arm2/measured_js", 1, &CRTK_robot::crtk_measured_js_arm2_cb,this);
  pub_servo_cr1 = n.advertise<geometry_msgs::TransformStamped>("arm1/servo_cr", 1);
  pub_servo_cr2 = n.advertise<geometry_msgs::TransformStamped>("arm2/servo_cr", 1);
  pub_servo_jr_grasp1 = n.advertise<sensor_msgs::JointState>("grasp1/servo_jr", 1);
  pub_servo_jr_grasp2 = n.advertise<sensor_msgs::JointState>("grasp2/servo_jr", 1);
  return true;
}

void CRTK_robot::set_state(CRTK_robot_state *new_state){
  state = *new_state;
}

void CRTK_robot::crtk_measured_cp_arm1_cb(geometry_msgs::TransformStamped msg){
  tf::Transform in;
  tf::transformMsgToTF(msg.transform, in);
  arm[0].set_measured_cp(in);

}

void CRTK_robot::crtk_measured_cp_arm2_cb(geometry_msgs::TransformStamped msg){
  tf::Transform in;
  tf::transformMsgToTF(msg.transform, in);
  arm[1].set_measured_cp(in);
}


void CRTK_robot::crtk_measured_js_arm1_cb(sensor_msgs::JointState msg){

  int size = msg.position.size();

  if(size>MAX_JOINTS){
    ROS_ERROR("Joint state size incorrect.");
  }

  float tmp_pos[MAX_JOINTS],tmp_vel[MAX_JOINTS],tmp_eff[MAX_JOINTS];

  for(int i=0;i<size ;i++){
    tmp_pos[i] = msg.position[i];
    tmp_vel[i] = msg.velocity[i];
    tmp_eff[i] = msg.effort[i];
  }

  arm[0].set_measured_js_pos(tmp_pos,MAX_JOINTS); 
  arm[0].set_measured_js_vel(tmp_vel,MAX_JOINTS); 
  arm[0].set_measured_js_eff(tmp_eff,MAX_JOINTS); 
}

void CRTK_robot::crtk_measured_js_arm2_cb(sensor_msgs::JointState msg){

  int size = msg.position.size();

  if(size>MAX_JOINTS){
    ROS_ERROR("Joint state size incorrect.");
  }

  float tmp_pos[MAX_JOINTS],tmp_vel[MAX_JOINTS],tmp_eff[MAX_JOINTS];

  for(int i=0;i<size ;i++){
    tmp_pos[i] = msg.position[i];
    tmp_vel[i] = msg.velocity[i];
    tmp_eff[i] = msg.effort[i];
  }

  arm[1].set_measured_js_pos(tmp_pos,MAX_JOINTS); 
  arm[1].set_measured_js_vel(tmp_vel,MAX_JOINTS); 
  arm[1].set_measured_js_eff(tmp_eff,MAX_JOINTS); 
}


void CRTK_robot::check_motion_commands_to_publish(){
  // TODO check robot state first and skip or warn
  // ROS_INFO("published something...");
  // ROS_INFO("Checking publish");
  for(int i=0;i<2;i++){
    if(arm[i].get_servo_cr_updated()){ // TODO: add more stuff 
      publish_servo_cr(i);
    }  

    if(arm[i].get_servo_jr_grasp_updated()){
      // ROS_INFO("Trying to publish");
      publish_servo_jr_grasp(i);
    }
  }
}

void CRTK_robot::run(){
  check_motion_commands_to_publish(); // TODO: add more stuff 
}

void CRTK_robot::publish_servo_cr(char i){
  geometry_msgs::TransformStamped msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  tf::Transform cmd = arm[i].get_servo_cr_command(); 
  tf::transformTFToMsg(cmd,msg.transform);


  if(i == 0) {
    pub_servo_cr1.publish(msg);
    arm[i].reset_servo_cr_updated();
  }
  else if(i == 1){
    pub_servo_cr2.publish(msg);
    arm[i].reset_servo_cr_updated();
  } 
}


void CRTK_robot::publish_servo_jr_grasp(char i){
  sensor_msgs::JointState msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  float cmd = arm[i].get_servo_jr_grasp_command(); 
  msg.position.push_back(cmd);
  msg.name.push_back("grasp");

  if(i == 0) {
    pub_servo_jr_grasp1.publish(msg);
    arm[i].reset_servo_jr_grasp_updated();
  }
  else if(i == 1){
    pub_servo_jr_grasp2.publish(msg);
    arm[i].reset_servo_jr_grasp_updated();
  } 
}