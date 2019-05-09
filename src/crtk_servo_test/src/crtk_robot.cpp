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

tf::Quaternion raven_gold_home_rot(-0.14,0.636,0.02,0.76);
tf::Quaternion raven_green_home_rot(0.12,0.62,0.07,0.77);
tf::Vector3 raven_gold_home_pos(0.0239,-0.0144,-0.0779);
tf::Vector3 raven_green_home_pos(0.0242, 0.0141,-0.0780);

float raven_gold_home_jpos[MAX_JOINTS] = {0.523, 1.584, 0.4002, 0.0715, 0.0950, 0.7537, 0.852};
float raven_green_home_jpos[MAX_JOINTS] = {0.524, 1.583, 0.400, 0.0660, 0.0598, 0.736, 0.844};



/**
 * @brief      Constructs the robot object.
 *
 * Starts the ROS interfaces and sets the home position for the specific robot.
 * 
 * @todo query the robot to find it's home position
 *
 * @param[in]  n     ros node handle
 */
CRTK_robot::CRTK_robot(ros::NodeHandle n):state(n){
  init_ros(n);

  #ifdef RAVEN
  arm[0].set_home_pos(raven_gold_home_rot, raven_gold_home_pos);
  arm[1].set_home_pos(raven_green_home_rot, raven_green_home_pos);
  arm[0].set_home_jpos(raven_gold_home_jpos, MAX_JOINTS);
  arm[1].set_home_jpos(raven_green_home_jpos, MAX_JOINTS);
  #endif
}

bool CRTK_robot::init_ros(ros::NodeHandle n){

  sub_measured_cp1 = n.subscribe("arm1/measured_cp", 1, &CRTK_robot::crtk_measured_cp_arm1_cb,this);
  sub_measured_cp2 = n.subscribe("arm2/measured_cp", 1, &CRTK_robot::crtk_measured_cp_arm2_cb,this);
  sub_measured_js1 = n.subscribe("arm1/measured_js", 1, &CRTK_robot::crtk_measured_js_arm1_cb,this);
  sub_measured_js2 = n.subscribe("arm2/measured_js", 1, &CRTK_robot::crtk_measured_js_arm2_cb,this);
  pub_servo_cr1 = n.advertise<geometry_msgs::TransformStamped>("arm1/servo_cr", 1);
  pub_servo_cr2 = n.advertise<geometry_msgs::TransformStamped>("arm2/servo_cr", 1);
  pub_servo_cp1 = n.advertise<geometry_msgs::TransformStamped>("arm1/servo_cp", 1);
  pub_servo_cp2 = n.advertise<geometry_msgs::TransformStamped>("arm2/servo_cp", 1);
  pub_servo_jr1 = n.advertise<sensor_msgs::JointState>("arm1/servo_jr", 1);
  pub_servo_jr2 = n.advertise<sensor_msgs::JointState>("arm2/servo_jr", 1); 
  pub_servo_jp1 = n.advertise<sensor_msgs::JointState>("arm1/servo_jp", 1);
  pub_servo_jp2 = n.advertise<sensor_msgs::JointState>("arm2/servo_jp", 1); 
  pub_servo_jr_grasp1 = n.advertise<sensor_msgs::JointState>("grasp1/servo_jr", 1);
  pub_servo_jr_grasp2 = n.advertise<sensor_msgs::JointState>("grasp2/servo_jr", 1);
  pub_servo_jp_grasp1 = n.advertise<sensor_msgs::JointState>("grasp1/servo_jp", 1);
  pub_servo_jp_grasp2 = n.advertise<sensor_msgs::JointState>("grasp2/servo_jp", 1);

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

    else if(arm[i].get_servo_cp_updated()){ // TODO: add more stuff 
      publish_servo_cp(i);
    }  

    else if(arm[i].get_servo_jr_updated()){
      publish_servo_jr(i);
    }
    else if(arm[i].get_servo_jr_grasp_updated()){
      // ROS_INFO("Trying to publish");
      publish_servo_jr_grasp(i);
    }

    else if(arm[i].get_servo_jp_updated()){
      publish_servo_jp(i);
    }
    else if(arm[i].get_servo_jp_grasp_updated()){
      // ROS_INFO("Trying to publish");
      publish_servo_jp_grasp(i);
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


void CRTK_robot::publish_servo_cp(char i){
  geometry_msgs::TransformStamped msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  tf::Transform cmd = arm[i].get_servo_cp_command(); 
  tf::transformTFToMsg(cmd,msg.transform);


  if(i == 0) {
    pub_servo_cp1.publish(msg);
    arm[i].reset_servo_cp_updated();
  }
  else if(i == 1){
    pub_servo_cp2.publish(msg);
    arm[i].reset_servo_cp_updated();
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


void CRTK_robot::publish_servo_jr(char i){
  
  sensor_msgs::JointState msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  float cmd[MAX_JOINTS];
  // static int ount;
  // if()

  arm[i].get_servo_jr_command(cmd, MAX_JOINTS); 

  for(int j=0;j<MAX_JOINTS;j++)
    msg.position.push_back(cmd[j]);

  if(i == 0) {
    pub_servo_jr1.publish(msg);
    arm[i].reset_servo_jr_updated();
  }
  else if(i == 1){
    pub_servo_jr2.publish(msg);
    arm[i].reset_servo_jr_updated();
  } 
}



/**
 * @brief      publish servo jp grasp command
 *
 * @param[in]  i     { parameter_description }
 */
void CRTK_robot::publish_servo_jp_grasp(char i){
  sensor_msgs::JointState msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  float cmd = arm[i].get_servo_jp_grasp_command(); 
  msg.position.push_back(cmd);
  msg.name.push_back("grasp");

  if(i == 0) {
    pub_servo_jp_grasp1.publish(msg);
    arm[i].reset_servo_jp_grasp_updated();
  }
  else if(i == 1){
    pub_servo_jp_grasp2.publish(msg);
    arm[i].reset_servo_jp_grasp_updated();
  } 
}



/**
 * @brief      publish servo jp command
 *
 * @param[in]  i     { parameter_description }
 */
void CRTK_robot::publish_servo_jp(char i){
  
  sensor_msgs::JointState msg;

  if(i != 0 && i != 1){
    ROS_ERROR("Invalid arm type for servo command.");
  }

  msg.header.stamp = msg.header.stamp.now();
  float cmd[MAX_JOINTS];
  // static int ount;
  // if()

  arm[i].get_servo_jp_command(cmd, MAX_JOINTS); 

  for(int j=0;j<MAX_JOINTS;j++)
    msg.position.push_back(cmd[j]);

  if(i == 0) {
    pub_servo_jp1.publish(msg);
    arm[i].reset_servo_jp_updated();
  }
  else if(i == 1){
    pub_servo_jp2.publish(msg);
    arm[i].reset_servo_jp_updated();
  } 
}