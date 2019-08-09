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

  // get params from cfg file

  #ifdef RAVEN
  arm.set_home_pos(raven_gold_home_rot, raven_gold_home_pos);

  arm.set_home_jpos(raven_gold_home_jpos, MAX_JOINTS);

  #endif
}



/**
 * @brief      Initialize ROS: publisher/subscriber relation setup
 *
 * @param[in]  n     ROS node handler
 * @return           success
 */
bool CRTK_robot::init_ros(ros::NodeHandle n){
  
  std::string topic;
  double tmp_max_joints;

  // Read ROS Parameter Values from yaml file
  if(!n.getParam("/robot_namespace", robot_name))
    ROS_ERROR("Cannot read robot_namespace from the param.yaml file.");

  if(!n.getParam("/arm_namespace", arm_name))
    ROS_ERROR("Cannot read arm_namespace from the param.yaml file.");

  if(!n.getParam("/grasper_namespace", grasper_name))
    ROS_ERROR("Cannot read grasper_namespace from the param.yaml file.");

  if(!n.getParam("/max_joints", tmp_max_joints))
    ROS_ERROR("Cannot read max_joints from the param.yaml file.");
  max_joints = (unsigned int) tmp_max_joints;

  // Set Publisher and Subscribers under the namespace from from the parameter list
  topic = arm_name + "measured_cp";
  sub_measured_cp = n.subscribe(topic, 1, &CRTK_robot::crtk_measured_cp_arm_cb,this);

  topic = arm_name + "measured_js";
  sub_measured_js = n.subscribe(topic, 1, &CRTK_robot::crtk_measured_js_arm_cb,this);

  topic = arm_name + "servo_cr";
  pub_servo_cr = n.advertise<geometry_msgs::TransformStamped>(topic, 1);

  topic = arm_name + "servo_cp";
  pub_servo_cp = n.advertise<geometry_msgs::TransformStamped>(topic, 1);

  topic = arm_name + "servo_jr";
  pub_servo_jr = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = arm_name + "servo_jp";
  pub_servo_jp = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = grasper_name + "servo_jr";
  pub_servo_jr_grasp = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = grasper_name + "servo_jp";
  pub_servo_jp_grasp = n.advertise<sensor_msgs::JointState>(topic, 1);

  return true;


}



/**
 * @brief      Sets the state.
 *
 * @param      new_state  The new robot state class object
 */
void CRTK_robot::set_state(CRTK_robot_state *new_state){
  state = *new_state;
}



/**
 * @brief      arm1 callback function for measured_cp
 *
 * @param[in]  msg   The message
 */
void CRTK_robot::crtk_measured_cp_arm_cb(geometry_msgs::TransformStamped msg){
  tf::Transform in;
  tf::transformMsgToTF(msg.transform, in);
  arm.set_measured_cp(in);

}


/**
 * @brief      arm1 callback function for measured_js
 *
 * @param[in]  msg   The message
 */
void CRTK_robot::crtk_measured_js_arm_cb(sensor_msgs::JointState msg){

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

  arm.set_measured_js_pos(tmp_pos,MAX_JOINTS); 
  arm.set_measured_js_vel(tmp_vel,MAX_JOINTS); 
  arm.set_measured_js_eff(tmp_eff,MAX_JOINTS); 
}


/**
 * @brief      Checks all types of motion commands to publish to the robot.
 */
void CRTK_robot::check_motion_commands_to_publish(){

  if(arm.get_servo_cr_updated()){ 
    publish_servo_cr();
  }  

  else if(arm.get_servo_cp_updated()){ 
    publish_servo_cp();
  }  

  else if(arm.get_servo_jr_updated()){
    publish_servo_jr();
  }
  else if(arm.get_servo_jr_grasp_updated()){
    publish_servo_jr_grasp();
  }

  else if(arm.get_servo_jp_updated()){
    publish_servo_jp();
  }
  else if(arm.get_servo_jp_grasp_updated()){
    publish_servo_jp_grasp();
  }
}



/**
 * @brief      Initiate CRTK command publishing
 */
void CRTK_robot::run(){
  check_motion_commands_to_publish(); 
}



/**
 * @brief      publish servo_cr_command
 */
void CRTK_robot::publish_servo_cr(){
  geometry_msgs::TransformStamped msg;

  msg.header.stamp = msg.header.stamp.now();
  tf::Transform cmd = arm.get_servo_cr_command(); 
  tf::transformTFToMsg(cmd,msg.transform);

  pub_servo_cr.publish(msg);
  arm.reset_servo_cr_updated();

}



/**
 * @brief      publish servo_cp command
 */
void CRTK_robot::publish_servo_cp(){
  geometry_msgs::TransformStamped msg;

  msg.header.stamp = msg.header.stamp.now();
  tf::Transform cmd = arm.get_servo_cp_command(); 
  tf::transformTFToMsg(cmd,msg.transform);

  pub_servo_cp.publish(msg);
  arm.reset_servo_cp_updated();
}



/**
 * @brief      publish servo_jr grasper command
 */
void CRTK_robot::publish_servo_jr_grasp(){
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd = arm.get_servo_jr_grasp_command(); 
  msg.position.push_back(cmd);
  msg.name.push_back("grasp");

  pub_servo_jr_grasp.publish(msg);
  arm.reset_servo_jr_grasp_updated();
}



/**
 * @brief      publish servo_jr command
 */
void CRTK_robot::publish_servo_jr(){
  
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd[MAX_JOINTS];

  arm.get_servo_jr_command(cmd, MAX_JOINTS); 

  for(int j=0;j<MAX_JOINTS;j++)
    msg.position.push_back(cmd[j]);
    
    pub_servo_jr.publish(msg);
    arm.reset_servo_jr_updated();
}



/**
 * @brief      publish servo jp grasper command
 */
void CRTK_robot::publish_servo_jp_grasp(){
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd = arm.get_servo_jp_grasp_command(); 
  msg.position.push_back(cmd);
  msg.name.push_back("grasp");

  pub_servo_jp_grasp.publish(msg);
  arm.reset_servo_jp_grasp_updated();
}



/**
 * @brief      publish servo jp command
 */
void CRTK_robot::publish_servo_jp(){
  
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd[MAX_JOINTS];

  arm.get_servo_jp_command(cmd, MAX_JOINTS); 

  for(int j=0;j<MAX_JOINTS;j++)
    msg.position.push_back(cmd[j]);

  pub_servo_jp.publish(msg);
  arm.reset_servo_jp_updated();
}



/**
 * @brief      The main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0
 */
int main(int argc, char **argv){

  ros::init(argc, argv, "crtk_robot_library_wtf");  

  static ros::NodeHandle n; 
  ros::Rate loop_rate(LOOP_RATE); 

  CRTK_robot robot(n);

  int count = 0;

}