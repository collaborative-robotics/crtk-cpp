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
 * @param[in]  n           ros node handle
 * @param[in]  robot_ns    The robot namespace
 * @param[in]  grasper_ns  The grasper namespace
 */
CRTK_robot::CRTK_robot(ros::NodeHandle n, std::string robot_ns):state(n, robot_ns){

  robot_name = robot_ns;
  init_param(n);
  init_ros(n);  
}

/**
 * @brief      Initialize ROS parameter settings
 *
 * @param[in]  n     ROS node handler
 *
 * @return           success
 */
bool CRTK_robot::init_param(ros::NodeHandle n){

  char set_new_home_pos  = 0;
  char set_new_home_jpos = 0;
  char set_new_home_quat = 0;
  char set_is_prismatic  = 0;

  // Read ROS Parameter Values from yaml file
  if(!n.getParam("/"+robot_name+"/grasper_name", grasper_name))
    ROS_ERROR("Cannot read grasper_name from the %s's yaml file.", robot_name.c_str());
  else
    ROS_INFO("Robot namespace: %s, Grasper namespace: %s",robot_name.c_str(),grasper_name.c_str());


  double tmp_max_joints;
  if(!n.getParam("/"+robot_name+"/num_joints", tmp_max_joints))
    ROS_ERROR("Cannot read num_joints from the %s's yaml file.", robot_name.c_str());
  max_joints = (unsigned int) tmp_max_joints;

  float home_jpos[max_joints];
  char is_prismatic[max_joints];

  XmlRpc::XmlRpcValue tmp_home_pos;
  XmlRpc::XmlRpcValue tmp_home_jpos;
  XmlRpc::XmlRpcValue tmp_home_quat;
  XmlRpc::XmlRpcValue tmp_is_prismatic;
  n.getParam("/"+robot_name+"/home_pos", tmp_home_pos);
  n.getParam("/"+robot_name+"/home_jpos", tmp_home_jpos);
  n.getParam("/"+robot_name+"/home_quat", tmp_home_quat);
  n.getParam("/"+robot_name+"/is_prismatic", tmp_is_prismatic);

  ROS_ASSERT(tmp_home_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(tmp_home_jpos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(tmp_home_quat.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(tmp_is_prismatic.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if(tmp_home_pos.size()!=3) 
    ROS_ERROR("Wrong length for home_pos parameter. (desired 3, actual %d",tmp_home_pos.size());
  else
    set_new_home_pos = 1;

  if(tmp_home_jpos.size()!=max_joints) 
    ROS_ERROR("Wrong length for home_jpos parameter. (desired %d, actual %d",max_joints,tmp_home_jpos.size());
  else
    set_new_home_jpos = 1;

  if(tmp_home_quat.size()!=4) 
    ROS_ERROR("Wrong length for home_quat parameter. (desired 4, actual %d",tmp_home_quat.size());
  else
    set_new_home_quat = 1;

  if(tmp_is_prismatic.size()!=max_joints) 
    ROS_ERROR("Wrong length for is_prismatic parameter. (desired %d, actual %d",max_joints,tmp_is_prismatic.size());
  else
    set_is_prismatic = 1;

  for(int i=0; i<tmp_home_pos.size();i++)
    ROS_ASSERT(tmp_home_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  for(int i=0; i<tmp_home_quat.size();i++)
    ROS_ASSERT(tmp_home_quat[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  tf::Vector3 home_pos(tmp_home_pos[0],tmp_home_pos[1],tmp_home_pos[2]);
  tf::Quaternion home_quat(tmp_home_quat[0],tmp_home_quat[1],tmp_home_quat[2],tmp_home_quat[3]);
  
  for(int i=0; i<max_joints;i++)
  {
    ROS_ASSERT(tmp_home_jpos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    home_jpos[i] = (float)(double)tmp_home_jpos[i];

    ROS_ASSERT(tmp_is_prismatic[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    is_prismatic[i] = (char)(int)(double)tmp_is_prismatic[i];
  }

  if(set_new_home_pos && set_new_home_quat)
    arm.set_home_pos(home_quat, home_pos);

  if(set_new_home_jpos)
    arm.set_home_jpos(home_jpos, max_joints);

  if(set_is_prismatic)
    arm.set_prismatic_joints(is_prismatic, max_joints);

  ROS_INFO("All ROS parameters loaded.");
}


/**
 * @brief      Initialize ROS: publisher/subscriber relation setup
 *
 * @param[in]  n     ROS node handler
 * @return           success
 */
bool CRTK_robot::init_ros(ros::NodeHandle n){
  
  std::string topic;

  // Set Publisher and Subscribers under the namespace from from the parameter list
  topic = "/" + robot_name + "/measured_cp";
  sub_measured_cp = n.subscribe(topic, 1, &CRTK_robot::crtk_measured_cp_arm_cb,this);

  topic = "/" + robot_name + "/measured_js";
  sub_measured_js = n.subscribe(topic, 1, &CRTK_robot::crtk_measured_js_arm_cb,this);

  topic = "/" + robot_name + "/servo_cr";
  pub_servo_cr = n.advertise<geometry_msgs::TransformStamped>(topic, 1);

  topic = "/" + robot_name + "/servo_cp";
  pub_servo_cp = n.advertise<geometry_msgs::TransformStamped>(topic, 1);

  topic = "/" + robot_name + "/servo_cv";
  pub_servo_cv = n.advertise<geometry_msgs::TransformStamped>(topic, 1);

  topic = "/" + robot_name + "/servo_jr";
  pub_servo_jr = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = "/" + robot_name + "/servo_jv";
  pub_servo_jv = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = "/" + robot_name + "/servo_jp";
  pub_servo_jp = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = "/" + grasper_name + "/servo_jr";
  pub_servo_jr_grasp = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = "/" + grasper_name + "/servo_jv";
  pub_servo_jv_grasp = n.advertise<sensor_msgs::JointState>(topic, 1);

  topic = "/" + grasper_name + "/servo_jp";
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

  else if(arm.get_servo_cv_updated()){ 
    publish_servo_cv();
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

  else if(arm.get_servo_jv_updated()){
    publish_servo_jv();
  }
  else if(arm.get_servo_jv_grasp_updated()){
    publish_servo_jv_grasp();
  }
}

/**
 * @brief      Return the maximum joints of the robot
 */
unsigned int CRTK_robot::get_max_joints(){
  return max_joints;
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
 * @brief      publish servo_cv command
 */
void CRTK_robot::publish_servo_cv(){
  geometry_msgs::TransformStamped msg;

  msg.header.stamp = msg.header.stamp.now();
  tf::Transform cmd = arm.get_servo_cv_command(); 
  tf::transformTFToMsg(cmd,msg.transform);

  pub_servo_cv.publish(msg);
  arm.reset_servo_cv_updated();
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
 * @brief      publish servo_jv grasper command
 */
void CRTK_robot::publish_servo_jv_grasp(){
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd = arm.get_servo_jv_grasp_command(); 
  msg.velocity.push_back(cmd);
  msg.name.push_back("grasp");

  pub_servo_jv_grasp.publish(msg);
  arm.reset_servo_jv_grasp_updated();
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
 * @brief      publish servo_jv command
 */
void CRTK_robot::publish_servo_jv(){
  
  sensor_msgs::JointState msg;

  msg.header.stamp = msg.header.stamp.now();
  float cmd[MAX_JOINTS];

  arm.get_servo_jv_command(cmd, MAX_JOINTS); 

  for(int j=0;j<MAX_JOINTS;j++)
    msg.velocity.push_back(cmd[j]);
    
    pub_servo_jv.publish(msg);
    arm.reset_servo_jv_updated();
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

  static ros::NodeHandle n("~"); 
  ros::Rate loop_rate(LOOP_RATE); 

  std::string r_space;
  if(!n.getParam("r_space", r_space))
    ROS_ERROR("No Robot namespace provided in command line!");
  CRTK_robot robot(n,r_space);

  int count = 0;

}