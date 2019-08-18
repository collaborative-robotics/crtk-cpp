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
 * crtk_robot.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 29, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#ifndef CRTK_ROBOT_H_
#define CRTK_ROBOT_H_
#include "defines.h"
#include "ros/ros.h"
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h> // catkin component

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <crtk_msgs/operating_state.h>
#include "crtk_robot_state.h"
#include "crtk_motion.h"

// Max DOF 
// extern const int MAX_JOINTS;

class CRTK_robot{

  public:
    CRTK_robot_state state;
    CRTK_motion arm;

    CRTK_robot(ros::NodeHandle n,std::string);
    ~CRTK_robot(){};
    bool init_param(ros::NodeHandle);
    bool init_ros(ros::NodeHandle);
    void crtk_measured_cp_arm_cb(geometry_msgs::TransformStamped);
    void crtk_measured_js_arm_cb(sensor_msgs::JointState);
    void set_state(CRTK_robot_state *new_state);

    void check_motion_commands_to_publish();
    void publish_servo_cr();
    void publish_servo_cv();
    void publish_servo_cp();
    void publish_servo_jr_grasp();
    void publish_servo_jr();
    void publish_servo_jp_grasp();
    void publish_servo_jp();
    void publish_servo_jv_grasp();
    void publish_servo_jv();
    unsigned int get_max_joints();
    void run();
  private:
    unsigned int max_joints; 
    std::string robot_name;
    std::string grasper_name;

    ros::Subscriber sub_measured_cp;
    ros::Subscriber sub_measured_js; 

    ros::Publisher pub_servo_cr;
    ros::Publisher pub_servo_cp;
    ros::Publisher pub_servo_cv;
    ros::Publisher pub_servo_jr;
    ros::Publisher pub_servo_jv;
    ros::Publisher pub_servo_jp;
    ros::Publisher pub_servo_jr_grasp;
    ros::Publisher pub_servo_jv_grasp;
    ros::Publisher pub_servo_jp_grasp;
};

#endif
