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
    CRTK_motion arm[2];

    //CRTK_robot(){};
    CRTK_robot(ros::NodeHandle n);
    ~CRTK_robot(){};
    bool init_ros(ros::NodeHandle);
    void crtk_measured_cp_arm1_cb(geometry_msgs::TransformStamped);
    void crtk_measured_cp_arm2_cb(geometry_msgs::TransformStamped);
    void crtk_measured_js_arm1_cb(sensor_msgs::JointState);
    void crtk_measured_js_arm2_cb(sensor_msgs::JointState);
    void set_state(CRTK_robot_state *new_state);

    void check_motion_commands_to_publish();
    void publish_servo_cr(char);
    void publish_servo_cp(char);
    void publish_servo_jr_grasp(char);
    void publish_servo_jr(char);
    void run();
  private:
    ros::Subscriber sub_measured_cp1;
    ros::Subscriber sub_measured_cp2;
    ros::Subscriber sub_measured_js1;
    ros::Subscriber sub_measured_js2;   

    ros::Publisher pub_servo_cr1;
    ros::Publisher pub_servo_cr2; 
    ros::Publisher pub_servo_cp1;
    ros::Publisher pub_servo_cp2; 
    ros::Publisher pub_servo_jr1;
    ros::Publisher pub_servo_jr2; 
    ros::Publisher pub_servo_jp1;
    ros::Publisher pub_servo_jp2; 
    ros::Publisher pub_servo_jr_grasp1;
    ros::Publisher pub_servo_jr_grasp2; 
};

#endif
