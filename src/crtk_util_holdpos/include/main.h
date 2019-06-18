#ifndef _MAIN_H_
#define _MAIN_H_


#include "ros/ros.h"
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/operating_state.h>
#include <sstream>
#include <iostream>
#include <string>
#include <tf/tf.h>
#include <cstdio>
#include <iomanip>
#include <termios.h>  // needed for terminal settings in getkey()


using namespace std;

#define LOOP_RATE 1000

//subscribe and publish
ros::Subscriber sub_measured_cp;
ros::Subscriber sub_operating_state;
ros::Publisher pub_servo_cp;
ros::Publisher pub_state_command;

tf::Transform current_pos;
char robot_state;

bool is_homed;
bool is_busy;

enum CRTK_robot_command {CRTK_ENABLE, CRTK_DISABLE, CRTK_PAUSE, CRTK_RESUME, CRTK_UNHOME, CRTK_HOME};


ros::Publisher command_pub;
void crtk_command_pb(CRTK_robot_command command);
void crtk_measured_cp_arm_cb(geometry_msgs::TransformStamped);

void operating_state_cb(crtk_msgs::operating_state msg);

int main(int argc, char **argv);
char enable_if_safe();
int getkey();



#endif