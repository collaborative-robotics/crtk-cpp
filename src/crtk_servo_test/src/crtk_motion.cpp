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
  servo_cp_updated = 0;
  servo_jr_updated = 0;
  servo_jp_updated = 0;
  servo_jr_grasp_updated = 0;
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
  motion_start_tf = get_measured_cp();
  get_measured_js_pos(motion_start_js_pos,MAX_JOINTS);
}



/**
 * @brief      Sends a servo_cr time. (Must call start_motion function first)
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
  tf::Transform tf_out = tf::Transform();
  tf_out.setIdentity();
  tf_out.setOrigin(vec*step);
  out = send_servo_cr(tf_out);

  // check time
  if(curr_time - motion_start_time > duration){
    ROS_INFO("%f sec movement complete.",duration);
    
    //at end of time, send 0 command
    tf::Transform ident = tf::Transform();
    // ident.setIdentity();
    // out = send_servo_cr(ident);
    return 1;
  }
  return out;
}  

/**
 * @brief      Sends a servo_cp distance. (Must call start_motion function first)
 * 
 * minimum of 1 sec movements
 *
 * @param[in]  vec         The vector
 * @param[in]  total_dist  The total distance
 * @param[in]  duration    The duration
 * @param[in]  curr_time   The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::send_servo_cp_distance(tf::Vector3 vec, float total_dist, time_t curr_time){
  // static char start = 1;
  char out=0;
  
  float safe_speed = 0.025; // m/s
  //figure out total duration with ramp up and ramp down as 1/4 of movement each
  float duration_loops = total_dist/(.875 * safe_speed/LOOP_RATE);
  float step = safe_speed/LOOP_RATE;
  float scale;

  int ramp_loops = duration_loops/4;


  static int loop_count = 0;
  
  // check time
  if(loop_count > duration_loops) {
    ROS_INFO("%f sec movement complete.",duration_loops/LOOP_RATE);
    loop_count = 0;  
    return 1;
  }
    
  loop_count++;

  //determine ramp-up scale for this loop
  if(loop_count <= duration_loops/2){
    scale = std::min((double)loop_count/(double)ramp_loops, (double)1);
  } else{
    scale = 1;
    //scale = std::min((double)(-loop_count+duration_loops)/(double)ramp_loops, (double)1);
  }

  if(scale <= 0) ROS_ERROR("Negative scaling factor of %f OMGWTFBBQsemicolon", scale);

  if(total_dist <= 0){
    ROS_ERROR("Duration and distance should be positive.");
    return -1;    
  }

  // check command
  if(step > 0.001){
    ROS_ERROR("Step size is too big.");
    return -1;
  }  
  if(!vec.normalized()){
    vec = vec.normalize();
    ROS_INFO("Servo_cp direction not normalized. (set to normalized)");
  }

  // send command
  tf::Transform tf_out = tf::Transform();
  tf::Transform curr_pos = get_measured_cp();

  tf_out = curr_pos;
  tf_out.setRotation(motion_start_tf.getRotation());
  tf_out.setOrigin(motion_start_tf.getOrigin()+vec*(step*loop_count*scale));
  out = send_servo_cp(tf_out);

  

  
  return out;
}


/**
 * @brief      goes to desired position and orientation
 *
 * @param[in]  end        The desired endpoint
 * @param[in]  curr_time  The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::go_to_pos(tf::Transform end, time_t curr_time){
  static int loop_count = 0;
  int out = 0;

  float safe_speed = 0.015; // m/s
  float max_omega = 15 DEG_TO_RAD; //per second 
  //figure out total duration with ramp up and ramp down as 1/4 of movement each
  float cart_loops, rot_loops, duration_loops;
  float cart_step, rot_step;
  float scale;
  float rot_diff, cart_diff;
  int ramp_loops;
  

  //determine cartesian diff and rotation diff
  cart_diff =  (end.getOrigin() - motion_start_tf.getOrigin()).length();
  rot_diff = motion_start_tf.getRotation().angleShortestPath(end.getRotation());

  //determine max steps needed and corresponding steps for sync'ed finish
  cart_loops = cart_diff/(.875 * safe_speed/LOOP_RATE); //.875
  rot_loops = rot_diff/(.875 * max_omega/LOOP_RATE); //.875

  duration_loops = std::max((double)cart_loops, (double)rot_loops);
  cart_step = cart_diff/duration_loops;
  rot_step = rot_diff / duration_loops;

  if(loop_count == 5){
    ROS_INFO("cart diff: %f \tcart speed = %f ", cart_diff,  cart_step * 1000);
    ROS_INFO("rot diff: %f \trot speed = %f ", rot_diff RAD_TO_DEG,  rot_step * 1000 RAD_TO_DEG);
  }

  ramp_loops = duration_loops /4;

 
  
  // check time
  if(loop_count > duration_loops) {
    ROS_INFO("%f sec movement complete.",duration_loops/LOOP_RATE);
    loop_count = 0;  
    return 1;
  }
    
  loop_count++;


  scale = std::min((double)loop_count/(double)ramp_loops, (double)1);


  if(scale <= 0) ROS_ERROR("Negative scaling factor of %f OMGWTFBBQsemicolon", scale);

  // check command
  if(cart_step > safe_speed){
    ROS_ERROR("Step size is too big.");
    return -1;
  } else if(rot_step > max_omega){
    ROS_ERROR("Step size is too big.");
  } 

  //calculate new position and orientation
  tf::Transform error = motion_start_tf.inverse() *  end;

  tf::Vector3 error_vec = end.getOrigin() - motion_start_tf.getOrigin();
  
  tf::Vector3 vec_plus = error_vec.normalized() * scale * cart_step * loop_count;
  tf::Vector3 vec_out = motion_start_tf.getOrigin() + vec_plus;

  tf::Vector3 rot_vec = error.getRotation().getAxis();
  tf::Quaternion quat_out = motion_start_tf.getRotation().slerp(end.getRotation(), loop_count/duration_loops);//motion_start_tf.getRotation() * tf::Quaternion(rot_vec, scale * rot_step * loop_count);
  

    tf::Transform tf_out;
    tf_out.setRotation(quat_out);
    tf_out.setOrigin(vec_out);



  if(loop_count == 25){
    ROS_INFO("Start pos: %f, %f, %f", motion_start_tf.getOrigin().x(), motion_start_tf.getOrigin().y()
      , motion_start_tf.getOrigin().z() );
    ROS_INFO("End pos: %f, %f, %f", end.getOrigin().x(), end.getOrigin().y()
      , end.getOrigin().z() );
    ROS_INFO("Difference pos: %f, %f, %f", error_vec.x(), error_vec.y()
      , error_vec.z() );
    error_vec = motion_start_tf.getOrigin() + error_vec;
    ROS_INFO("Start + difference: %f, %f, %f", error_vec.x(), error_vec.y()
      , error_vec.z() );
  }

  out = send_servo_cp(tf_out);

  return out;

}



/**
 * @brief      goes to desired position and orientation
 *
 * @param[in]  end        The desired endpoint
 * @param[in]  curr_time  The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::go_to_jpos(float angle, time_t curr_time){
  

  static int loop_count = 0;
  int out = 0;

  float jr_out[MAX_JOINTS];
  for(int i=0;i<MAX_JOINTS;i++)
    jr_out[i] = 0;

  float max_omega = 20 DEG_TO_RAD; //per second 

  //determine max steps needed and corresponding steps for sync'ed finish
  float rot_step = (.875 * max_omega/LOOP_RATE);
  float duration_loops = angle/rot_step; //.875
  
  // check time
  if(loop_count > duration_loops) {
    ROS_INFO("%f sec movement complete.",duration_loops/LOOP_RATE);
    loop_count = 0;  
    return 1;
  }
  
  //float scale = 1.0*(float)loop_count/(float)duration_loops;
  float scale = 1;
  jr_out[0] = scale*rot_step;

  out = send_servo_jr(jr_out);
  loop_count++;

  return out;

}



/**
 * @brief      Sends a servo_cr increment for a given time. (Must call start_motion
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
    
    // //at end of time, send 0 command
    // tf::Transform ident = tf::Transform();
    // // ident.setIdentity();
    // out = send_servo_cr(ident);
    return 1;
  }
  return out;
}  

/**
 * @brief      Sends a servo_cp rotation over a given distance. (Must call start_motion
 *             function first)
 *
 * @param[in]  vec          The rotation vector
 * @param[in]  total_angle  The total angle (radian)
 * @param[in]  duration     The duration
 * @param[in]  curr_time    The curr time
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion::send_servo_cp_rot_angle(tf::Vector3 vec, float total_angle, time_t curr_time){
  // static char start = 1;
  char out=0;
  float max_omega = 15 DEG_TO_RAD; //per second 
  float step = max_omega/LOOP_RATE;
  static int loop_count = 0;
  int loop_duration = total_angle / step;
  
  // check time 
  if(loop_count >= loop_duration){
    ROS_INFO("%f sec rotation movement complete", (float)loop_duration/LOOP_RATE);
    loop_count = 0;
    return 1;
  }
  loop_count++;

  if(step > max_omega/LOOP_RATE){
    ROS_ERROR("Step size is too big.");
    return -1;
  }

  // send command
  if(!vec.normalized()){
    vec = vec.normalize();
    ROS_INFO("Servo_cp rot direction not normalized. (set to normalized)");
  }

  //get current pose
  tf::Quaternion start_rot = motion_start_tf.getRotation();
  tf::Vector3 curr_pos = motion_start_tf.getOrigin();

  // add (by multiplication) current pose and increment quaternion
  float out_angle = step * loop_count;
  tf::Quaternion out_qua = start_rot * tf::Quaternion(vec, out_angle);
  
  out = send_servo_cp(tf::Transform(out_qua, curr_pos));

  return out;
}  

//_cr
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
  servo_cr_command = tf::Transform();
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

//_cp

char CRTK_motion::send_servo_cp(tf::Transform trans){

  // send command
  servo_cp_updated = 1;
  servo_cp_command = trans;

  return 0;
}

void CRTK_motion::reset_servo_cp_updated(){
  servo_cp_command = tf::Transform();
  servo_cp_updated = 0;
}

char CRTK_motion::get_servo_cp_updated(){
  // servo_cp_updated = 1;
  // ROS_INFO("get says: %i", servo_cp_updated);
  return servo_cp_updated;
}

tf::Transform CRTK_motion::get_servo_cp_command(){
  return tf::Transform(servo_cp_command);
}


time_t CRTK_motion::get_start_time(){
  return motion_start_time;
}

tf::Transform CRTK_motion::get_start_tf(){
  return motion_start_tf;
}


char CRTK_motion::send_servo_jr(float jpos_d[MAX_JOINTS]){

  float step_angle;
  for(int i=0;i<MAX_JOINTS;i++){
    step_angle = jpos_d[i];
    if(fabs(step_angle) > STEP_ROT_LIMIT){ 
      ROS_ERROR("Servo_jr step limit exceeded. Motion not sent.");
      reset_servo_jr_updated();
      return -1;
    }
  }
  
  // send command
  servo_jr_updated = 1;
  for(int i=0;i<MAX_JOINTS;i++)
    servo_jr_command[i] = jpos_d[i];
  return 0;
}

char CRTK_motion::send_servo_jp(float jpos_d[MAX_JOINTS]){

  // send command
  servo_jp_updated = 1;
  for(int i=0;i<MAX_JOINTS;i++)
    servo_jp_command[i] = jpos_d[i];
  return 0;
}


char CRTK_motion::send_servo_jr_grasp(float step_angle){

  
  if(fabs(step_angle) > STEP_ROT_LIMIT){ 
    ROS_ERROR("Servo_jr_grasp step limit exceeded. Motion not sent.");
    reset_servo_jr_grasp_updated();
    return -1;
  }
  // send command
  servo_jr_grasp_updated = 1;
  servo_jr_grasp_command = step_angle;
  return 0;
}


void CRTK_motion::reset_servo_jr_updated(){
  servo_jr_updated = 0;
}

void CRTK_motion::reset_servo_jp_updated(){
  servo_jp_updated = 0;
}

void CRTK_motion::reset_servo_jr_grasp_updated(){
  servo_jr_grasp_updated = 0;
}



char CRTK_motion::get_servo_jr_updated(){
  return servo_jr_updated;
}


char CRTK_motion::get_servo_jp_updated(){
  return servo_jp_updated;
}


char CRTK_motion::get_servo_jr_grasp_updated(){
  return servo_jr_grasp_updated;
}

float CRTK_motion::get_servo_jr_grasp_command(){
  return servo_jr_grasp_command;
}

void CRTK_motion::get_servo_jr_command(float* out, int length){
  for(int i=0;i<length;i++)
   out[i] = servo_jr_command[i];
}

void CRTK_motion::get_servo_jp_command(float* out, int length){
  for(int i=0;i<length;i++)
   out[i] = servo_jp_command[i];
}

char CRTK_motion::set_home_pos(tf::Quaternion q_in, tf::Vector3 v_in){
  home_pos.setRotation(q_in);
  home_pos.setOrigin(v_in);
  return 1;
}

tf::Transform  CRTK_motion::get_home_pos(){
  return home_pos;
}