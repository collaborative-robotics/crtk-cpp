#ifndef _MAIN_H_
#define _MAIN_H_

#include <crtk_lib_cpp/defines.h>

enum cube_dir{cube_x, cube_y, cube_z};
char front_face = 0b100;
char left_face  = 0b010;
char lower_face = 0b001;

/**
 * @brief      The function executes a random cube tracing example
 *             CRTK Command:     servo_cr 
 *             Passing criteria: ask user
 *
 * @param      robot         The robot
 * @param[in]  current_time  The current system time
 *
 * @return     0
 */
int run_cube(CRTK_robot *, time_t);


/**
 * @brief      The function decides the next robot motion direction randomly
 *
 * @param      curr_vertex  The curr vertex
 * @param      move_vec     The move vector
 * @param      prev_axis    The previous axis
 *
 * @return     success
 */
char rand_cube_dir(char *, tf::Vector3 *, CRTK_axis *);

#endif