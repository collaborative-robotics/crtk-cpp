#ifndef _MAIN_H_
#define _MAIN_H_

#include <crtk_lib_cpp/defines.h>

enum cube_dir{cube_x, cube_y, cube_z};
char front_face = 0b100;
char left_face  = 0b010;
char lower_face = 0b001;

int run_cube(CRTK_robot *, time_t);
char rand_cube_dir(char *, tf::Vector3 *, CRTK_axis *);

#endif