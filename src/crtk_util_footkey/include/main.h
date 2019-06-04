#ifndef _MAIN_H_
#define _MAIN_H_

// The main function that executes keyboard alternative for foot pedal up and down
int main(int argc, char **argv);

// checks terminal for 'e' or 'd' 
int foot_pedal();

// publishes CRTK commands for the given foot command
int pub_foot(int foot);

// gets keyboard character for switch case's of console_process()
int getkey();

#endif