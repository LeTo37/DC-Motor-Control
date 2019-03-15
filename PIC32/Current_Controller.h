#ifndef CURRENT_CONTROLLER
#define CURRENT_CONTROLLER

//motor drive variables
volatile int curr_E_int;
volatile int dir; 

void current_init(void);
int curr_PI(float curr_Kp, float curr_Ki, int ref, int actual);
void set_direction(int d);


#endif