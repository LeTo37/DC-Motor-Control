#ifndef POS_CONTROLLER
#define POS_CONTROLLER

//errors
volatile int pos_E_int;
volatile int pos_E_prev;

void pos_init(void);
int pos_PID(float pos_Kp, float pos_Ki, float pos_Kd, int ref_pos, int actual_pos);

#endif