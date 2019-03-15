#include "Mode.h"
volatile int mode;
//Getter and Setter
int get_mode(void)
{
	return mode;
}
void set_mode(int new_mode)
{
	mode = new_mode;
}