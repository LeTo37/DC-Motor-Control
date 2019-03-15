#include "Current_Controller.h"
#include <xc.h>


void current_init(void)
{
//Configure Timer 2 for 5kHz interrupt:
    T2CONbits.TCKPS = 0b110;     // Timer2 prescaler N=64 (1:64)
    PR2 = 249;              // period = (PR2+1) * N * 12.5 ns = 200us, 5 kHz
    TMR2 = 0;                // initial TMR2 count is 0
    T2CONbits.ON = 1;        // turn on Timer2
    IPC2bits.T2IP = 5;       //priority
    IPC2bits.T2IS = 0;       //sub-priority       
    IFS0bits.T2IF = 0;       //int flag
    IEC0bits.T2IE = 1;       //enable int

//Configure Timer3 and OC1 (D0) for 20kHz PWM:
  T3CONbits.TCKPS = 0;     // Timer3 prescaler N=1 (1:1)
  PR3 = 3999;              // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
  TMR3 = 0;                // initial TMR3 count is 0
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 3000;             // duty cycle = OC1RS/(PR3+1) = 75%
  OC1R = 3000;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.OC32 = 0;
  OC1CONbits.OCTSEL = 1;  //Use Timer3
  OC1CONbits.ON = 1;       // turn on OC1

//Direction bit:
  //D10
  TRISDbits.TRISD10 = 0;//set as output
  set_direction(0);//Set as low to begin with
}

int curr_PI(float curr_Kp, float curr_Ki, int ref, int actual)
{
  int error = ref-actual;
  curr_E_int = curr_E_int + error;

  if (error >0){
    dir = 1;
  }else{
    dir = 0;
  }
  
  int u = abs((curr_Kp*error) + (curr_Ki*curr_E_int));
  
  if (u > 100)
  {u = 100;} //can't have pwm above 100%
   
  return (unsigned int)((u/100.0)*4000);
}

void set_direction(int d)
{
  //0 = low, 1 = high
  LATDbits.LATD10 = d;
}
