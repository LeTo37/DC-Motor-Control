#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"       // Encoder module
#include "Mode.h"         // Mode module
#include "ADC.h"          // ADC Module
#include "Current_Controller.h"   //Current Controller Module
#include "Pos_Controller.h"       //Position Controller Module
#include <stdio.h>
// include other header files here

#define BUF_SIZE 200
//Mode macros
#define IDLE 1
#define PWM 2
#define ITEST 3
#define HOLD 4
#define TRACK 5

//Function Declerations
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) current_controller(void);       //Timer 2
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) pos_controller(void);  //Timer 4

//Global
static volatile int duty_cycle = 0;
static volatile float curr_Kp = 1.8, curr_Ki = 0.003; //current gains
static volatile int curr_Count = 0, curr_ref_test[99],curr_actual_test[99]; //current testing
static volatile float pos_Kp = 75, pos_Ki = 0, pos_Kd = 5000; //position gains
static volatile int curr_ref = 0,deg_desired = 0;  //position and current reference signals
static volatile int traj_No_Samples = 0, traj_i = 0, traj_actual[5000];
static volatile float traj_ref[5000];

int main() 
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;        
  __builtin_disable_interrupts();
  encoder_init();
  adc_init();
  current_init();
  pos_init();
  __builtin_enable_interrupts();

  set_mode(IDLE);

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':             //Read Current Sensor (ADC counts)
      {
        sprintf(buffer,"%d\r\n", adc_read_counts());
        NU32_WriteUART3(buffer);  //send adc counts to client
        break;
      }
      case 'b':             // Read Current sensor (mA)
      { 
        sprintf(buffer,"%d\r\n", adc_read_mA());
        NU32_WriteUART3(buffer);  //send mA to client
        break;
      }
      case 'c':             //Read Encoder
      {
        encoder_counts();    //initial read to clear
        sprintf(buffer,"%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer); //send encoder count to client
        break;
      }
      case 'd':                      //Read Encoder in Degrees
      {            
        float degrees = encoder_deg();
        sprintf(buffer,"%f\r\n", degrees);
        NU32_WriteUART3(buffer);  //send encoder degrees to client
        break;
      }
      case 'e':                      // Reset encoder
      { 
        encoder_reset_count();             //reset
        break;
      }
      case 'f':                      //Set PWM (-100 to 100)
      { 
        int pwm_in = 0;
        set_mode(PWM);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &pwm_in);
        if (pwm_in < 0)
          {dir = 0;}
        else
          {dir = 1;}
        duty_cycle = abs(pwm_in)*(4000/100);
        break;
      }
      case 'g':                   //Set Current Gains
      {
       NU32_ReadUART3(buffer,BUF_SIZE);
       sscanf(buffer, "%f %f", &curr_Kp, &curr_Ki);
       break; 
      }
      case 'h':                   //Get Current Gains
      {
       sprintf(buffer,"%f %f\r\n", curr_Kp, curr_Ki);
       NU32_WriteUART3(buffer);
       break; 
      }
      case 'i':                   //Set Position Gains
      {
       NU32_ReadUART3(buffer,BUF_SIZE);
       sscanf(buffer, "%f %f %f", &pos_Kp, &pos_Ki, &pos_Kd);
       break; 
      }
      case 'j':                   //Get Position Gains
      {
       sprintf(buffer,"%f %f %f\r\n", pos_Kp, pos_Ki, pos_Kd);
       NU32_WriteUART3(buffer);
       break; 
      }
      case 'k':                   //Test Current Control
      {
       set_mode(ITEST);
       while (get_mode() == ITEST);      //testing

       sprintf(buffer,"%d\r\n",99);  //send number of samples
       NU32_WriteUART3(buffer);

       int i;
       for (i = 0; i < 99; i++)
       {
          sprintf(buffer,"%d %d\r\n", curr_ref_test[i], curr_actual_test[i]);
          NU32_WriteUART3(buffer); //send data for plotting
       }
       break; 
      }
      case 'l':                   //Go to angle (deg)
      {
       curr_E_int = 0;
       pos_E_int = 0;
       pos_E_prev = 0;
       NU32_ReadUART3(buffer,BUF_SIZE);
       sscanf(buffer, "%d", &deg_desired);
       set_mode(HOLD);
       break;
      }
      case 'm':                   //Load Step Trajectory
      {
        set_mode(IDLE);         //want everyone to calm down while loading trajectory
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &traj_No_Samples);
        int i, temp = 0;
        for (i = 0; i < traj_No_Samples; i ++)
        {
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer,"%d",&temp);
          traj_ref[i]=temp;
        }
        break;
      }
      case 'n':                   //Load Cubic Trajectory
      { 
        set_mode(IDLE);         //want everyone to calm down while loading trajectory
        // __builtin_disable_interrupts();
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &traj_No_Samples);
        int i=0;        
        float temp = 0;
        for (i = 0; i < traj_No_Samples; i ++)
        {
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer,"%f",&temp);
          traj_ref[i]=(int)temp;
        }
        // __builtin_enable_interrupts();
        break;
      }
      case 'o':                   //Execute Trajectory
      {
        set_mode(TRACK);
        while (get_mode() == TRACK);      //Executing
        sprintf(buffer,"%d\r\n", traj_No_Samples);
        NU32_WriteUART3(buffer);
        int i = 0;
        for (i = 0; i < traj_No_Samples; i ++)
        {
          sprintf(buffer,"%f %d\r\n", traj_ref[i], traj_actual[i]);
          NU32_WriteUART3(buffer); 
        }
        break;
      }
      case 'p':                   //Unpower the motor
      {
        set_mode(IDLE);
        break;
      }
      case 'q':
      {
        set_mode(IDLE);
        break;
      }
      case 'r':                       //get mode
      {
        sprintf(buffer,"%d\r\n", get_mode());
        NU32_WriteUART3(buffer);  //send mode to client
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}


void __ISR(_TIMER_2_VECTOR, IPL5SOFT) current_controller(void)
{
  switch (get_mode()) 
  {
    case 1:  //IDLE
    { //Should put H-bridge into brake mode.
      set_direction(0);
      OC1RS = 0;
      duty_cycle = 0;  //ensure its not set somewhere else
      break;
    }
    case 2:  //PWM
    {
      set_direction(dir);
      OC1RS = duty_cycle;
      break;
    }
    case 3:  //ITEST
    {
      if ((curr_Count > 25 && curr_Count < 50)||(curr_Count > 75))
      {
        curr_ref_test[curr_Count] = (-1)*(200);
      }
      else 
      {
        curr_ref_test[curr_Count] = 200;
      }
      curr_actual_test[curr_Count] = adc_read_mA();

      OC1RS = curr_PI(curr_Kp, curr_Ki, curr_ref_test[curr_Count], curr_actual_test[curr_Count]);
      set_direction(dir);

      if (curr_Count == 99)
      {
        curr_Count = 0;
        set_mode(IDLE);
        curr_E_int = 0;
        break;
      }

      curr_Count++;
      break;
    }
    case 4:  //HOLD
    {
      if (curr_ref < 0)
        {dir = 0;}
      else
        {dir = 1;}
      int curr_actual = adc_read_mA();
      OC1RS = curr_PI(curr_Kp,curr_Ki,curr_ref, curr_actual);
      set_direction(dir);
      break;
    }
    case 5:  //TRACK
    { 
      if (curr_ref < 0)
        {dir = 0;}
      else
        {dir = 1;}
      int curr_actual = adc_read_mA();
      OC1RS = curr_PI(curr_Kp,curr_Ki,curr_ref, curr_actual);
      set_direction(dir);

      break;
    }

  }

  IFS0bits.T2IF = 0;       //int flag
}


void __ISR(_TIMER_4_VECTOR, IPL5SOFT) pos_controller(void)  //Timer 4
{
  if (get_mode() == HOLD)
  {
    int actual_pos = (int)encoder_deg();
    curr_ref = pos_PID(pos_Kp, pos_Ki, pos_Kd, deg_desired, actual_pos);
  }

  if (get_mode() == TRACK)
  {
    int actual_pos = (int)encoder_deg();
    traj_actual[traj_i] = actual_pos;
    curr_ref = pos_PID(pos_Kp, pos_Ki, pos_Kd, traj_ref[traj_i], traj_actual[traj_i]);

    if(traj_i == traj_No_Samples)
    {
      curr_E_int = 0;
      pos_E_int = 0;
      pos_E_prev = 0;
      traj_i = 0;
      deg_desired = actual_pos;
      set_mode(HOLD);

    }
    else
      {traj_i++;}
  }

  IFS0bits.T4IF = 0;       //int flag
}