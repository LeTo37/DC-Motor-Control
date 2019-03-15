#include "ADC.h"                   
#include <xc.h>

void adc_init(void) 
{
  //ADC Setup
  AD1PCFGbits.PCFG0 = 0; 
  AD1CON1bits.SSRC = 0x7; //Set the conversion process to be automatic
  AD1CON1bits.ASAM = 0;   //Set the sampling process to be manual
  AD1CON3bits.ADCS = 2;   //Set Tad to be 2*(ADCS+1)*Tpb = 75ns
  AD1CON3bits.SAMC = 2;   //Set sampling time to be 2*Tad = 150ns
  AD1CON1bits.ADON = 1;   // turn on A/D converter
}

int adc_read_counts(void)
{
  float fin_val = 0;
  int i = 0,avg_count=5;
  for (i = 0; i < avg_count; i++){//Read a few times and average
    AD1CHSbits.CH0SA = 0;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    // AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    fin_val = fin_val + ADC1BUF0;     //increment value from buffer
  }
  fin_val = fin_val/avg_count;  	// average value
  return (int)(fin_val);
}


int adc_read_mA(void)
{//read the mA from calculated line
  int adc_counts = adc_read_counts();
  //line = 2.084x-1032.48
  float fin_val = (2.084*(float)adc_counts)-1032.48;
  return (int)(fin_val);
}