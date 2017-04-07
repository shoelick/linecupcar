/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Spring 2016
*
* Filename: main_A2D_template.c
*/
 
#include "uart.h"
#include "MK64F12.h"
#include "stdio.h"

 
void PDB_INIT(void) {
    //Enable PDB Clock
    SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;
    //PDB0_CNT = 0x0000;
    PDB0_MOD = 50000; // 50,000,000 / 50,000 = 1000

    PDB0_SC = PDB_SC_PDBEN_MASK | PDB_SC_CONT_MASK | PDB_SC_TRGSEL(0xf)
                                    | PDB_SC_LDOK_MASK;
    PDB0_CH1C1 = PDB_C1_EN(0x01) | PDB_C1_TOS(0x01);
}
 
void ADC1_INIT(void) {
    unsigned int calib;
 
    // Turn on ADC1
	SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
	
    // Configure CFG Registers
    // Configure ADC to divide 50 MHz down to 6.25 MHz AD Clock, 16-bit single ended
    ADC1_CFG1 |= ADC_CFG1_ADIV_MASK;
	ADC1_CFG1 |= ADC_CFG1_MODE_MASK;
	
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC1_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC1_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC1_CLP0;
    calib += ADC1_CLP1;
    calib += ADC1_CLP2;
    calib += ADC1_CLP3;
    calib += ADC1_CLP4;
    calib += ADC1_CLPS;
    calib = calib >> 1;
    calib |= 0x8000;
    ADC1_PG = calib;
 
    // Configure SC registers.
    // Select hardware trigger.
    ADC1_SC2 |= ADC_SC2_ADTRG_MASK;
 
    // Configure SC1A register.
    // Select ADC Channel and enable interrupts. Use ADC1 channel DAD3 in single ended mode.
    ADC1_SC1A |= ADC_SC1_AIEN_MASK; 
	ADC1_SC1A &= ~(ADC_SC1_ADCH_MASK);
	ADC1_SC1A |= 0x3;
 
    // Enable NVIC interrupt
    NVIC_EnableIRQ(ADC1_IRQn);
}
 
// ADC1 Conversion Complete ISR
void ADC1_IRQHandler(void) {
    // Read the result (upper 12-bits). This also clears the Conversion complete flag.
    unsigned short i = ADC1_RA >> 4;

    //Set DAC output value (12bit)
    DAC0_DAT0L = i << 4;        //store least significant 8 bits                     
    DAC0_DAT0H = i >> 8;        //store most significant 4 bits
}

void DAC0_INIT(void) {
    //enable DAC clock
    SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;
    DAC0_C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
    DAC0_C1 = 0;
}
 
int main(void) {
    int i; char str[100];
		int tempC, tempF;
		double V;
    // Enable UART Pins
 
   
    // Initialize UART
    uart_init();
               
    DAC0_INIT();
    ADC1_INIT();
    PDB_INIT();
	

    // Start the PDB (ADC Conversions)
    PDB0_SC |= PDB_SC_SWTRIG_MASK;
 
    for(;;) {
		V = ADC1_RA*3.3/(65536);	
		tempC = 100*V - 50;
		tempF = (tempC*9/5)+(32);
		sprintf(str,"\nTempC: %d TempF: %d \n\r",tempC,tempF);
		uart_put(str);
		for( i=0; i < 5000000; ++i ){
                       
        }
    }
 
    return 0;
}
 