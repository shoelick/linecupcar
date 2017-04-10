/* 
 * This file will contain the main racing program.
 */
 
 /*****************************************************************************
  * DISTRIBUTION OF HARDWARE
	*
	* -- Devices
	* 	ADC0: Line scan using channel 0
	* 	FTM0: Line scan using channel 5, drives clock and SI pulse
	* 	FTM2: Servo motor control, using channel 0
	* 	FTM3: DC Motor Control, using channels 0 and 1
	*		PIT0: Line scan camera -- triggers line scan 
	*   LED : Running state
	*		Pushbutton: Running control
	*
	* -- Pins
	* 	Starboard motor PWM -------- PTD0  (FTM3_CH0)
	*		Port motor PWM ------------- PTD1  (FTM3_CH1)
	* 	Servo PWM ------------------ PTB18 (FTM2_CH0)
	*		Camera CLK ----------------- PTC1  (FTM0_CH0)
	*		Camera SI ------------------ PTC4	 (GPIO)
	*		Camera Analog Out ---------- ADC0_DP0
	*		Pushbutton ----------------- PTC6
	*		LED	(Red) ------------------ PTB22
	*		LED	(Blue) ----------------- PTE26
	*		LED	(Green) ---------------- PTB21
	* 
	*****************************************************************************/
 
#include "MK64F12.h"
#include "ftm_driver.h"
#include "adc_driver.h"
#include "pit_driver.h"
#include "stdlib.h"

#define DEFAULT_SYSTEM_CLOCK 20485760U

void delay(int del);

int main() {
		
		/***************************************************************************
	  * VARIABLES AND INSTANCES
	  **************************************************************************/
	  ftm_driver dc_ftm, servo_ftm, camera_ftm;
		adc_driver adc;
	 
		/***************************************************************************
	  * CONFIGURATION
	  **************************************************************************/
	
		// Enable clock on GPIO PORTS so it can output
		SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | \
			SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
	
		// Mux all FTMs
		PORTD_PCR0 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
		PORTD_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
		PORTB_PCR18 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;
		PORTC_PCR0 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	
	  // Configure DC FTM
		ftm_init(&dc_ftm, 3); /* use ftm3 for dc motors */
		ftm_set_frequency(&dc_ftm, 0, 1000);
		ftm_enable_pwm(&dc_ftm, 0);
		ftm_enable_pwm(&dc_ftm, 1);
		ftm_set_duty(&dc_ftm, 0, 0.2);
		ftm_set_duty(&dc_ftm, 1, 0.5);
		ftm_enable_int(&dc_ftm);
	
		// Configure Servo FTM
		ftm_init(&servo_ftm, 2);
		ftm_set_frequency(&servo_ftm, 4, 50);
		ftm_enable_pwm(&servo_ftm, 0);
		ftm_set_duty(&servo_ftm, 0, 0.2);
		ftm_enable_int(&servo_ftm);
	
		// Configure camera FTM
	  ftm_init(&camera_ftm, 0);
	 
	  // Set up PIT 
		init_PIT();
	 
	  // Configure camera ADC
	 
	 /***************************************************************************
	  * LOOP
	  **************************************************************************/
		while (1) {
			
			
			
		}
}

void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

