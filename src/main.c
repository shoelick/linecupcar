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
 *  PIT0: Line scan camera -- triggers line scan 
 *  LED : Running state
 *  Pushbutton: Running control
 *
 * -- Pins
 * 	Starboard motor PWM -------- PTD0  (FTM3_CH0) [EXT]
 *  Port motor PWM ------------- PTD1  (FTM3_CH1) [EXT]
 *  Servo PWM ------------------ PTB18 (FTM2_CH0) [EXT]
 *  Camera CLK ----------------- PTC1  (GPIO) [EXT]
 *  Camera SI ------------------ PTC4  (GPIO) [EXT]
 *  Camera Analog Out ---------- ADC0_DP0 [EXT]
 *  Pushbutton ----------------- PTC6
 *  LED	(Red) ------------------ PTB22
 *  LED	(Blue) ----------------- PTE26
 *  LED	(Green) ---------------- PTB21
 * 
 *****************************************************************************/

#include "MK64F12.h"
#include "main.h"
#include "ftm_driver.h"
#include "adc_driver.h"
#include "pit_driver.h"
#include "camera_driver.h"
#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include "uart.h"

/* Camera debugging */
camera_driver camera;
char str[100];

int main() {

    /***************************************************************************
     * VARIABLES AND INSTANCES
     **************************************************************************/
    ftm_driver dc_ftm, servo_ftm, camera_ftm;
    adc_driver adc;
	
	  int i;

    /* Set camera struct valus */
    camera.pixcnt = 0;
    camera.capcnt = 0;
    memset(camera.scan, 0, sizeof(camera.scan));
    camera.clkval = 0;
    camera.ftm = &camera_ftm;
    camera.adc = &adc;

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

    // GPIO camera clk, SI
    PORTC_PCR1 = PORT_PCR_MUX(1); // CLK
    PORTC_PCR4 = PORT_PCR_MUX(1); // SI
	  GPIOC_PDDR |= (1 << 1); // Set output
	  GPIOC_PDDR |= (1 << 4);
	  GPIOB_PSOR |= (1 << 1); // Set default value
	  GPIOB_PSOR |= (1 << 4);

    // Initialize uart for debugging
    uart_init();

		sprintf(str, "MOD: %d C0V: %d \r\n", dc_ftm.regs->MOD, \
			dc_ftm.regs->CONTROLS[0].CnV);
		uart_put(str);

    // Configure DC FTM
    ftm_init(&dc_ftm, 3); /* use ftm3 for dc motors */
		
    ftm_set_frequency(&dc_ftm, 0, 10e3);
    ftm_enable_pwm(&dc_ftm, 0);
    ftm_enable_pwm(&dc_ftm, 1);
    ftm_set_duty(&dc_ftm, 0, 0.2);
    ftm_set_duty(&dc_ftm, 1, 0.5);
    ftm_enable_int(&dc_ftm);

    // Configure Servo FTM
    ftm_init(&servo_ftm, 2);
    ftm_set_frequency(&servo_ftm, 3, 50);
    ftm_enable_pwm(&servo_ftm, 0);
    ftm_set_duty(&servo_ftm, 0, 0.2);
    ftm_enable_int(&servo_ftm);

    // Configure camera FTM
    ftm_init(&camera_ftm, 0);
		ftm_set_frequency(&camera_ftm, 0, DEFAULT_SYSTEM_CLOCK / 100);
		ftm_enable_cntin_trig(&camera_ftm);

    // Configure camera ADC
    adc_init(&adc, 0);
		adc_enable_int(&adc);
		adc_set_ftm0_trig(&adc);

    // Set up PIT 
    init_PIT();

    // Enable FTM interrupts now that everything else is ready
    ftm_enable_int(&camera_ftm);

    /***************************************************************************
     * LOOP
     **************************************************************************/
		 //dc_ftm.regs->MOD = 0x800;
		 //dc_ftm.regs->CONTROLS[0].CnV = 0x666;
		sprintf(str, "Entering loop with MOD: %d C0V: %d \r\n", dc_ftm.regs->MOD, \
		dc_ftm.regs->CONTROLS[0].CnV);
		uart_put(str);
		 
    while (1) {
			
			/*sprintf(str, "MOD: %d C0V: %d \r\n", dc_ftm.regs->MOD, \
				dc_ftm.regs->CONTROLS[0].CnV);
			uart_put(str);
			delay(10);*/

			if (DEBUG_CAM) { // Every 2 seconds
				//if (capcnt >= (2/INTEGRATION_TIME)) {
				if (camera.capcnt >= (500)) {
									// Set SI
					GPIOC_PCOR |= (1 << 4);
					// send the array over uart
					sprintf(str,"%i\n\r",-1); // start value
					uart_put(str);
					for (i = 0; i < 127; i++) {
						sprintf(str,"%i\r\n", camera.scan[i]);
						uart_put(str);
					}
					sprintf(str,"%i\n\r",-2); // end value
					uart_put(str);
					camera.capcnt = 0;
					GPIOC_PSOR |= (1 << 4);
				}
			}
			
	}
}

void delay(int del){
    int i;
    for (i=0; i<del*50000; i++){
        // Do nothing
    }
}

