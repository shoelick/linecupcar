/* 
 * main.c
 * The main racing program for Imagine RIT Line-Following Car Cup.
 * Author: Michael Shullick
 * April, 2017
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

/* System includes */
#include <stdio.h>
#include <string.h>
#include <MK64F12.h>

/* Project includes */
#include "main.h"
#include "ftm_driver.h"
#include "gpio.h"
#include "adc_driver.h"
#include "pit_driver.h"
#include "camera_driver.h"
#include "stdlib.h"
#include "uart.h"
#include "util.h"

unsigned long DEFAULT_SYSTEM_CLOCK = 20485760U;

char str[100]; /* This is used to print from everywhere */
camera_driver camera; /* Externally defined for use in ISRs */

double normalized[SCAN_LEN];
double filtered[SCAN_LEN];
int processed[SCAN_LEN];

/* Macro to turn the setpoint into a servo duty */
const double SERVO_MIN  = 0.07;
const double SERVO_MAX  = 0.1042;
#define TO_SERVO_DUTY(S) ((SERVO_MAX-SERVO_MIN) * S + SERVO_MIN) 

const double DC_MAX = 0.25;

/* FTM Channels */
const int CH_STARBOARD = 0;
const int CH_PORT = 1;
const int CH_SERVO = 0;

int main() {

    /***************************************************************************
     * VARIABLES AND INSTANCES
     **************************************************************************/
    ftm_driver dc_ftm, servo_ftm, camera_ftm;
    adc_driver adc;

    /* Setpoint values */
    double s_throttle = 0.5, p_throttle = 0.5, steering = 0.5;

    /* state management */
    int8_t running = 0, sw;
    uint8_t button_held = 0, state_color = 1, line_detected = 0;
    long int light_elapsed = 0;
    const long int LIGHT_INT = 3000;
    int numlines = 0; 
    int center = 0;
    double position;

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

    // Initialize the rest of the GPIO for buttons/LEDs
    led_init();
    button_init();

    // Initialize uart for debugging
    uart_init();

    // Configure DC FTM
    ftm_init(&dc_ftm, 3); /* use ftm3 for dc motors */
    ftm_set_frequency(&dc_ftm, 0, 10e3);
    ftm_enable_pwm(&dc_ftm, 0);
    ftm_enable_pwm(&dc_ftm, 1);
    ftm_set_duty(&dc_ftm, 0, 0);
    ftm_set_duty(&dc_ftm, 1, 0);
    ftm_enable_int(&dc_ftm);

    // Configure Servo FTM
    ftm_init(&servo_ftm, 2);
    ftm_set_frequency(&servo_ftm, 3, 50);
    ftm_enable_pwm(&servo_ftm, 0);
    ftm_set_duty(&servo_ftm, 0, 0.5);
    ftm_enable_int(&servo_ftm);

    // Configure camera FTM
    ftm_init(&camera_ftm, 0);
    //ftm_set_frequency(&camera_ftm, 0, 150k);
    ftm_set_mod(&camera_ftm, 0, 100);
    ftm_set_duty(&camera_ftm, 0, 0.5);
    ftm_enable_pwm(&camera_ftm, 0);
    ftm_enable_cntin_trig(&camera_ftm);

    // Configure camera ADC
    adc_init(&adc, 0);
    adc_enable_int(&adc);
    adc_set_ftm0_trig(&adc);

    // Don't enable FTM interrupts until adc has been initalized
    ftm_enable_int(&camera_ftm);

    // Set up PIT 
    init_PIT();


    /***************************************************************************
     * LOOP
     **************************************************************************/

    while (1) {

        /* Output to UART if enabled */
        if (DEBUG_CAM) matlab_print();

        /* Do camera processing */
        if (camera.newscan) {

            // Normalize input
            i_normalize(&normalized[0], &camera.wbuffer[0], SCAN_LEN);

            // Perform low pass for noise cleaning
            //convolve(&filtered[0], &normalized[0], SCAN_LEN, LOW_PASS, 3);
            convolve(&filtered[0], &normalized[0], SCAN_LEN, BOXCAR_4, 4);

            // Do naiive derivative
            slopify(&normalized[0], &filtered[0], SCAN_LEN);

            // Perform high pass for derivative 
            // Put back into normalized because we need two separate 
            // buffers
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, DERIVATIVE, 2);
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, HIGH_PASS, 3);

            // Normalize derivative
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, BOXCAR_4, 4);
            convolve(&filtered[0], &normalized[0], SCAN_LEN, BOXCAR_4, 4);
            d_normalize(&normalized[0], &filtered[0], SCAN_LEN);

            // Threshold
            threshold(&processed[0], &normalized[0], SCAN_LEN, 0.5); 

            numlines= count_lines(processed, SCAN_LEN);
            printu("Num lines: %d\n\r", numlines); 

            // Find maximum groups
            if (numlines >= 1) {

                center = center_average(processed, SCAN_LEN);

                printu("center of lines: %d\n\r",  center);
                printu("--------------------");
                
                if (numlines == 2) {
                    steering = ((double) center / SCAN_LEN);
                    position = 1 - steering;
                } else if (numlines == 1) {
                    //steering center /= SCAN_LEN; 
                    if (position > 0.5) {
                        steering = pos - center;
                    } else {
                        steering = center - pos;
                    }
                } else {
                    steering = 0.5;
                }


                /*sprintf(str, "steering: 0.%d\n\r",  (int) (steering * 1000));
                uart_put(str);*/
                // Check for finish line 

                // Try to find left line

                // Try to find right line

                // If we have both, find center

                // Otherwise, hug the one we have

                // we found lines 
                line_detected = 1;
            } else {
                line_detected = 0;

                // TODO: Keep track of time we haven't seen a line 
            }

            // Allow a new scan
            camera.newscan = 0;
           
        }

        /* Compute error and update setpoints */
        if (line_detected) {



        }

        if (running) {

            // update values 
            if (p_throttle < DC_MAX) p_throttle += 0.05;
            if (s_throttle < DC_MAX) s_throttle += 0.05;



        } else {

            // slow down
            if (p_throttle > 0) p_throttle -= 0.02;
            if (s_throttle > 0) s_throttle -= 0.02;

            if (p_throttle < 0) p_throttle = 0.00;
            if (s_throttle < 0) s_throttle = 0.00;

            steering = 0.5;
        }

        /* Motors Update */
        ftm_set_duty(&dc_ftm, CH_STARBOARD, s_throttle);
        ftm_set_duty(&dc_ftm, CH_PORT, p_throttle);
        ftm_set_duty(&servo_ftm, CH_SERVO, TO_SERVO_DUTY(steering));

        /* Update LED */
        if (light_elapsed >= LIGHT_INT) {

            if (state_color) {
                (running) ? set_led(GREEN) : set_led(RED);
            } else {
                (line_detected) ? set_led(BLUE) : set_led(OFF);    
            }

            state_color = !state_color;
            light_elapsed = 0;

        } else light_elapsed++;

        /* Update state */
        sw = sw_active();
        if (sw && !button_held) {

            running = !running;
            button_held = 1;

        } else if (!sw && button_held) button_held = 0;
    }
}


/*
 * This function prints out camera values in a format expected by a listening 
 * matlab instance.
 */
void matlab_print() {
    int i;

    //if (capcnt >= (2/INTEGRATION_TIME)) {
    if (camera.capcnt >= (500)) {
        // Set SI
        //GPIOC_PCOR |= (1 << 4);
        // send the array over uart
        sprintf(str,"%d\n\r",-1); // start value
        uart_put(str);
        for (i = 0; i < SCAN_LEN - 1; i++) {
            //sprintf(str,"%d\r\n", camera.wbuffer[i]);
            //sprintf(str,"%d\r\n", (int) (filtered[i] * 100.0));
            sprintf(str,"%d\r\n", (int) (processed[i]));
            //sprintf(str, "%d\r\n", (int) (normalized[i] * 1000.0));
            uart_put(str);
        }
        sprintf(str,"%d\n\r",-2); // end value
        uart_put(str);
        camera.capcnt = 0;
        //GPIOC_PSOR |= (1 << 4);
    }
}

