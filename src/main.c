/* 
 * main.c
 * The main racing program for Imagine RIT Line-Following Car Cup.
 * Authors: Michael Shullick, Cindy Gomez
 * April, 2017
 */

/*****************************************************************************
 * DISTRIBUTION OF HARDWARE
 *
 * -- Devices
 *  ADC0: Line scan using channel 0
 *  FTM0: Line scan using channel 5, drives clock and SI pulse
 *  FTM2: Servo motor control, using channel 0
 *  FTM3: DC Motor Control, using channels 0 and 1
 *  PIT0: Line scan camera -- triggers line scan 
 *  LED : Running state
 *  Pushbutton: Running control
 *
 * -- Pins
 *  Starboard motor PWM -------- PTD0  (FTM3_CH0) [EXT]
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
#include <math.h>
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

camera_driver camera; /* Externally defined for use in ISRs */

/* Line scan processing buffers */
double normalized[SCAN_LEN];
double filtered[SCAN_LEN];
int processed[SCAN_LEN];

//const double SERVO_MIN  = 0.0525;
const double SERVO_MIN  = 0.05;
const double SERVO_MAX  = 0.097;
const double STEER_CENTER = 0.5;

/* 
 * Macro to turn the setpoint into a servo duty 
 * This way we can think about steering where 0 is the hardest left and 1.0 is
 * the hardest right.
 */
#define TO_SERVO_DUTY(S) ((SERVO_MAX-SERVO_MIN) * S + SERVO_MIN) 

/*
 * Max speed [0, 1.0]
 * Corresponds to FTM duty
 */
const double DC_MAX = 0.40;

/* PID Constants */
const double Kp = 0.55, Ki = 0.05, Kd = 0.1;

/* FTM Channels */
const int CH_STARBOARD = 0;
const int CH_PORT = 1;
const int CH_SERVO = 0;

ftm_driver dc_ftm, servo_ftm, camera_ftm;
adc_driver adc;

int main() {

    /***************************************************************************
     * VARIABLES AND INSTANCES
     **************************************************************************/

    /* Setpoint values */
    double s_throttle = 0.5, p_throttle = 0.5, steering = STEER_CENTER;

    /* old steering and error vars for integral control */
    double old_steer, old_err, error;

    /* state management */
    int8_t running = 0, sw;
    uint8_t button_held = 0, state_color = 1, line_detected = 0;
    const long int LIGHT_INT = 500;
    long int light_elapsed = 0;

    /* Position tracking */
    //int center = 0;
    double position;
    double goal = 0.3; // for now, let's stick to staying the middle 
    double goal_th = 0; // Goal throttle value 
    double integral = 0, derivative;

    /* 
     * Using the derivative, we end up producing different values for 
     * dark-to-light and light-to-dark, corresponding to left and right
     * respectively.
     */
    const int RIGHT_VAL = -1;
    const int LEFT_VAL = 1;
    int right_ind, left_ind;
    double right_pos, left_pos, right_d, left_d;
    double c_thresh = 0.32;

    /* Initialize camera struct valus */
    camera.pixcnt = 0;
    camera.capcnt = 0;
    memset(camera.scan, 0, sizeof(camera.scan));
    camera.clkval = 0;
    camera.ftm = &camera_ftm;
    camera.adc = &adc;

    /***************************************************************************
     * CONFIGURATION
     **************************************************************************/

    hardware_init();

    /***************************************************************************
     * LOOP
     **************************************************************************/

    while (1) {


        /* Do camera processing */
        if (camera.newscan) {

            /* 
             * Perform noise cleaning
             */
            convolve(&filtered[0], &camera.wbuffer[0], SCAN_LEN, BOXCAR_4, 4);

            /* 
             * Get derivative, either naiively or fancily
             * Put back into normalized because we need two separate buffers
             * A high pass filter seemed to work best as derivative
             */
            //slopify(&normalized[0], &filtered[0], SCAN_LEN);
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, DERIVATIVE, 2);
            convolve(&normalized[0], &filtered[0], SCAN_LEN, HIGH_PASS, 3);
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, DERIV2, 3);

            /* 
             * More noise clean and normalize
             */
            //convolve(&filtered[0], &normalized[0], SCAN_LEN, GAUSS_SMOOTH_7, 7);
            //convolve(&filtered[0], &normalized[0], SCAN_LEN, BOXCAR_5, 5);
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, LOW_PASS5, 5);
            d_normalize(normalized, normalized, SCAN_LEN);

            /*
             * Amplify? Amplify.
             * Causes definite line blobs to clip and enhances the less 
             * pronounced dark line blobs
             */
            amplify(normalized, normalized, SCAN_LEN, 5);

            /* 
             * Threshold for clipped values
             */
            threshold(processed, normalized, SCAN_LEN, 0.90); 

            /* Output to UART if enabled */
            if (DEBUG_CAM) matlab_print();

            /*******************************************************************
             * POSITION CALCULATION
             ******************************************************************/
            //printu("-------------------\r\n");
            //numlines= count_lines(processed, SCAN_LEN);
            //printu("Num lines: %d\n\r", numlines); 

            // Attempt to find left and right line centers
            right_ind = find_blob(processed, SCAN_LEN, RIGHT_VAL);
            left_ind = find_blob(processed, SCAN_LEN, LEFT_VAL);

            right_pos = (double) right_ind / SCAN_LEN;
            left_pos = (double) left_ind / SCAN_LEN;

            // TODO: Check for finish line?
            
            // If we found both lines...
            if (right_ind != -1 && left_ind != -1) {

                // Get each line's distance from the center
                right_d = fabs(0.5 - right_pos);
                left_d = fabs(0.5 - left_pos);

                // Else if right is close to center  
                if (right_d < c_thresh) {
                    // give priority to right line
                    position = 1 - (right_pos - 0.5);
                } 
                else if (left_d < c_thresh) {
                    // give priority to left line
                    position = 0.5 - left_pos;
                } else {
                     // use the center
                    position = 1 - ((right_pos + left_pos) / 2);
                }

                // we found lines; show the blue light
                line_detected = 1;
            } 

            // else if we found either line  
            else if (right_ind + left_ind >= 0) {

                // ... and we have the right one...
                if (right_ind != -1) {
                    position = right_pos + 0.5;
                }
                // ... and we have the left one...
                else if (left_ind != -1) {
                    position = 0.5 - left_pos;
                }
                
                // we found lines; show the blue light
                line_detected = 1;

            } else {
                line_detected = 0;
                //steering = STEER_CENTER;
                //position = 0.5; // go straight 
                // TODO: Keep track of time we haven't seen a line 
            }

            // Allow a new scan to be copied in
            camera.newscan = 0;
        }

        /***********************************************************************
         * STEERING UPDATE  
         **********************************************************************/
       
        /* Record old values */
        old_steer = steering;
        old_err = error;

        /* Update error */
        error = goal-position;

        /* Accumulate error */
        integral += error;
        integral = bound(integral, 0, 1);

        derivative = error - old_err;

        /* P */
        //steering = 0.5 + Kp * error;

        /* PI */
        steering = 
            0.5 + Kp * (error) + 
            Ki * integral;

        /* PID */
        /*steering = 
            0.5 + Kp * (error) + 
            Ki * integral +
            Kd * derivative; */


        /***********************************************************************
         * HARDWARE UPDATE
         **********************************************************************/

        /* Update steering pid */
        if (running) {

            /* Throttle update; only happens when running */

            // constant throttle
            s_throttle = p_throttle = DC_MAX;

            // Linearly proportional to steering
            //s_throttle = p_throttle = (1 - fabs(steering - 0.5))/0.5 * DC_MAX;

            // Exponentially proportional to steering?
            //s_throttle = p_throttle = pow(fabs(steering - 0.5), 2) * DC_MAX;

            // Cap at DC_MAX, just in case
            s_throttle = bound(s_throttle, 0, DC_MAX);
            p_throttle = bound(p_throttle, 0, DC_MAX);

            // update values 
            // TODO: allow for slowing down
            // TODO: update these based on steering
            //if (p_throttle < goal_th) p_throttle += 0.05;
            //if (s_throttle < goal_th) s_throttle += 0.05;

        } else {

            // otherwise slow down
            if (p_throttle > 0) p_throttle -= 0.01;
            if (s_throttle > 0) s_throttle -= 0.01;

            if (p_throttle < 0) p_throttle = 0.00;
            if (s_throttle < 0) s_throttle = 0.00;

            steering = STEER_CENTER;
        }

        /* Motors Update */
        ftm_set_duty(&dc_ftm, CH_STARBOARD, s_throttle);
        ftm_set_duty(&dc_ftm, CH_PORT, p_throttle);

        /* Clip steering within reaonable values */
        steering = bound(steering, 0, 1);
        printu("steering: %d Position * 1000: %d\n\r",  
                (int) (steering * 1000.0), (int) (position * 1000.0));

        ftm_set_duty(&servo_ftm, CH_SERVO, TO_SERVO_DUTY(steering));

        /***********************************************************************
         * STATE AND LED MANAGEMENT 
         **********************************************************************/

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
        if (sw && !button_held) { // debounce, kind of

            running = !running;
            button_held = 1;

            /* Force light to display new state */
            state_color = 1;
            light_elapsed = LIGHT_INT;

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
    if (camera.capcnt >= (400)) {
        // Set SI
        //GPIOC_PCOR |= (1 << 4);
        // send the array over uart
        sprintf(str,"%d\n\r",-2); // start value
        uart_put(str);
        for (i = 0; i < SCAN_LEN - 1; i++) {
            //sprintf(str,"%d\r\n", (int) (camera.wbuffer[i] * 10000));
            //sprintf(str, "%d\r\n", (int) (normalized[i] * 10000.0));
            //sprintf(str,"%d\r\n", (int) (filtered[i] * 100.0));
            sprintf(str,"%d\r\n", processed[i]);
            uart_put(str);
        }
        sprintf(str,"%d\n\r",-3); // end value
        uart_put(str);
        camera.capcnt = 0;
        //GPIOC_PSOR |= (1 << 4);
    }
}

/*
 * Hardware initialization
 */
static void hardware_init() {

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
    //ftm_set_frequency(&camera_ftm, 0, 150k); // wasn't working?
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

}
