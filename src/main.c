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
 *  Starboard motor PWM FWD ---- PTD0  (FTM3_CH0) [EXT] 
 *  Starboard motor PWM BWD ---- PTC8  (FTM3_CH4) [EXT] 
 *  Port motor PWM FWD --------- PTD1  (FTM3_CH1) [EXT]
 *  Port motor PWM BWD --------- PTC9  (FTM3_CH5) [EXT]
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
#include "rollqueue.h"
#include "stdlib.h"
#include "uart.h"
#include "util.h"

/* Used to debug camera processing */
static void matlab_print(uart_driver *drv);
static void hardware_init();

unsigned long DEFAULT_SYSTEM_CLOCK = 20485760U;

camera_driver camera; /* Externally defined for use in ISRs */

/* Line scan processing buffers */
double normalized[SCAN_LEN];
double filtered[SCAN_LEN];
int processed[SCAN_LEN];

//const double SERVO_MIN  = 0.0525;
const double SERVO_MIN  = 0.05;
const double SERVO_MAX  = 0.097;
const double STEER_CENTER = 0.55;

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
const double DC_MAX = 0.50;
//const double DC_MAX = 0.3;
const double DC_MIN = 0.35;

/* PID Constants */
const double Kp = 0.73, Ki = 0.05, Kd = 0.8;

/* FTM Channels */
const int CH_SERVO = 0;
const int STAR_CH_FWD = 0;
const int STAR_CH_BACK = 4;
const int PORT_CH_FWD = 1;
const int PORT_CH_BACK = 5;

/* Goal bounds */
const double LEFT_BOUND = 0.1;
const double RIGHT_BOUND = 0.9;

ftm_driver dc_ftm, servo_ftm, camera_ftm;
adc_driver adc;
uart_driver usb_uart, bt_uart;

int main() {

    /***************************************************************************
     * VARIABLES AND INSTANCES
     **************************************************************************/

    /* Setpoint values */
    double s_throttle = 0.0, p_throttle = 0.0; // current fwd
    double sbwd_throttle = 0.0, pbwd_throttle = 0.0; // current bwd
    double sg_throttle = 0.0, pg_throttle = 0.0;  // goal
    double steering = STEER_CENTER;

    /* old steering and error vars for integral control */
    double error;

    /* state management */
    int8_t running = 0, sw;
    uint8_t button_held = 0, state_color = 1, line_detected = 0;
    const long int LIGHT_INT = 500;
    long int light_elapsed = 0;
    uint8_t sys_error = 0;

    /* Position tracking */
    //int center = 0;
    double position, derivative;
    double goal = 0.58; // for now, let's stick to staying the middle 
    rollqueue steer_hist, error_hist;

    /* 
     * Using the derivative, we end up producing different values for 
     * dark-to-light and light-to-dark, corresponding to left and right
     * respectively.
     */
    const int RIGHT_VAL = 1;
    const int LEFT_VAL = -1;
    int right_ind, left_ind;
    double right_pos, left_pos;
    int lineless_cycles = 0;

    /* Initialize camera struct valus */
    camera.pixcnt = 0;
    camera.capcnt = 0;
    memset(camera.scan, 0, sizeof(camera.scan));
    camera.clkval = 0;
    camera.ftm = &camera_ftm;
    camera.adc = &adc;

    /* Initialize rolling steer history */
    sys_error = init_rollqueue(&steer_hist, 4);
    sys_error = init_rollqueue(&error_hist, 8);

    /***************************************************************************
     * CONFIGURATION
     **************************************************************************/

    hardware_init();

    /***************************************************************************
     * LOOP
     **************************************************************************/

    while (!sys_error) {


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
            convolve(filtered, normalized, SCAN_LEN, BOXCAR_4, 4);
            //convolve(&normalized[0], &filtered[0], SCAN_LEN, LOW_PASS5, 5);
            d_normalize(normalized, filtered, SCAN_LEN);

            /*
             * Amplify? Amplify.
             * Causes definite line blobs to clip and enhances the less 
             * pronounced dark line blobs
             */
            amplify(normalized, normalized, SCAN_LEN, 7.2);

            /* 
             * Threshold for clipped values
             */
            threshold(processed, normalized, SCAN_LEN, 0.90); 

            /* Output to UART if enabled */
            if (DEBUG_CAM) matlab_print(&bt_uart);

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
                /*right_d = fabs(0.5 - right_pos);
                left_d = fabs(0.5 - left_pos);

                // Else if right is close to center  
                if (right_d < c_thresh) {
                    // give priority to right line
                    position = (0.5 + right_pos);
                } 
                else if (left_d < c_thresh) {
                    // give priority to left line
                    position = (left_pos - 0.5);
                } else {
                     // use the center
                    position = 1 - ((right_pos + left_pos) / 2);
                }*/

                position = 2.0 * (right_pos + left_pos) / 2.0 - 0.5;
                //position = ((right_pos + left_pos) / 2);

                // we found lines; show the blue light
                line_detected = 1;
                lineless_cycles = 0;
            } 

            // else if we found either line  
            else if (right_ind + left_ind >= 0) {

                // ... and we have the right one...
                if (right_ind != -1) {
                    position = right_pos + goal;
                }
                // ... and we have the left one...
                else if (left_ind != -1) {
                    position = left_pos - goal;
                }
                
                // we found lines; show the blue light
                line_detected = 1;
                lineless_cycles = 0;

            } else {
                line_detected = 0;
                lineless_cycles++;
                if (lineless_cycles > 4) {
                    steering = STEER_CENTER;
                    position = 0.5; // go straight 
                }
                // TODO: Keep track of time we haven't seen a line 
            }

            // Allow a new scan to be copied in
            camera.newscan = 0;
        }

        /***********************************************************************
         * STEERING UPDATE  
         **********************************************************************/
       
        /* Record old values */
        /*old_err = error;*/

        /* Update error */
        error = goal-position;
        add_data(&error_hist, error);

        /* Accumulate error */
        /*integral += error;
        integral = bound(integral, 0, 1);*/

        /*derivative = error - old_err;*/

        /* P */
        //steering = 0.5 + Kp * pow(error, 2);
        //steering = 0.5 + Kp * error;
        steering = 0.5 + Kp * get_average(&error_hist);

        /* PI */
        /*steering = 
            0.5 + Kp * (error) + 
            Ki * get_average(&error_hist);*/

        /* PID */
        /*steering = 
            0.5 + Kp * (error) + 
            Ki * integral +
            Kd * derivative; */

        /* PD? */
        /*derivative = get_ending_slope(&error_hist, 3);
        steering = 
            //0.5 + Kp * error + 
            0.5 + Kp * error + 
            Kd * derivative;*/

        /*goal = (RIGHT_BOUND - LEFT_BOUND) * (get_average(&steer_hist)) + \
            LEFT_BOUND;*/
        add_data(&steer_hist, steering);

        /***********************************************************************
         * THROTTLE UPDATE
         **********************************************************************/

        /* Update steering pid */
        if (running) {

            /* Throttle update; only happens when running */

            // constant throttle
            //sg_throttle = pg_throttle = DC_MAX;

            // Linearly proportional to steering
            sg_throttle = pg_throttle = (DC_MAX-DC_MIN) * \
                (1 - (fabs(steering - 0.5) / 0.5) ) + DC_MIN;

            // Exponentially proportional to steering?
            //s_throttle = p_throttle = pow(fabs(steering - 0.5), 2) * DC_MAX;

            pbwd_throttle = sbwd_throttle = 0.0;

            /*if (rollqueue_max(&steer_hist) > 0.65) {
            
                sg_throttle = 0;
                pg_throttle = 0.8;
                sbwd_throttle = 0.4;

            } else if (rollqueue_max(&steer_hist) < 0.35) {

                pg_throttle *= 0;
                sg_throttle = 0.8;
                pbwd_throttle = 0.4;

            }*/

            // Cap at DC_MAX, just in case
            sg_throttle = bound(sg_throttle, 0, DC_MAX);
            pg_throttle = bound(pg_throttle, 0, DC_MAX);

            // update values 
            //p_throttle += (pg_throttle - p_throttle) * 0.1;
            //s_throttle += (sg_throttle - s_throttle) * 0.1;
            s_throttle = sg_throttle;
            p_throttle = pg_throttle;


        } else {

            // otherwise slow down
            //if (p_throttle > 0) p_throttle -= 0.01;
            //if (s_throttle > 0) s_throttle -= 0.01;

            //if (p_throttle < 0) p_throttle = 0.00;
            //if (s_throttle < 0) s_throttle = 0.00;
            
            p_throttle = s_throttle = 0.0;
            pbwd_throttle = sbwd_throttle = 0.0;

            steering = STEER_CENTER;
        }

        /***********************************************************************
         * HARDWARE UPDATE
         **********************************************************************/
        /* Motors Update */
        ftm_set_duty(&dc_ftm, STAR_CH_FWD, s_throttle);
        ftm_set_duty(&dc_ftm, PORT_CH_FWD, p_throttle);
        ftm_set_duty(&dc_ftm, STAR_CH_BACK, sbwd_throttle);
        ftm_set_duty(&dc_ftm, PORT_CH_BACK, pbwd_throttle);

        /* Clip steering within reaonable values */
        steering = bound(steering, 0, 1);
        ftm_set_duty(&servo_ftm, CH_SERVO, TO_SERVO_DUTY(steering));

        /***********************************************************************
         * STATUS REPORT
         **********************************************************************/

        if (ENABLE_PRINT) {
            /*print_serial(&bt_uart, \
                    "Goal: %5d Position: %5d Steer: %5d " \
                    "Port Throttle (actual / goal): %5d / %-5d " \
                    "Starboard Throttle (actual / goal): %5d / %-5d \r\n",
                    (int) (goal * 1000), (int) (position * 1000), (int) (steering * 1000), 
                    (int) (p_throttle * 1000), (int) (pg_throttle * 1000),
                    (int) (s_throttle * 1000), (int) (sg_throttle * 1000));*/

            /*print_serial(&bt_uart, \
                    "Position: %d Steering: %d Error avg: %d Derivative: %d\n\r",
                    (int) (1000 * position), (int) (1000 * steering),
                    (int) (1000 * get_average(&error_hist)), 
                    (int) (1000 * derivative));*/

            print_serial(&bt_uart, \
                    "Left pos: %d Right pos: %d Position: %d Error: %d\n\r", 
                    (int) (left_pos * 1000), (int) (right_pos * 1000),
                    (int) (position * 1000), (int) (error * 1000));
        }

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
static void matlab_print(uart_driver *drv) { 
    
    int i; 

    //if (capcnt >= (2/INTEGRATION_TIME)) {
    if (camera.capcnt >= (400)) {
        // Set SI
        //GPIOC_PCOR |= (1 << 4);
        // send the array over uart
        sprintf(str,"%d\n\r",-2); // start value
        uart_put(drv, str);
        //printu(&bt_uart,"%d\n\r",-2); // start value
        for (i = 0; i < SCAN_LEN - 1; i++) {
            sprintf(str,"%d\r\n", (int) (camera.wbuffer[i] * 10000));
            //sprintf(str,"%d\r\n", (int) (filtered[i] * 100.0));
            //sprintf(str, "%d\r\n", (int) (normalized[i] * 1000.0));
            //sprintf(str,"%d\r\n", processed[i]);
            uart_put(drv, str);
        }
        //printu(&bt_uart,"%d\n\r",-3); // start value
        sprintf(str,"%d\n\r",-3); // end value
        uart_put(drv, str);
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
    PORTD_PCR0 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; // Starboard Forward
    PORTD_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; // Port Forward 
    PORTC_PCR8 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Starboard backward
    PORTC_PCR9 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Port backward

    PORTB_PCR18 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Servo

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
    uart_init(&usb_uart, 0);
    uart_init(&bt_uart, 3);

    // Configure DC FTM
    ftm_init(&dc_ftm, 3); /* use ftm3 for dc motors */
    ftm_set_frequency(&dc_ftm, 0, 10e3);
    ftm_enable_pwm(&dc_ftm, STAR_CH_FWD);
    ftm_enable_pwm(&dc_ftm, STAR_CH_BACK);
    ftm_enable_pwm(&dc_ftm, PORT_CH_FWD);
    ftm_enable_pwm(&dc_ftm, PORT_CH_BACK);
    ftm_set_duty(&dc_ftm, STAR_CH_FWD, 0);
    ftm_set_duty(&dc_ftm, STAR_CH_BACK, 0);
    ftm_set_duty(&dc_ftm, PORT_CH_FWD, 0);
    ftm_set_duty(&dc_ftm, PORT_CH_BACK, 0);
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
