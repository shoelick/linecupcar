/* 
 * gpio.h
 * This file handles gpio peripherals of LEDs and the pushbutton on the 
 * FRDM-KL64F.
 */

#ifndef GPIO_H
#define GPIO_H 

/* Set color variations */
typedef enum {RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, OFF} color_t;

/* 
 * Initialize various peripherals 
 */
void button_init(void);
void led_init(void);

/* 
 * Set LED color 
 */
void set_led(color_t color);

/* 
 * Returns whether the switch is PRESSED
 */
int sw_active(void);

#endif 
