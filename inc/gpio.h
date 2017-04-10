/* 
 * gpio.h
 * This file handles gpio peripherals of LEDs and the pushbutton.
 */

typedef enum {RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, OFF} color_t;

void button_init(void);
void led_init(void);
void set_led(color_t color);
int sw_active(void);

