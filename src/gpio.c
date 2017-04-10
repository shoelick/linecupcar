/* 
 * gpio.c
 * This file handles gpio peripherals of LEDs and the pushbutton.
 */

#include "MK64F12.h"                    // Device header
#include "gpio.h"

void led_init(void) {
    // Enable clocks on Ports B, C and E for LED timing
    SIM_SCGC5 = SIM_SCGC5_PORTB_MASK; 
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure the Signal Multiplexer for GPIOode
    // RED	B	22
    PORTB_PCR22 = PORT_PCR_MUX(1);

    // BLU	E	26
    PORTE_PCR26 = PORT_PCR_MUX(1);

    // GRE	B	21
    PORTB_PCR21 = PORT_PCR_MUX(1);

    // Switch the GPIO pins to output mode
    GPIOB_PDDR = (1 << 22) | (1 << 21);
    GPIOE_PDDR = (1 << 26);

    // Turn off the LEDs
    GPIOB_PSOR = (1 << 22) | (1 << 21);
    GPIOE_PSOR = (1 << 26);

}

void button_init(void) {

    // Enable clock for Port C PTC6 button
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure the Mux for the button
    // SWI	C	6
    PORTC_PCR6 = PORT_PCR_MUX(1);

    // Set the push button as an input
    GPIOC_PDDR = (GPIOC_PDDR) & ~(1 << 6);

}

void set_led(color_t color) {
    switch(color) {
        case RED:
            // Red
            GPIOB_PCOR = 1UL << 22;
            GPIOE_PSOR = 1UL << 26;
            GPIOB_PSOR = 1UL << 21;
            break;
        case GREEN:
            // Green
            GPIOB_PSOR = 1UL << 22;
            GPIOE_PCOR = 1UL << 26;
            GPIOB_PSOR = 1UL << 21;
            break;
        case BLUE:
            // Blue
            GPIOB_PSOR = 1UL << 22;
            GPIOE_PSOR = 1UL << 26;
            GPIOB_PCOR = 1UL << 21;
            break;
        case CYAN:
            GPIOB_PSOR = 1UL << 22;
            GPIOE_PCOR = 1UL << 26;
            GPIOB_PCOR = 1UL << 21;
            break;
        case MAGENTA:
            GPIOB_PCOR = 1UL << 22;
            GPIOE_PSOR = 1UL << 26;
            GPIOB_PCOR = 1UL << 21;
            break;
        case YELLOW:
            GPIOB_PCOR = 1UL << 22;
            GPIOE_PCOR = 1UL << 26;
            GPIOB_PSOR = 1UL << 21;
            break;
        case OFF:
            GPIOB_PSOR = 1UL << 22;
            GPIOE_PSOR = 1UL << 26;
            GPIOB_PSOR = 1UL << 21;
            break;
        default:
            break;
    }
}

int sw_active() {
    return (GPIOC_PDIR & (1 << 6)) == 0;
}

