/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 * Authors:     Michael Shullick
 * Notes:		
 *
 */

#include <MK64F12.h>
#include <stdio.h>
#include <stdarg.h>
#include <main.h>

#include "uart.h"

char str[100]; /* This is used to print from everywhere */

/* 
 * Initialize UART0 hardware
 */
int uart_init(uart_driver *drv, int num)
{
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    switch (num) {

        case 0:
            drv->regs = UART0; 
            // OpenSDA UART
            PORTB_PCR16 = PORT_PCR_MUX(3); 
            PORTB_PCR17 = PORT_PCR_MUX(3);
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;             
            SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
            break;
        case 1:
            drv->regs = UART1; 
            SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
            break;
        case 2:
            drv->regs = UART2; 
            SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
            break;
        case 3:
            drv->regs = UART3; 
            // PTB10 & PTB11
            SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; 
            PORTB_PCR10 = PORT_PCR_MUX(3); 
            PORTB_PCR11 = PORT_PCR_MUX(3);
            break;
        case 4:
            drv->regs = UART4; 
            SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
            break;
        case 5:
            drv->regs = UART5; 
            SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
            break;
        default:
            return 1;
    }

    /*Configure the UART for establishing serial communication*/

    // Disable transmitter and receiver until proper settings are chosen for the
    // UART module
    drv->regs->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

    //Select default transmission/reception settings for serial communication
    //of UART by clearing the control register 1
    drv->regs->C1 = 0x00;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 ×
    //(SBR[12:0] + BRFD)) 13 bits of SBR are shared by the 8 bits of UART3_BDL
    //and the lower 5 bits of UART3_BDH BRFD is dependent on BRFA, refer Table
    //52-234 in K64 reference manual BRFA is defined by the lower 4 bits of
    //control register, UART0_C4 
    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((DEFAULT_SYSTEM_CLOCK)/(BAUD_RATE * 16));  

    //clear SBR bits of BDH
    drv->regs->BDH &= ~UART_BDH_SBR_MASK;

    //distribute this ubd in BDH and BDL
    drv->regs->BDH |= (ubd >> 8) & UART_BDH_SBR_MASK;
    drv->regs->BDL = ubd & UART_BDL_SBR_MASK;

    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((DEFAULT_SYSTEM_CLOCK*32)/(BAUD_RATE * 16)) / (ubd * 32));

    //write the value of brfa in UART0_C4
    drv->regs->C4 = (UART_C4_BRFA_MASK & brfa);

    //Enable transmitter and receiver of UART
    drv->regs->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

    return 0;
}

/*
 * Read a single character from UART
 */
uint8_t uart_getchar(uart_driver *drv)
{
    /* Wait until there is space for more data in the receiver buffer*/
    while (!(drv->regs->S1 & UART_S1_RDRF_MASK));

    /* Return the 8-bit data from the receiver */
    return drv->regs->D;
}

/*
 * Print a single character to UART
 */
void uart_putchar(uart_driver *drv, char ch)
{
    /* Wait until transmission of previous bit is complete */
    while (!(drv->regs->S1 & UART_S1_TDRE_MASK));

    /* Send the character */
    drv->regs->D = (uint8_t) ch;
}

/* 
 * Print passed string 
 */
void uart_put(uart_driver *drv, char *ptr_str)
{
	while(*ptr_str) uart_putchar(drv, *ptr_str++);
}

/*
 * Variable arg UART print
 */
void printu(uart_driver *drv, char *format, ...) {

    if (!DEBUG_CAM && ENABLE_PRINT) { 
        va_list args;
        va_start(args, format);
        uart_put(drv, str);
        vsprintf(str, format, args);
    }

}
