/*
 * uart.h
 * Provides functionality for printing to UART0.
 * NOT generic, only works with UART connected to USB port on FRDM-KL64F
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

#include "MK64F12.h"

#define BAUD_RATE 9600      //default baud rate 
#define UART0_TXRX_ENABLE 0x00C0

/* 
 * Initialize uart hardware 
 * */
void uart_init(void);

/*
 * Print the passed string to UART
 */
void uart_put(char *ptr_str);

/*
 * Read a single character of input from UART
 */
uint8_t uart_getchar(void);

/*
 * Print a single character to UART
 */
void uart_putchar(char ch);

/* 
 * Variable argument print to UART
 * Use like printf
 */
void printu(char *format, ...);

#endif
