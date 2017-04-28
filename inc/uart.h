/*
 * uart.h
 * Provides functionality for printing to UART0.
 * NOT generic, only works with UART connected to USB port on FRDM-KL64F
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

#include "MK64F12.h"
extern char str[100];

#define BAUD_RATE 9600      //default baud rate 
#define UART0_TXRX_ENABLE 0x00C0
#define BUFLEN 240

typedef struct uart_driver {
    UART_Type *regs;
    IRQn_Type irq;
    char buffer[BUFLEN]; // print up to three full lines
} uart_driver;

/* 
 * Initialize uart hardware 
 * */
int uart_init(uart_driver *drv, int num);

/*
 * Print the passed string to UART
 */
void uart_put(uart_driver *drv, char *ptr_str);

/*
 * Read a single character of input from UART
 */
uint8_t uart_getchar(uart_driver *drv);

/*
 * Print a single character to UART
 */
void uart_putchar(uart_driver *drv, char ch);

/* 
 * Variable argument print to UART
 * Use like printf
 */
void printu(uart_driver *drv, char *format, ...);

void print_serial(uart_driver *drv, char *format, ...);
#endif
