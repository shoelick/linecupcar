/* 
 * adc_driver.c
 * Driver for ADC on the KL46Z. 
 * Depends on existing headers for the KL46Z.
 * Includes the functionality below.
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "MK64F12.h"

#define ADC_ERR_NONE 0
#define ADC_ERR_INIT 1

#define ADC_TRGSEL_FTM0 8

typedef struct adc_driver {
    ADC_Type *regs;
    IRQn_Type irqn; 
} adc_driver;

/* Initializes an adc driver instance using the given number adc */
int adc_init(adc_driver *driver, int adc_num);

void adc_enable_int(adc_driver *driver);

uint32_t adc_get_data(adc_driver *driver);

#endif
