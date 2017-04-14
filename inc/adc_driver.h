/* 
 * adc_driver.c
 * Driver for ADC on the KL46Z. 
 * Depends on existing headers for the KL46Z. Includes the functionality below.
 * This driver is hardcoded to enable channel 1 on whatever
 * adc number is passed.
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "MK64F12.h"

#define ADC_ERR_NONE 0
#define ADC_ERR_INIT 1

/* Reg val for choosing FTM0 as trigger */
#define ADC_TRGSEL_FTM0 8

typedef struct adc_driver {
    ADC_Type *regs;
    IRQn_Type irqn; 
} adc_driver;

/* 
 * Initializes an adc driver instance using the given number adc 
 */
int adc_init(adc_driver *driver, int adc_num);

/* 
 * Enables interrupts on the passed adc instance.
 * Requires driver to be initialized with adc_init
 */
void adc_enable_int(adc_driver *driver);

/*
 * Grabs the content of the ADC data regs
 * NEEDS TO BE TESTED
 * Usually just grabbed data register value directly
 */
uint32_t adc_get_data(adc_driver *driver);

/* 
 * Enables the passed ADC to be triggered using hardware timer FTM0
 */
void adc_set_ftm0_trig(adc_driver *driver);

#endif
