
#ifndef FTM_DRIVER_H
#define FTM_DRIVER_H

#include "MK64F12.h"

#define FTM_ERR_NONE 0
#define FTM_ERR_INIT 1

/* Default System clock value */
#define DEFAULT_SYSTEM_CLOCK 20485760u 
#define DEFAULT_MOD 				 32768
#define DEFAULT_CNV					 128 

typedef struct ftm_driver {
    FTM_Type *regs;
    IRQn_Type irqn;
		int mod;
	  int duty;
} ftm_driver;

int ftm_init(ftm_driver *drv, int num);
void ftm_enable_int(ftm_driver *drv);
void ftm_disable_int(ftm_driver *drv);

/*
 * Sets the frequency of the FTM interrupts based on the system clock.
 * You must calculate what to use as clock prescaler and multiplier of 
 * prescaler.
 */ 
void ftm_set_frequency(ftm_driver *drv, int ch, int prescaler, int mult);

/*
 * Sets the duty cycle based on the set frequency.
 * Takes the duty between 0 and 1. (E.g., 50% corresponds to 0.5);
 */
void ftm_set_duty(ftm_driver *drv, int ch, double duty);

/*
 * Use this to enable trigger things when FMT counter is equal to CNTIN.
 * We use this to trigger ADC.
 */
void ftm_enable_cntin_trig(ftm_driver *drv);
#endif
