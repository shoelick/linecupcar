/*
 * ftm_driver.h
 * Driver for the FTMs on KL64.
 */
#ifndef FTM_DRIVER_H
#define FTM_DRIVER_H

#include "MK64F12.h"

#define FTM_ERR_NONE 0
#define FTM_ERR_INIT 1

/* Default value values */
#define DEFAULT_MOD 				32768
#define DEFAULT_CNV                 100

typedef struct ftm_driver {
    FTM_Type *regs;
    IRQn_Type irqn;
} ftm_driver;

/* 
 * Initialized the given FTM number and set the given instance's register
 * handle. Also initializes the hardware.
 */
int ftm_init(ftm_driver *drv, int num);

/*
 * Interrupt control
 */
void ftm_enable_int(ftm_driver *drv);
void ftm_disable_int(ftm_driver *drv);

/*
 * Write protection control 
 * FTM enable does nothing right now, because the current line in it (which
 * is commented out) broke functionality.
 */
void ftm_enable_wr(ftm_driver *drv);
void ftm_disable_wr(ftm_driver *drv);

/* 
 * Enable edge-aligned PWM output from this FTM on the given channel number.
 */
void ftm_enable_pwm(ftm_driver *drv, int ch);

/*
 * Sets the frequency of the FTM interrupts based on the system clock.
 * You must calculate what to use as clock prescaler and multiplier of 
 * prescaler.
 *
 * Remember that compute the necessary prescaler depending on how high you 
 * have to count. Counter max is 65536.
 *
 * Sometimes my set frequency function is finicky in that the MOD register
 * doesn't get set correctly. In the case, I wrote a function to se the mod 
 * directly.
 */ 
void ftm_set_frequency(ftm_driver *drv, int prescaler, int freq);
void ftm_set_mod(ftm_driver *drv, int prescaler, int mod);

/*
 * Sets the duty cycle based on the set frequency.
 * Takes the duty between 0 and 1. (E.g., 50% corresponds to 0.5);
 * DOES NOT take care of pinmuxing output.
 */
void ftm_set_duty(ftm_driver *drv, int ch, double duty);

/*
 * Use this to enable trigger things when FMT counter is equal to CNTIN.
 * We use this to trigger the ADC.
 */
void ftm_enable_cntin_trig(ftm_driver *drv);

#endif
