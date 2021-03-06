/*
 * ftm_driver.h
 * Driver for the FTMs on KL64.
 */

#include <math.h>
#include <stdio.h>

#include "MK64F12.h"
#include "main.h" /* Need for system clock */
#include "ftm_driver.h"
#include "uart.h"
#include "util.h"

/*
 * Initialize an FTM
 */
int ftm_init(ftm_driver *drv, int num){

    int retval = FTM_ERR_NONE;

    /* Protect against null pointer */
    if (drv == 0) {
        return FTM_ERR_INIT;
    }

    switch(num) {

        /* Set up according to passed FTM number */
        case 0:
            drv->regs = FTM0;
            drv->irqn = FTM0_IRQn;

            // Enable clock
            SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

            break;
        case 1:
            drv->regs = FTM1;
            drv->irqn = FTM1_IRQn;

            // Enable clock
            SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

            break;
        case 2:
            drv->regs = FTM2;
            drv->irqn = FTM2_IRQn;

            // Enable clock
            SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

            break;
        case 3:
            drv->regs = FTM3;
            drv->irqn = FTM3_IRQn;

            // Enable clock
            SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
            break;
        default: 
            retval = FTM_ERR_INIT;
            break;
    }

    if (retval == FTM_ERR_NONE) {

        // Disable Write Protection
        ftm_enable_wr(drv);

        // Set output to '1' on init
        drv->regs->OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

        // Set the Counter Initial Value to 0
        drv->regs->CNTIN = 0;

        // Initialize the CNT to 0 before writing to MOD
        drv->regs->CNT = 0;

        // Set the period (~10us)
        drv->regs->MOD = DEFAULT_MOD;

        // No prescalar, system clock
        drv->regs->SC = FTM_SC_PS(0)| FTM_SC_CLKS(1);

        // Enable write protection
        ftm_disable_wr(drv);
    }

    return retval;
}

/* 
 * Used to use this FTM for triggering ADC
 */
void ftm_enable_cntin_trig(ftm_driver *drv) {

    // Enable hardware trigger from FTM2
    drv->regs->EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

}

/*
 * Enable PWM output with passed channel
 */
void ftm_enable_pwm(ftm_driver *drv, int ch) {

    ftm_enable_wr(drv);

    // Set edge-aligned mode
    drv->regs->CONTROLS[ch].CnSC |= FTM_CnSC_MSB_MASK;

    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    drv->regs->CONTROLS[ch].CnSC |= FTM_CnSC_ELSB_MASK;
    drv->regs->CONTROLS[ch].CnSC &= ~(FTM_CnSC_ELSA_MASK);

    ftm_disable_wr(drv);

}

/* 
 * Attempt to set the frequency of this ftm according to params.
 * Finicky
 */
void ftm_set_frequency(ftm_driver *drv, int prescaler, int freq) {

    // Disable Write Protection
    ftm_enable_wr(drv);

    // Set prescaler value
    drv->regs->SC &= ~FTM_SC_PS_MASK;
    drv->regs->SC |= FTM_SC_PS(prescaler);

    // Calculate corresponding mod value
    prescaler = int_pow(2,prescaler);

    // Set the mod value according to the desired frequency
    drv->regs->CNT = 0;
    drv->regs->MOD = DEFAULT_SYSTEM_CLOCK / prescaler / freq;

    // Enable write protection
    ftm_disable_wr(drv);

    // Allow mod value to get latched in
    delay(1);
}

/* 
 * Directly set mod and prescaler values
 */
void ftm_set_mod(ftm_driver *drv, int prescaler, int mod) {

    // Disable Write Protection
    ftm_enable_wr(drv);

    // Set prescaler value
    drv->regs->SC &= ~FTM_SC_PS_MASK;
    drv->regs->SC |= FTM_SC_PS(prescaler);

    // Set the mod value according to the desired frequency
    drv->regs->CNT = 0;
    drv->regs->MOD = mod; 

    // Enable write protection
    ftm_disable_wr(drv);

    // Allow mod value to get latched in
    delay(1);
}

/* 
 * Set the duty output of the passed ftm channel
 */
void ftm_set_duty(ftm_driver *drv, int ch, double duty) {

    int cnv = ((double) (drv->regs->MOD)) * duty;

    // Don't allow a negative value
    drv->regs->CONTROLS[ch].CnV = (cnv < 0) ? 0 : cnv;

}

/*
 * Disable FTM interrupts
 */
void ftm_disable_int(ftm_driver *drv) {

    // Don't enable interrupts yet (disable)
    drv->regs->SC &= ~(FTM_SC_TOIE_MASK);

}

/*
 * Enable FTM interrupts 
 */
void ftm_enable_int(ftm_driver *drv) {

    // Enable interrupts 
    drv->regs->SC |= FTM_SC_TOIE_MASK;

    // Set up interrupt
    NVIC_EnableIRQ(drv->irqn);

}

/*
 * Handling FTM write protection
 */
void ftm_disable_wr(ftm_driver *drv) {

    // Enable write protection
    //drv->regs->FMS |= FTM_FMS_WPEN_MASK;
}

void ftm_enable_wr(ftm_driver *drv) {

    // Disable Write Protection
    drv->regs->MODE |= FTM_MODE_WPDIS_MASK;
}

