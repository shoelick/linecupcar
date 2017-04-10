

#include "MK64F12.h"
#include <math.h>
#include "ftm_driver.h"

#define DEFAULT_SYSTEM_CLOCK 20485760U

int ftm_init(ftm_driver *drv, int num){

    int retval = FTM_ERR_NONE;

    /* Protect against null pointer */
    if (drv == 0) {
        return FTM_ERR_INIT;
    }

    drv->duty[0] = DEFAULT_CNV;

    switch(num) {

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

void ftm_enable_cntin_trig(ftm_driver *drv) {

    // Enable hardware trigger from FTM2
    drv->regs->EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

}

void ftm_enable_pwm(ftm_driver *drv, int ch) {

    ftm_enable_wr(drv);

    // Set output to '1' on init
    drv->regs->OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

    // Set the Counter Initial Value to 0
    drv->regs->CNTIN = 1;

    // Set edge-aligned mode
    drv->regs->CONTROLS[ch].CnSC |= FTM_CnSC_MSB_MASK;

    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    drv->regs->CONTROLS[ch].CnSC |= FTM_CnSC_ELSB_MASK;
    drv->regs->CONTROLS[ch].CnSC &= ~(FTM_CnSC_ELSA_MASK);

    ftm_disable_wr(drv);

}

void ftm_set_frequency(ftm_driver *drv, int prescaler, int freq) {

    // Disable Write Protection
    ftm_enable_wr(drv);

    // Set prescaler value
    drv->regs->SC &= ~FTM_SC_PS_MASK;
    drv->regs->SC |= FTM_SC_PS(prescaler);

    prescaler = (int) powl(2,prescaler);

    // Set the mod value according to the desired frequency
    drv->regs->CNT = 0;

    drv->regs->MOD = DEFAULT_SYSTEM_CLOCK / prescaler / freq;

    // Enable write protection
    ftm_disable_wr(drv);

}

void ftm_set_duty(ftm_driver *drv, int ch, double duty) {

    drv->regs->CONTROLS[ch].CnV = (int) ((double) (drv->regs->MOD)) * duty;

}

void ftm_disable_int(ftm_driver *drv) {

    // Don't enable interrupts yet (disable)
    drv->regs->SC &= ~(FTM_SC_TOIE_MASK);

}

void ftm_enable_int(ftm_driver *drv) {

    // Enable FTM2 interrupts (camera)
    drv->regs->SC |= FTM_SC_TOIE_MASK;

    // Set up interrupt
    NVIC_EnableIRQ(drv->irqn);

}

void ftm_disable_wr(ftm_driver *drv) {

    // Enable write protection
    //drv->regs->FMS |= FTM_FMS_WPEN_MASK;
}

void ftm_enable_wr(ftm_driver *drv) {

    // Disable Write Protection
    drv->regs->MODE |= FTM_MODE_WPDIS_MASK;
}
