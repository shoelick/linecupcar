

#include "MK64F12.h"
#include "ftm_driver.h"

int ftm_init(ftm_driver *drv, int num){
	
	int retval = FTM_ERR_NONE;
	
	/* Protect against null pointer */
    if (drv == 0) {
        return FTM_ERR_INIT;
    }
	
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
		default: 
			retval = FTM_ERR_INIT;
			break;
	}

	if (retval == FTM_ERR_NONE) {

		// Disable Write Protection
		drv->regs->MODE |= FTM_MODE_WPDIS_MASK;
		
		// Set output to '1' on init
		//drv->regs->OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
		
		// Initialize the CNT to 0 before writing to MOD
		drv->regs->CNT = 0;
		
		// Set the Counter Initial Value to 0
		//drv->regs->CNTIN = 0;
		
		// Set the period (~10us)
		drv->regs->MOD = 100;
		
		// Set edge-aligned mode
		//drv->regs->CONTROLS[0].CnSC |= FTM_CnSC_MSB_MASK;
		
		// Enable High-true pulses
		// ELSB = 1, ELSA = 0
		//drv->regs->CONTROLS[0].CnSC |= FTM_CnSC_ELSB_MASK;
		//drv->regs->CONTROLS[0].CnSC &= ~(FTM_CnSC_ELSA_MASK);
		
		// 50% duty
		//drv->regs->CONTROLS[0].CnV = 50;
		
		// Enable hardware trigger from FTM2
		//drv->regs->EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
		
		// Don't enable interrupts yet (disable)
		//drv->regs->SC &= ~(FTM_SC_TOIE_MASK);
		
		// No prescalar, system clock
		drv->regs->SC = FTM_SC_PS(7)| FTM_SC_CLKS(1);
	}
	
	return retval;
}

void ftm_enable_int(ftm_driver *drv) {
	
	// Enable FTM2 interrupts (camera)
	drv->regs->SC |= FTM_SC_TOIE_MASK;
	
	// Set up interrupt
	NVIC_EnableIRQ(drv->irqn);
	
}
