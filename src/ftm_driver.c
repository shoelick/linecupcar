

#include "MK64F12.h"
#include "ftm_driver.h"

int ftm_init(ftm_driver *drv, int num){
	
	int retval = FTM_ERR_NONE;
	
	/* Protect against null pointer */
	if (drv == 0) {
			return FTM_ERR_INIT;
	}
	
	drv->mod = DEFAULT_MOD;
	drv->mod = DEFAULT_CNV;
	
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
		drv->regs->MODE |= FTM_MODE_WPDIS_MASK;
		
		// Initialize the CNT to 0 before writing to MOD
		drv->regs->CNT = 0;
		
		// Set the period (~10us)
		drv->regs->MOD = 100;
		
		// No prescalar, system clock
		drv->regs->SC = FTM_SC_PS(7)| FTM_SC_CLKS(1);
	}
	
	return retval;
}

void ftm_enable_cntin_trig(ftm_driver *drv) {
	
		// Enable hardware trigger from FTM2
		drv->regs->EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
}

void ftm_enable_pwm(ftm_driver *drv) {
	
		// Set output to '1' on init
		drv->regs->OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
		// Set the Counter Initial Value to 0
		drv->regs->CNTIN = 1;
	
		// Set edge-aligned mode
		drv->regs->CONTROLS[0].CnSC |= FTM_CnSC_MSB_MASK;
	
		// Enable High-true pulses
		// ELSB = 1, ELSA = 0
		drv->regs->CONTROLS[0].CnSC |= FTM_CnSC_ELSB_MASK;
		drv->regs->CONTROLS[0].CnSC &= ~(FTM_CnSC_ELSA_MASK);
	
}

void ftm_set_frequency(ftm_driver *drv, int ch, int prescaler, int mult) {
	
}

void ftm_set_duty(ftm_driver *drv, int ch, double duty) {
	
		drv->regs->CONTROLS[ch].CnV = (int) ((double) (drv->mod)) * duty;
	
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
