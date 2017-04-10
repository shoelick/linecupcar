/*
 * This file will contain all interrupt handlers.
 * THIS FILE MUST BE MANUAL UPDATED AS IT HAS NO HANDLE 
 * ON MAIN PROGRAM STATE.
 */
 
#include "MK64F12.h"

void FTM0_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM0_SC &= ~FTM_SC_TOF_MASK;

	// Setting mod resets the FTM counter
	FTM0_MOD = 100;
}

void FTM2_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM2_SC &= ~FTM_SC_TOF_MASK;

	// Setting mod resets the FTM counter
	FTM2_MOD = FTM2_MOD;
}

void FTM3_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM3_SC &= ~FTM_SC_TOF_MASK;
	
	// Setting mod resets the FTM counter
	FTM3_MOD = FTM3_MOD;
}

// ADC1 Conversion Complete ISR
void ADC0_IRQHandler(void) {
    // Read the result (upper 12-bits). This also clears the Conversion complete flag.
    unsigned short i = ADC0_RA >> 4;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2_MOD = 100;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}*/


