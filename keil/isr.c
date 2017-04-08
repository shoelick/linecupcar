/*
 * This file will contain all interrupt handlers.
 * THIS FILE MUST BE MANUAL UPDATED AS IT HAS NO HANDLE 
 * ON MAIN PROGRAM STATE.
 */
 
#include "MK64F12.h"

void FTM2_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM2_SC &= ~FTM_SC_TOF_MASK;

	// Setting mod resets the FTM counter
	FTM2_MOD = 100;
}

void FTM0_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM0_SC &= ~FTM_SC_TOF_MASK;

	// Setting mod resets the FTM counter
	FTM0_MOD = 100;
}

// ADC1 Conversion Complete ISR
void ADC0_IRQHandler(void) {
    // Read the result (upper 12-bits). This also clears the Conversion complete flag.
    unsigned short i = ADC0_RA >> 4;
}
