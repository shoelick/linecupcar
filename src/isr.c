/*
 * This file will contain all interrupt handlers.
 * THIS FILE MUST BE MANUALLY UPDATED AS IT HAS NO HANDLE 
 * ON MAIN PROGRAM STATE.
 */

#include "camera_driver.h"
#include "MK64F12.h"

/*
 * Handles line scan FTM-driven interrupts
 */
void FTM0_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM0_SC &= ~FTM_SC_TOF_MASK;

	// Toggle clk
	GPIOC_PTOR |= (1<<1);
	camera->clkval = ~(camera->clkval);

    // Handle on ADC value
    uint16_t adcval = camera.adc->regs->RA;

	// Line capture logic
	if ((camera->pixcnt >= 2) && (camera->pixcnt < 256)) {
		if (!camera->clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			camera->scan[camera->pixcnt/2] = adcval;
		}
		camera->pixcnt += 1;
	} else if (camera->pixcnt < 2) {
		if (camera->pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (camera->pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			camera->scan[0] = adcval;
		} 
		camera->pixcnt += 1;
	} else {
		GPIOC_PCOR |= (1 << 1); // CLK = 0
		camera->clkval = 0; // make sure clock variable = 0
		camera->pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM0_SC &= ~(FTM_SC_TOIE_MASK);
	
	}
	return;
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
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		camera->capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM0_MOD = FTM0_MOD;
	
	// Enable FTM2 interrupts (camera)
	FTM0_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


