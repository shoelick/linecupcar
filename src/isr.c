/*
 * This file will contain all interrupt handlers.
 */

#include <MK64F12.h>
#include <string.h>

#include "camera_driver.h"
#include "main.h"

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

/*
 * Handles line scan FTM-driven interrupts
 */
void FTM0_IRQHandler(void){ //For FTM timer
	
	// Clear interrupt
	FTM0_SC &= ~FTM_SC_TOF_MASK;

	// Toggle clk
	GPIOC_PTOR |= (1<<1);
	camera.clkval = ~(camera.clkval);

	// Line capture logic
	if ((camera.pixcnt >= 2) && (camera.pixcnt < 256)) {
		if (!camera.clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			camera.scan[camera.pixcnt/2] = ADC0VAL;
		}
		camera.pixcnt += 1;
	} else if (camera.pixcnt < 2) {
		if (camera.pixcnt == -1) {
			GPIOC_PSOR |= (1 << 4); // SI = 1
		} else if (camera.pixcnt == 1) {
			GPIOC_PCOR |= (1 << 4); // SI = 0
			// ADC read
			camera.scan[0] = ADC0VAL;
		} 
		camera.pixcnt += 1;
	} else {
        
		GPIOC_PCOR |= (1 << 1); // CLK = 0
		camera.clkval = 0; // make sure clock variable = 0
		camera.pixcnt = -2; // reset counter
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
    ADC0VAL = ADC0_RA;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){

    size_t i;

	if (DEBUG_CAM) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		camera.capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    /* Only move new scan into working buffer if existing scan is unprocessed */
    if (!camera.newscan) {

        //memcpy(camera.wbuffer, camera.scan, SCAN_LEN * sizeof(int));
        for (i = 0; i < SCAN_LEN; i++) {
            // Copy in everything as a double
            camera.wbuffer[i] = (int) camera.scan[i];
        }
        camera.newscan = 0;
    }

	// Setting mod resets the FTM counter
	//FTM0_MOD = FTM0_MOD;
	FTM0_MOD = 100;
	
	// Enable FTM2 interrupts (camera)
	FTM0_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


