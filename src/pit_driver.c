/*
 * Driver for the periodic interrupt timer.
 * See header for details.
 */

#include "MK64F12.h"

// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
// (camera clk is the mod value set in FTM)
#define INTEGRATION_TIME .0075f

#define DEFAULT_SYSTEM_CLOCK 20485760U

/* 
 * Initialization of PIT timer to control 
 * 		integration period
 */
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR = PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = (uint32_t) (INTEGRATION_TIME * DEFAULT_SYSTEM_CLOCK);
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; 
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


