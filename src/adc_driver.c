/* 
 * adc_driver.c
 * Driver for ADC on the KL46Z. See header for details.
 */

#include "../cmsis/MK64F12/MK64F12.h"
#include "adc_driver.h"

#define NULL    0

#define ADC_ERR_INIT   1

int init_adc(adc_driver *driver, int adc_num) {

    int retval = 0;
    unsigned int calib = 0;

    /* Protect against null pointer */
    if (driver == 0) {
        return ADC_ERR_INIT;
    }

    /* initalize the appropriate driver map */
    switch (adc_num) {

        case (0):
            driver->regs = ADC0; 
            driver->irqn = ADC0_IRQn;

            // Turn on ADC0
            SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
            break;
        case (1):
            driver->regs = ADC1;
            driver->irqn = ADC1_IRQn;

            // Turn on ADC1
            SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
	

            break;
        default:
            driver->regs = NULL;
            retval = -1;
            break;
    }

    if (retval == ADC_ERR_NONE) {
        /* Set default configuration */

        /* Do calibration */
        driver->regs->SC3 = ADC_SC3_CAL_MASK;
        while ( (driver->regs->SC3 & ADC_SC3_CAL_MASK) != 0 );
        calib = driver->regs->CLP0; 
        calib += driver->regs->CLP1; 
        calib += driver->regs->CLP2;
        calib = calib >> 1; calib |= 0x8000;
        driver->regs->PG = calib;

        // Select hardware trigger.
        driver->regs->SC2 |= ADC_SC2_ADTRG_MASK;
        
        // Set to single ended mode	
		driver->regs->SC1[0] |= ADC_SC1_AIEN_MASK; 
		driver->regs->SC1[0] &= ~(ADC_SC1_ADCH_MASK);
		driver->regs->SC1[0] |= 0x1;
	
		// Set up FTM2 trigger on ADC0
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA); // FTM2 select
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
		SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK); // Pretrigger A
    }

    return retval;
}

void enable_interrupt(adc_driver *driver) {
	
		// Enable NVIC interrupt
        NVIC_EnableIRQ(driver->irqn);
}

uint32_t get_data(adc_driver *driver) {

    return (driver->regs->R[1] << 16) | driver->regs->R[0];
}
