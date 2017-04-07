
#include "adc_driver.h"
#include "ftm_driver.h"

#define ADC_TEST 0
#define FTM_TEST 1

int main() {
	
	int test = FTM_TEST;
	
	/* test vars */
	adc_driver adc;
	ftm_driver ftm;
	int conv;
	
	switch(test) {
		case ADC_TEST:
			adc_init(&adc, 0);
			
			while(1) {
				
				conv = adc_get_data(&adc);
				adc.regs->SC1[0] = adc.regs->SC1[0];
			}
		case FTM_TEST:
			ftm_init(&ftm, 2);
			while(1);
		default:
			break;
		
		
	}
}
