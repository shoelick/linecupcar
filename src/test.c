
#include "adc_driver.h"
#include "ftm_driver.h"

#define ADC_TEST 0
#define FTM_TEST 1

void FTM2_IRQHandler(void);

int main() {
	
	int test = ADC_TEST;
	
	/* test vars */
	adc_driver adc;
	ftm_driver ftm0;
	int conv;
	
	switch(test) {
		case ADC_TEST:
			ftm_init(&ftm0, 0);
			ftm_enable_cntin_trig(&ftm0);
			adc_enable_int(&adc);
			adc_init(&adc, 0);
			
		  /* Enable ADC0 Alternative Trigger is FTM0 */
			SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(ADC_TRGSEL_FTM0);
			SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;
			SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK); // Pretrigger A
		
			while(1) {
				/* ADC will use ftm0 */
				conv = adc_get_data(&adc);
			}
		case FTM_TEST:
			ftm_init(&ftm0, 2);
			ftm_enable_int(&ftm0);
			while(1) {
				continue;
			};
		default:
			break;
		
		
	}
}

