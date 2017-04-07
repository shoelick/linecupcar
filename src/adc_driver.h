/* 
 * adc_driver.c
 * Driver for ADC on the KL46Z. 
 * Depends on existing headers for the KL46Z.
 * Includes the functionality below.
 */

const int ADC_ERR_NONE = 0;

typedef struct  {
    ADC_Type *regs;
    IRQn_Type irqn; 
} adc_driver;

/* Initializes an adc driver instance using the given number adc */
int init_adc(adc_driver *driver, int adc_num);

void enable_interrupt(adc_driver *driver);

uint32_t get_data(adc_driver *driver);
