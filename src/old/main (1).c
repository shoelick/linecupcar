/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Spring 2016
*
* Filename: main_timer_template.c
*/

#include "MK64F12.h"
#include "uart.h"
#include "isr.h"
#include <stdio.h>

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK 20485760u /* Default System clock value */

void initPDB(void);
void initGPIO(void);
void initFTM(void);
void initInterrupts(void);
void LED_Init(void);
void Button_Init(void);


int main(void){
	//initializations
	initPDB();
	initGPIO();
	initFTM();
	uart_init();
	initInterrupts();
		
	for(;;){
		//To infinity and beyond
	}
}

void initPDB(void){
	//Enable clock for PDB module
	SIM_SCGC6 |= (SIM_SCGC6_PDB_MASK);

	// Set continuous mode
	PDB0_SC |= PDB_SC_CONT_MASK;
	
	//prescaler of 128, 
	PDB0_SC |= PDB_SC_PRESCALER(0x7);	//128 = 111 = 0x7
	
	//multiplication factor of 20,
	PDB0_SC |= PDB_SC_MULT(0x2);	//20 = 10 = 0x2
	
	// software triggering (at end of code) 
	PDB0_SC |= PDB_SC_TRGSEL(0xF);	//sw trigger = 1111 = 0xF
	
	//PDB enabled
	PDB0_SC |= PDB_SC_PDBEN_MASK;
	
	
	//Set the mod field to get a 1 second period.
	//There is a division by 2 to make the LED blinking period 1 second.
	//This translates to two mod counts in one second (one for on, one for off)
	PDB0_MOD = DEFAULT_SYSTEM_CLOCK / (128 * 20);
	//Configure the Interrupt Delay register.
	PDB0_IDLY = 10;
	
	//Enable the interrupt mask.
	PDB0_SC |= PDB_SC_PDBIE_MASK;
	
	//Enable LDOK to have PDB0_SC register changes loaded. 
	PDB0_SC |= PDB_SC_LDOK_MASK;
	
	PDB0_SC |= PDB_SC_SWTRIG_MASK;
	
	return;
}

void initFTM(void){
	//Enable clock for FTM module (use FTM0)
	SIM_SCGC6 |= (SIM_SCGC6_FTM0_MASK);
	
	//turn off FTM Mode to  write protection;
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	
	//divide the input clock down by 128,  
	FTM0_SC |= FTM_SC_PS(0x7);		//Division by 128 = 111 = 0x7
	
	//reset the counter to zero
	FTM0_CNT = FTM_CNT_COUNT(0);
	
	
	//Set the overflow rate
	//(Sysclock/128)- clock after prescaler
	//(Sysclock/128)/1000- slow down by a factor of 1000 to go from
	//Mhz to Khz, then 1/KHz = msec
	//Every 1msec, the FTM counter will set the overflow flag (TOF) and 
	FTM0->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;
	
	//Select the System Clock 
	FTM0_SC |= FTM_SC_CLKS(0x1);	//System clock = 01 = 0x1
	
	//Enable the interrupt mask. Timer overflow Interrupt enable
	FTM0_SC |= FTM_SC_TOIE_MASK;
	
	return;
}

void initGPIO(void){
	//initialize push buttons and LEDs
	LED_Init();
	Button_Init();
	
	//initialize clocks for each different port used.
	
	
	//Configure Port Control Register for Inputs with pull enable and pull up resistor

	// Configure mux for Outputs
	
	
	// Switch the GPIO pins to output mode (Red and Blue LEDs)
	
	
	// Turn off the LEDs

	// Set the push buttons as an input
	
	
	// interrupt configuration for SW3(Rising Edge) and SW2 (Either)
	PORTA_PCR4 |= PORT_PCR_IRQC(0x9);		// SW3 1001 = 0x9
	PORTC_PCR6 |= PORT_PCR_IRQC(0xB);		// SW2 1011 = 0xB
	
	
	return;
}

void initInterrupts(void){
	/*Can find these in MK64F12.h*/
	// Enable NVIC for portA,portC, PDB0,FTM0

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PDB0_IRQn);
	NVIC_EnableIRQ(FTM0_IRQn);
	
	return;
}


void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
  	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	 
	// Configure the Signal Multiplexer for GPIO
  	PORTB_PCR21 = PORT_PCR_MUX(1);
  	PORTB_PCR22 = PORT_PCR_MUX(1);
  	PORTE_PCR26 = PORT_PCR_MUX(1);
  
	// Switch the GPIO pins to output mode
		GPIOB_PDDR |= (1 << 21)|(1 << 22);
  	GPIOE_PDDR |= (1 << 26);
  

	// Turn off the LEDs
  	GPIOB_PSOR = (1 << 21)|(1 << 22);
  	GPIOE_PSOR = (1 << 26);  
}

void Button_Init(void){
	// Enable clock for Port C PTC6 button
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	// Enable clock for Port A PTC4 button
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// Configure the Mux for the button
	PORTC_PCR6 = PORT_PCR_MUX(1);
	PORTA_PCR4 = PORT_PCR_MUX(1);

	// Set the push button as an input
	GPIOC_PDDR &= ~(1 << 6);
	GPIOA_PDDR &= ~(1 << 4);	
}
