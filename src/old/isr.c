/*
 * isr.c
 */

#include "isr.h"
#include "MK64F12.h"
#include <stdio.h>
#include "uart.h"

//variables global to the IRQ handlers which dictates if timer is enabled &  timer counter
int counter = 0;
int press = 0;
char str[255];

void PDB0_IRQHandler(void){ //For PDB timer
	//clear the interrupt in PDB0_SC
	PDB0_SC &= ~(PDB_SC_PDBIF_MASK);
	
	GPIOB_PTOR |= (1<<21);
	
	//toggle the output state for LED1 (BLUE)
//	if((GPIOB_PDIR & (1<<6))== 0 ){	
//		GPIOB_PCOR = (1UL << 21);	//turn on
//	}
//	else{
//		GPIOB_PSOR = (1UL << 21);	//turn off
//	}
	
	return;
}
	

void FTM0_IRQHandler(void){ //For FTM timer
	
	//clear the interrupt in register FTM0_SC
	(void)FTM0_SC;
	FTM0_SC &= ~(FTM_SC_TOF_MASK);
	
	if(press){
		counter++;
	}
	
	return;
}
	
void PORTA_IRQHandler(void){ //For switch 3
	//clear the interrupt
	PORTA->ISFR = (1<<4);
	
	if((PDB0_SC & (1<<7))){
		//disable the timer
		PDB0_SC &= ~(PDB_SC_PDBEN_MASK);
	}
	else{
		//Enable the timer
		PDB0_SC |= PDB_SC_PDBEN_MASK;
		//Start with trigger
		PDB0_SC |= PDB_SC_SWTRIG_MASK;
	}
	return;
}
	
void PORTC_IRQHandler(void){ //For switch 2
	//clear the interrupt
	PORTC->ISFR = (1<<6);
	
	// if switch is being pressed
	if((GPIOC_PDIR & (1<<6)) == 0){
			press = 1;			//set button press
			FTM0_CNT = 0;								//reset timer
			counter = 0;		//Reset counter
			GPIOB_PCOR = (1UL << 22);	//turn on red LED
	}
	else{
		press = 0;		//Reset for timer2
		
		GPIOB_PSOR = (1UL << 22);		//turn off red LED
		
		sprintf(str,"Button held for %d milliseconds!\n\r", counter);
		uart_put(str);
		
	}
	return;
}
