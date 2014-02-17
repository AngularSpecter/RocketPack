#include "timing.h"
#include "types.h"


//TB1 and TB2 pins are completely used by alternate modules and should be used first for internal timing


/*** Delay Timer *****************************************************************/
inline void init_delay(void)
{
	//Set timer B0 to 32kHz aclk (crystal) and set in 16 bit mode
	TB1CTL = TBCLR | TBSSEL_ACLK | CNTL_16;
}


void delay(uint16 ticks)
{
	init_delay();	           //setup and clear the timer
	TB1CCTL0 = CCIE;           //enable interrupt
	TB1CCR0  = ticks;          //Set trigger level
	TB1CTL  |= MC__CONTINUOUS;         //start the timer in up mode

	__bis_SR_register(LPM0_bits + GIE);  //go to sleep

	TB1CTL   &= ~(MC__CONTINUOUS);		 //shut down the timer
	TB1CCTL0 &= ~CCIE;		     //Kill the CCR interrupt
}

//CCR interrupt
#pragma vector=TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void)
{
	LPM0_EXIT;
}

//Timer overflow interrupt
#pragma vector=TIMER1_B1_VECTOR
__interrupt void TIMER1_B1_ISR(void)
{
	LPM0_EXIT;
}

