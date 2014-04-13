#include "timing.h"
#include "UART.h"
#include "types.h"

//TB1 and TB2 pins are completely used by alternate modules and should be used first for internal timing

/*** Delay Timer *****************************************************************/
void init_delay(void)
{
	//Set timer B0 to 32kHz aclk and set in 16 bit mode
	TB1CTL = TBCLR | TBSSEL_ACLK | CNTL_16;
}


void delay(uint16 ticks)
{
	init_delay();	           //setup and clear the timer
	TB1CCTL0 = CCIE;           //enable interrupt
	TB1CCR0  = ticks;          //Set trigger level
	TB1CTL  |= MC__CONTINUOUS;         //start the timer in up mode

	__bis_SR_register(LPM0_bits + GIE);  //go to sleep

	delay_cancel();
}

void delay_cancel(void)
{
	TB1CTL   &= ~(MC__CONTINUOUS);		 //shut down the timer
	TB1CCTL0 &= ~CCIE;		             //Kill the CCR interrupt
}



//CCR interrupt
#pragma vector=TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void)
{
	LPM0_EXIT;
	delay_cancel();
}

//Timer overflow interrupt
#pragma vector=TIMER1_B1_VECTOR
__interrupt void TIMER1_B1_ISR(void)
{
	LPM0_EXIT;
	delay_cancel();
}




void hs_interval_start(uint16 ticks)
{
	TB2CTL = TBCLR | TBSSEL_ACLK | CNTL_16;
	TB2CCTL0 = CCIE;
	TB2CCR0  = ticks;
	TB2CTL  |= MC__UP;
}

void hs_interval_stop(void)
{
	TB2CCTL0 &= ~CCIE;
	TB2CTL   &= ~0x18;
}

//CCR interrupt
#pragma vector=TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
{
	db_send_byte(sensorMask & ~0x20);
}

//Timer overflow interrupt
#pragma vector=TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
{
	db_send_byte(0x0F);
}


/*** Heartbeat interval timer ******************************************************/

unsigned int heartbeat_cnt;
const unsigned int heartbeat_int = WDTINT_1s;


void start_heartbeat(void)
{
	WDTCTL = WDTPW | WDTTMSEL | WDTSSEL__ACLK | heartbeat_int;
	SFRIE1 |= WDTIE;
}

void stop_heartbeat(void)
{
	WDTCTL = WDTPW | WDTHOLD;
	SFRIE1 &= ~WDTIE;
}

void wdt_force_puc(void)
{
	WDTCTL = (WDTPW + 1);  //write the wrong pw to force a PUC
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_TIMER_ISR(void)
{
	heartbeat_cnt++;
	LPM0_EXIT;
}




