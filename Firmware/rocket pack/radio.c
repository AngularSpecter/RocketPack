#include "radio.h"
#include "UART.h"

void config_radio(void)
{
	init_radio_UART();
	P3DIR &= ~(STATUS + RESET); //make sure the status pin is an input
	P3DIR |= SLEEP + CONFIG;    //sleep, config pins are outputs
}


void sleep_radio(char mode)
{
	if (mode) P3OUT |= SLEEP; else P3OUT &= ~SLEEP;
}
