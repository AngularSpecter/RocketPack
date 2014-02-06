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



void radio_enter_CMD(void)
{
	 radio_command_mode = 1;
	 delay(32768);
	 radio_send_string("+++");
	 delay(32768);
}

void radio_leave_CMD(void)
{
	radio_send_string("ATCN\r");
}
