#include "sensorTag.h"



void init_sensorTag(void)
{
	//Set up pin to control tag reset pin
    P1DIR |= BIT3;
    sensorTag_reset(TRUE);
}


void sensorTag_reset(char mode)
{
	if ( mode ) P1OUT &= ~BIT3; else P1OUT |= BIT3;
}

