#include "msp430g2553.h"
#include "uart.h"
#include "sensors.h"

unsigned int trigger = 0;


int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; 			//Stop WDT
    BCSCTL1 = CALBC1_8MHZ; 				//Set DCO to 8Mhz
    DCOCTL = CALDCO_8MHZ; 				//Set DCO to 8Mhz

    uart_init();

    __enable_interrupt();				//Interrupts Enabled


    P1REN |=  BIT3;                 // Enable internal pull-up/down resistors
    P1OUT |=  BIT3;                 //Select pull-up mode for P1.3
    P1IE  |=  BIT3;                  // P1.3 interrupt enabled
    P1IES |=  BIT3;                 // P1.3 Hi/lo edge
    P1IFG &= ~BIT3;                // P1.3 IFG cleared


    unsigned int gyro_buffer[3];

    while(1)
    {

    	if (trigger)
    	{
    		trigger = 0;

    		start_gyro();
    		//read_gyro(&gyro_buffer[0]);
    	}

    }
}


/*
 uint8 command = 0x00;          //put to sleep
  HalSensorWriteReg(ACC_CTRL_REG1, &command, 1);

    //configure CTRL2 - 6.25Hz
  command = ACC_OWUFB | ACC_OWUFC;
  HalSensorWriteReg(ACC_CTRL_REG2, &command, 1);

  // Pulse the interrupt, active high and enable
  command = ACC_IEL | ACC_IEA | ACC_IEN;
  HalSensorWriteReg(ACC_INT_CTRL_REG1, &command, 1);

  //Enable interrupt for all axes
  command = ACC_ALLWU;
  HalSensorWriteReg(ACC_INT_CTRL_REG2, &command, 1);

 */

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
   trigger = 1;
   P1OUT ^= BIT6;                        // Toggle P1.6
   P1IFG &=~BIT3;                        // P1.3 IFG cleared
}


