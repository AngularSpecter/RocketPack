<<<<<<< HEAD
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
=======
#include <msp430FR5739.h>
#include "UART.h"
#include "radio.h"
#include "sensorTag.h"
#include "types.h"
#include "timing.h"

volatile unsigned char dbRXflag = 0;
volatile UARTERROR dbRXERROR = NOERROR;
volatile UARTERROR dbTXERROR = NOERROR;
volatile unsigned int db_bytes_to_proc;

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    //Initialize the SensorTag daughter board
    init_sensorTag();

   /*** Configure crystal ****************************/
    PJSEL0 |= BIT4 + BIT5;

    CSCTL0_H = 0xA5;						  //Enable Edit mode
    CSCTL1 = DCOFSEL0 + DCORSEL; // DCOFSEL1 ;  // Set max. DCO setting
    CSCTL2 = 0x0003 | 0x0030;                 // set ACLK = XT1; MCLK = DCO; SMCLK = DCO
    CSCTL3 = 0x0000;				          // set all dividers to 1;
    CSCTL4 = XT1DRIVE0 | XT1DRIVE1;			  // adjust crystal drive current

    do
    {
      CSCTL5 &= ~XT1OFFG;                    // Clear XT1 fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                  // Test oscillator fault flag

   // CSCTL0_H = 0x00;




    /*** Configure Radio UART ( UCA0 ) *****************/
    config_radio();					        //configure UART
    sleep_radio(FALSE);
    RADIOISR |=  RXIE;					//enable rx interrupt


    /*** Configure Daughter Board UART ( UCA1 ) *****************/
    UARTERROR db_error = NOERROR;
    db_error = init_db_UART();    		//configure UART
    DBISR |=  RXIE;                     //enable rx interrupt


    _enable_interrupts();


    //Allow the sensorTag to boot
    sensorTag_reset(FALSE);
    //db_send_byte(0xF0);

    //delay(32000);
    //db_send_byte(0x2F);

    while(1) {

       // db_send_byte(count++);
       // delay(32768);

    /*	if (db_bytes_to_proc)	//If a new char has been pushed into the buffer
    	{
    		db_error = db_buffer_pop(&tmpBYTE);
    		db_send_byte(tmpBYTE);
    	}else{

    	    __bis_SR_register(LPM0_bits + GIE);   // LPM0 + Enable interrupt
    	}  */
    }


}




















>>>>>>> origin/streamlined


