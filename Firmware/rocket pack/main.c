#include <msp430FR5739.h>
#include "UART.h"
#include "radio.h"
#include "sensorTag.h"
#include "types.h"


volatile unsigned char dbRXflag = 0;
volatile UARTERROR dbRXERROR = NOERROR;
volatile UARTERROR dbTXERROR = NOERROR;

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    //Initialize the SensorTag daughter board
    init_sensorTag();

   /*** Configure crystal ****************************/
    PJSEL0 |= BIT4 + BIT5;

    CSCTL0_H = 0xA5;						  //Enable Edit mode
    CSCTL1 |= DCOFSEL0 + DCOFSEL1 + DCORSEL;  // Set max. DCO setting
    CSCTL2 = 0x0003 | 0x0030;                 // set ACLK = XT1; MCLK = DCO; SMCLK = DCO
    CSCTL3 = 0x0000;				          // set all dividers to 1
    CSCTL4 = XT1DRIVE0 | XT1DRIVE1;			  // adjust crystal drive current

    do
    {
      CSCTL5 &= ~XT1OFFG;                    // Clear XT1 fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                  // Test oscillator fault flag





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

    uint16 tmpBYTE = NULL;


    db_send_byte(0xF0);

    while(1) {

    	if (dbRXflag)	//If a new char has been pushed into the buffer
    	{
    		db_error = db_buffer_pop(&tmpBYTE);
    		db_send_byte(tmpBYTE);
    	}

    	__bis_SR_register(LPM0_bits + GIE);   // LPM0 + Enable interrupt
    }


}






















