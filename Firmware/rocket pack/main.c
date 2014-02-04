#include <msp430FR5739.h>
#include "UART.h"

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	

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




    /*** Configure Daughter Board UART ( UCA1 ) *****************/






  //   __bis_SR_register(LPM0_bits + GIE);      // LPM3 + Enable interrupt
    _enable_interrupts();
    while(1) {}


}






















