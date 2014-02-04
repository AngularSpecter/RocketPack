#include "UART.h"

#define TXCMPLT		UCTXCPTIE       /* UART Transmit Complete Interrupt Enable */
#define TXIE		UCTXIE          /* UART Transmit Interrupt Enable */
#define RXIE		UCRXIE          /* UART Receive Interrupt Enable */

/*** Radio UART (uartA0) *******/
#define RADIOISR	UCA0IE


void init_radio_UART(void)
{
    P2SEL1 |= BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTLW0 = UCSWRST;				   // Put UART in reset (off) mode

    UCA0CTLW0 |= 0x0040;				   // Use ACLK;  8n1 implied as default setting
    UCA0BRW    = 0x0003;                   // 9600 baud
    UCA0MCTLW |= 0x9200;                   // UCBRSx value = 0x92 (See UG)

    UCA0CTL1 &= ~UCSWRST;                  // release from reset
}



#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,0x08))
  {
  case 0:							// Vector 0 - no interrupt
	  break;
  case 2:                           // Vector 2 - RXIFG
	  break;
  case 4:    						// Vector 4 - TXIFG
	  break;
  default:
	  break;
  }
}


/*** Daughter Board UART (uartA1) *******/
#define DBISR	UCA1IE

void init_db_UART(void)
{
    P2SEL1 |= BIT5 + BIT6;
    P2SEL0 &= ~(BIT5 + BIT6);

    // Configure UART 0
    UCA1CTLW0 = UCSWRST;				   // Put UART in reset (off) mode

    UCA1CTLW0 |= 0x0040;				   // Use ACLK;  8n1 implied as default setting
    UCA1BRW    = 0x0003;                   // 9600 baud
    UCA1MCTLW |= 0x9200;                   // UCBRSx value = 0x92 (See UG)

    UCA1CTL1 &= ~UCSWRST;                  // release from reset
}



#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA0IV,0x08))
  {
  case 0:							// Vector 0 - no interrupt
	  break;
  case 2:                           // Vector 2 - RXIFG
	  break;
  case 4:    						// Vector 4 - TXIFG
	  break;
  default:
	  break;
  }
}
