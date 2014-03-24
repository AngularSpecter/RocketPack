#include "UART.h"
#include "buffer.h"
#include "timing.h"


UART_buffer radio_rx_buffer;
UART_buffer radio_tx_buffer;

UART_buffer db_rx_buffer;
UART_buffer db_tx_buffer;

//unsigned int radio_rx_buffer_array[RADIO_RX_BUFFER_LEN];
//unsigned int radio_tx_buffer_array[RADIO_TX_BUFFER_LEN];

//unsigned int db_rx_buffer_array[DB_RX_BUFFER_LEN];
//unsigned int db_tx_buffer_array[DB_TX_BUFFER_LEN];

volatile unsigned int db_process_packets = FALSE;

/*** Radio UART (uartA0) *******/
volatile BUFFERERROR RADIORXERROR = BUF_NOERROR;
volatile BUFFERERROR RADIOTXERROR = BUF_NOERROR;


void init_radio_UART(void)
{
    P2SEL1 |= BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTLW0 = UCSWRST;				   // Put UART in reset (off) mode

    //UCA0CTLW0 |= 0x0040;				   // Use ACLK;  8n1 implied as default setting
    //UCA0BRW    = 0x0003;                   // 9600 baud
    //UCA0MCTLW |= 0x9200;                   // UCBRSx value = 0x92 (See UG)


   // UCOS16 = 1; UCBR0 = 130;  UCBRF0 = 3; UCBRS0 = 0x25
    UCA0CTLW0 |= 0x00C0;				   // Use SMCLK;  8n1 implied as default setting
    UCA0BRW    = 130;
    UCA0MCTLW = 1 | 0x0030 | 0x2500;


    UCA0CTL1 &= ~UCSWRST;                  // release from reset

    init_buffer(&radio_rx_buffer);
    init_buffer(&radio_tx_buffer);
}

/**************************************************************************************/
void radio_send_byte(unsigned char byte)
{
	 while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
	 UCA0TXBUF = byte;                       // TX -> RXed character
}

void radio_send_string(char* string)
{
    while(*string) radio_send_byte(*string++);		//Advance though string till end
}

void radio_set_term(volatile unsigned char value)
{
	radio_rx_buffer.term_char = value;
}

void radio_tx_command(void)
{
  unsigned int value;

  if( radio_tx_buffer.packets_rxd )
  {
	  unsigned int length = command_length( &radio_tx_buffer );

	  while( length-- )
	   {
		  buffer_pop(&value, &radio_tx_buffer);
		  radio_send_byte( (char) value );
	   }
  }
}

UARTERROR radio_wait(uint16 timeout)
{
	//if a packet isn't waiting
	if (!radio_rx_buffer.packets_rxd)
	{

		/*Start a timer to exit LPM0 after <timeout>
		 * A full packet rxd will also exit LPM0   */
		delay(timeout);

		delay_cancel();    //kill the timer if the UART woke us up
	}

	//if a packet still isn't there, then we timed out
	if (!radio_rx_buffer.packets_rxd) return TIMEOUT;

	return NOERROR;
}



#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,0x08))
  {
  case 0:							// Vector 0 - no interrupt
	  break;
  case 2:                           // Vector 2 - RXIFG
	    RADIORXERROR = buffer_push(UCA0RXBUF, &radio_rx_buffer);
	    if (radio_rx_buffer.packets_rxd) LPM0_EXIT;
	  break;
  case 4:    						// Vector 4 - TXIFG
	  break;
  default:
	  break;
  }
}



/**************************************************************************************/
/*** Daughter Board UART (uartA1) *******/

volatile BUFFERERROR dbRXERROR;
volatile BUFFERERROR dbTXERROR;

UARTERROR init_db_UART(void)
{
    P2SEL1 |=   BIT5 + BIT6;
    P2SEL0 &= ~(BIT5 + BIT6);

    // Configure UART 0
    UCA1CTLW0 = UCSWRST;				   // Put UART in reset (off) mode

    //UCA1CTLW0 |= 0x0040;				   // Use ACLK;  8n1 implied as default setting
    //UCA1BRW    = 0x0003;                   // 9600 baud
    //UCA1MCTLW |= 0x9200;                   // UCBRSx value = 0x92 (See UG)

    //UCOS16 = 1; UCBR0 = 130;  UCBRF0 = 3; UCBRS0 = 0x25
    UCA1CTLW0 |= 0x00C0;				   // Use SMCLK;  8n1 implied as default setting
    UCA1BRW    = 130;
    UCA1MCTLW = 1 | 0x0030 | 0x2500;

    init_buffer(&db_rx_buffer);
    init_buffer(&db_tx_buffer);

    UCA1CTL1 &= ~UCSWRST;                  // release from reset

    return NOERROR;
}



UARTERROR db_send_byte(unsigned char byte)
{
	while ( !(UCA1IFG & UCTXIFG) );        //If the UART isn't ready to TX a byte
	UCA1TXBUF = byte;                      // TX byte

	 return NOERROR;
}





#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,0x08))
  {
  case 0:							// Vector 0 - no interrupt
	  break;
  case 2:                           // Vector 2 - RXIFG
	    dbRXERROR = buffer_push(UCA1RXBUF, &db_rx_buffer);
	    if (db_rx_buffer.packets_rxd) LPM0_EXIT;
	  break;
  case 4:    						// Vector 4 - TXIFG
	  break;
  case 8:
	  break;
  default:
	  break;
  }
}
