#include "UART.h"

void bridge_buffer(void)
{
	while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
		 UCA0TXBUF = UCA1RXBUF;
}

/*** Radio UART (uartA0) *******/
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

void radio_send_byte(unsigned char byte)
{
	 while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
	 UCA0TXBUF = byte;                       // TX -> RXed character
}

void radio_send_string(unsigned char* string)
{
    while(*string) radio_send_byte(*string++);		//Advance though string till end
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

/**************************************************************************************/
/*** Daughter Board UART (uartA1) *******/
#define  DB_BUFFER_LEN  5


const uint8 DBALLOWOVERFLOW  = FALSE;

volatile unsigned int db_RX_byte[DB_BUFFER_LEN];
volatile unsigned int db_read_idx  = 0;
volatile unsigned int db_write_idx = 0;



UARTERROR init_db_UART(void)
{
    P2SEL1 |=   BIT5 + BIT6;
    P2SEL0 &= ~(BIT5 + BIT6);

    // Configure UART 0
    UCA1CTLW0 = UCSWRST;				   // Put UART in reset (off) mode

    UCA1CTLW0 |= 0x0040;				   // Use ACLK;  8n1 implied as default setting
    UCA1BRW    = 0x0003;                   // 9600 baud
    UCA1MCTLW |= 0x9200;                   // UCBRSx value = 0x92 (See UG)

    UCA1CTL1 &= ~UCSWRST;                  // release from reset

    return NOERROR;
}

UARTERROR db_send_byte(unsigned char byte)
{
	while ( !(UCA1IFG & UCTXIFG) );        //If the UART isn't ready to TX a byte
	UCA1TXBUF = byte;                      // TX byte

	 return NOERROR;
}


/*** Generic functions to access buffer ***/

//push a new byte into the buffer
UARTERROR db_buffer_push(unsigned int value)
{
	//Check to see if we are going to overflow the buffer and throw an error if overflow isn't allowed
	if ( !DBALLOWOVERFLOW && (db_bytes_to_proc == DB_BUFFER_LEN) )return OVERFLOW_NA;

	db_RX_byte[db_write_idx++] = value;
	if (db_write_idx >= DB_BUFFER_LEN) db_write_idx = 0;

	//Check again for overflow and return a warning (without advancing the byte to proc count
	if (db_bytes_to_proc == DB_BUFFER_LEN) return OVERFLOW_WA;

	//If we make it here, then we haven't overflowed the buffer, so increase the number of bytes to proc
	db_bytes_to_proc++;
	return NOERROR;
}

//pop the first unread byte off the buffer
UARTERROR db_buffer_pop(volatile unsigned int *dest)
{
	*dest = NULL;
	if( db_bytes_to_proc )
	{
	  *dest = db_RX_byte[db_read_idx++];
	  if (db_read_idx >= DB_BUFFER_LEN) db_read_idx = 0;
	  db_bytes_to_proc--;
	  return NOERROR;
	}
	return BUFFEREMPTY;
}

//Spy on the (last)next byte received(to send) without advancing the pointer
UARTERROR db_spy_buffer(dbBuffPos whichBuffer, uint8 *value)
{
	*value = NULL;
	if ( !db_bytes_to_proc ) return BUFFEREMPTY;

	switch (whichBuffer)
	{
	case READ :
		*value = db_RX_byte[db_read_idx];
		return NOERROR;
	case WRITE :
		*value = db_RX_byte[db_write_idx-1];
		return NOERROR;
	default:
		*value = NULL;
		return INVALIDOPTION;
	}
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,0x08))
  {
  case 0:							// Vector 0 - no interrupt
	  break;
  case 2:                           // Vector 2 - RXIFG
	  dbRXERROR = db_buffer_push(UCA1RXBUF);
	  LPM0_EXIT;
	  break;
  case 4:    						// Vector 4 - TXIFG
	  break;
  case 8:
	  break;
  default:
	  break;
  }
}
