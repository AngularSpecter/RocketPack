#include "msp430g2553.h"
#include "uart.h"

#define LED BIT0
#define RXD BIT1
#define TXD BIT2


volatile unsigned int tx_flag;			//Mailbox Flag for the tx_char.
volatile unsigned char tx_char;			//This char is the most current char to go into the UART

volatile unsigned int rx_flag;			//Mailbox Flag for the rx_char.
volatile unsigned char rx_char;			//This char is the most current char to come out of the UART

volatile unsigned char buffer[8];
volatile unsigned char buffer_pos_w = 0;
volatile unsigned char buffer_pos_r = 0;

void uart_init(void)
{
	P1SEL = RXD + TXD;					//Setup the I/O
	P1SEL2 = RXD + TXD;

    P1DIR |= LED; 						//P1.0 red LED. Toggle when char received.
    P1OUT |= LED; 						//LED off

	UCA0CTL1 |= UCSSEL_2; 				//SMCLK
										//8,000,000Hz, 9600Baud, UCBRx=52, UCBRSx=0, UCBRFx=1
	UCA0BR0 = 52;                  		//8MHz, OSC16, 9600
	UCA0BR1 = 0;                   	 	//((8MHz/9600)/16) = 52.08333
	UCA0MCTL = 0x10|UCOS16; 			//UCBRFx=1,UCBRSx=0, UCOS16=1
	UCA0CTL1 &= ~UCSWRST; 				//USCI state machine
	IE2 |= UCA0RXIE; 					//Enable USCI_A0 RX interrupt

	rx_flag = 0;						//Set rx_flag to 0
	tx_flag = 0;						//Set tx_flag to 0

	return;
}


unsigned char uart_status(void)
{
	return rx_flag;
}



void uart_putc(unsigned char c)
{
	tx_char = c;						//Put the char into the tx_char
	IE2 |= UCA0TXIE; 					//Enable USCI_A0 TX interrupt
	while(tx_flag == 1);				//Have to wait for the TX buffer
	tx_flag = 1;						//Reset the tx_flag
	return;
}

/*uart_puts
* Sends a string to the UART. Will wait if the UART is busy
* INPUT: Pointer to String to send
* RETURN: None
*/
void uart_puts(char *str)				//Sends a String to the UART.
{
     while(*str) uart_putc(*str++);		//Advance though string till end
     return;
}


/*Fetch the value last written to the buffer */
unsigned char last_RXd(void)
{
   unsigned char idx = (buffer_pos_w > 0) ? buffer_pos_w -1 : 7;
   return buffer[idx];

}

unsigned char pop(void)
{
	unsigned char value = last_RXd();
	buffer_pos_w = (buffer_pos_w > 0) ? buffer_pos_w -1 : 7;
    return value;
}

unsigned char shift(void)
{
	unsigned char value = buffer[buffer_pos_r];
	buffer_pos_r = (buffer_pos_r == 8) ? 0 : buffer_pos_r + 1;
	return value;
}


void uart_read(unsigned char * rbuffer)
{
	rbuffer[0] = 0;
	unsigned char id = 0;
	while(!rx_flag);

	while(rx_flag)
		{
		rx_flag--;
		rbuffer[id++] = pop();
		}

}


/*** Interrupts ******************************************************************/
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	buffer[buffer_pos_w++] = UCA0RXBUF;
	rx_flag++;
	if (buffer_pos_w == 8) buffer_pos_w = 0;
	P1OUT ^= LED;
}

#pragma vector = USCIAB0TX_VECTOR		//UART TX USCI Interrupt
__interrupt void USCI0TX_ISR(void)
{
	UCA0TXBUF = tx_char;				//Copy char to the TX Buffer
	IE2 &= ~UCA0TXIE; 					//Turn off the interrupt to save CPU
	tx_flag = 0;
}
