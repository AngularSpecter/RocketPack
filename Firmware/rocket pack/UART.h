#ifndef UART_h
#define UART_h

#include <msp430FR5739.h>
#include "types.h"



#define TXCMPLT		UCTXCPTIE       /* UART Transmit Complete Interrupt Enable */
#define TXIE		UCTXIE          /* UART Transmit Interrupt Enable */
#define RXIE		UCRXIE          /* UART Receive Interrupt Enable */

#define RADIOISR	UCA0IE
#define DBISR	    UCA1IE

#define RADIO_RX_BUFFER_LEN  10
#define RADIO_TX_BUFFER_LEN  100

#define DB_RX_BUFFER_LEN  100
#define DB_TX_BUFFER_LEN  10

extern volatile unsigned char RADIORXflag;
extern volatile unsigned char dbRXflag;

extern UART_buffer radio_rx_buffer;
extern UART_buffer radio_tx_buffer;

extern UART_buffer db_rx_buffer;
extern UART_buffer db_tx_buffer;

//extern unsigned int radio_rx_buffer_array[RADIO_RX_BUFFER_LEN];
//extern unsigned int radio_tx_buffer_array[RADIO_TX_BUFFER_LEN];

//extern unsigned int db_rx_buffer_array[DB_RX_BUFFER_LEN];
//extern unsigned int db_tx_buffer_array[DB_TX_BUFFER_LEN];
/*****************************************************************/
void init_radio_UART(void);
void radio_send_byte(unsigned char byte);
void radio_send_string(char* string);
void radio_set_term(volatile unsigned char value);
void radio_tx_command(void);
UARTERROR radio_wait(uint16 timeout);

/**************************************************************/
UARTERROR init_db_UART(void);
UARTERROR db_send_byte(unsigned char byte);

#endif
