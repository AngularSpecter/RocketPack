#include <msp430FR5739.h>
#include "types.h"

#define TXCMPLT		UCTXCPTIE       /* UART Transmit Complete Interrupt Enable */
#define TXIE		UCTXIE          /* UART Transmit Interrupt Enable */
#define RXIE		UCRXIE          /* UART Receive Interrupt Enable */

#define RADIOISR	UCA0IE
#define DBISR	    UCA1IE

typedef enum {
	NOERROR,		//everything is fine
	OVERFLOW_NA,	//buffer write overflow detected...buffer write not allowed
	OVERFLOW_WA,	//buffer write overflow detected...buffer write allowed
	BUFFEREMPTY	,	//tried to read an empty buffer.
	INVALIDOPTION	//Invalid option supplied
}UARTERROR;



void bridge_buffer(void);




extern volatile unsigned char dbRXflag;
extern volatile UARTERROR dbRXERROR;
extern volatile UARTERROR dbTXERROR;

typedef enum {READ, WRITE} dbBuffPos;

UARTERROR init_db_UART(void);
UARTERROR db_send_byte(unsigned char byte);
UARTERROR db_buffer_push(unsigned int value);
UARTERROR db_buffer_pop(unsigned int *dest);
UARTERROR db_spy_buffer(dbBuffPos whichBuffer, uint8 *value);


void init_radio_UART(void);
void radio_send_byte(unsigned char byte);
