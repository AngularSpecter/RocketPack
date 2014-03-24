#ifndef BUFFER_H_
#define BUFFER_H_

#include <msp430FR5739.h>
#include "types.h"



void init_buffer(UART_buffer * buffer);
BUFFERERROR buffer_push(unsigned int value, UART_buffer * buffer);
BUFFERERROR buffer_pop(volatile unsigned int *dest, UART_buffer * buffer);
BUFFERERROR spy_buffer(dbBuffPos whichBuffer, uint8 *value, UART_buffer * buffer);
BUFFERERROR flush_buffer(UART_buffer * buffer);
BUFFERERROR pop_command(unsigned int * dest, UART_buffer * buffer);
BUFFERERROR command_copy(UART_buffer * src_buffer, UART_buffer * dest_buffer);
unsigned int command_length(UART_buffer * buffer);

#endif /* BUFFER_H_ */
