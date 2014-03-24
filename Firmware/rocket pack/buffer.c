#include "buffer.h"

const uint8 BUFFER_ALLOWOVERFLOW  = FALSE;

void init_buffer(UART_buffer * buffer)
{
	buffer->buffer_len = 50;
	buffer->read_idx  = 0;
	buffer->write_idx = 0;
	buffer->bytes_to_proc = 0;
	buffer->packets_rxd = FALSE;
    buffer->last_command_idx = 0;
    buffer->term_char = 0x00;
}

//push a new byte into the buffer
BUFFERERROR buffer_push(unsigned int value, UART_buffer * buffer)
{
	//Check to see if we are going to overflow the buffer and throw an error if overflow isn't allowed
	if ( !BUFFER_ALLOWOVERFLOW && (buffer->bytes_to_proc == buffer->buffer_len) )
	{
	  //roll back the buffer to the beginning of the new command

	  buffer->bytes_to_proc -= ( buffer->write_idx > buffer->last_command_idx ) ?
			                     buffer->write_idx - buffer->last_command_idx :
			                     buffer->write_idx + ( buffer->buffer_len - buffer->last_command_idx );

	  buffer->write_idx = buffer->last_command_idx;					//reset write index
	  return BUF_OVERFLOW_NA;
	}

	buffer->buffer[buffer->write_idx++] = value;
	if (buffer->write_idx >= buffer->buffer_len) buffer->write_idx = 0;

	//Check again for overflow and return a warning (without advancing the byte to proc count
	if (buffer->bytes_to_proc == buffer->buffer_len) return BUF_OVERFLOW_WA;

	//If we make it here, then we haven't overflowed the buffer, so increase the number of bytes to proc
	buffer->bytes_to_proc++;

	//if this is a termination character then flag it and advance the counter
	if (value == buffer->term_char)
		{
		buffer->packets_rxd++;
		buffer->last_command_idx = buffer->write_idx;	//store the reset position
		}

	return BUF_NOERROR;
}

//pop the first unread byte off the buffer
BUFFERERROR buffer_pop(volatile unsigned int *dest, UART_buffer * buffer)
{
	*dest = NULL;

	//if there are unread bytes in the buffer
	if( buffer->bytes_to_proc )
	{
	  //read the byte
	  *dest = buffer->buffer[buffer->read_idx++];

	  //overflow the read position
	  if (buffer->read_idx >= buffer->buffer_len) buffer->read_idx = 0;

	  //decrement the byte count
	  buffer->bytes_to_proc--;

	  //if this is the end of the command, decrement the buffer
	  if (*dest == buffer->term_char)  buffer->packets_rxd--;
	  return BUF_NOERROR;
	}
	return BUF_BUFFEREMPTY;
}

BUFFERERROR pop_command(unsigned int * dest, UART_buffer * buffer)
{
	BUFFERERROR error = BUF_NOERROR;
    unsigned int value = buffer->term_char + 1;   //make sure value does not equal the term char

   if ( buffer->packets_rxd == 0) return BUF_BUFFEREMPTY;

   while( value != buffer->term_char )
   {
	  error =  buffer_pop(&value, buffer);
	  *dest = (value == buffer->term_char) ? 0x00 : value;
	  dest++;
	  if ( error != BUF_NOERROR ) return error;
   }

   return error;
}

BUFFERERROR command_copy(UART_buffer * src_buffer, UART_buffer * dest_buffer)
{
	flush_buffer(dest_buffer);

	BUFFERERROR error = BUF_NOERROR;
	unsigned int value = src_buffer->term_char + 1;

	   while( value != src_buffer->term_char )
	   {
		  error =  buffer_pop(&value, src_buffer);
		  error =  buffer_push(value, dest_buffer);
		  if ( error != BUF_NOERROR ) return error;
	   }

	   return error;
}



//Spy on the (last)next byte received(to send) without advancing the pointer
BUFFERERROR spy_buffer(dbBuffPos whichBuffer, uint8 *value, UART_buffer * buffer)
{
	*value = NULL;
	if ( !buffer->bytes_to_proc ) return BUF_BUFFEREMPTY;

	switch (whichBuffer)
	{
	case READ :
		*value = buffer->buffer[buffer->read_idx];
		return BUF_NOERROR;
	case WRITE :
		*value = buffer->buffer[buffer->write_idx-1];
		return BUF_NOERROR;
	default:
		*value = NULL;
		return BUF_INVALIDOPTION;
	}
}



unsigned int command_length(UART_buffer * buffer)
{
	unsigned int count  = 0;
	unsigned int idx    = buffer->read_idx;

	if ( buffer->packets_rxd == 0) return 0;

	while( buffer->buffer[idx++] != buffer->term_char )
		{
		if ( idx >= buffer->buffer_len ) idx = 0;
		count++;
		}
	return ++count;

}

BUFFERERROR flush_buffer(UART_buffer * buffer)
{

	buffer->read_idx  = 0;
    buffer->write_idx = 0;

	//reset the number of bytes to process
	buffer->bytes_to_proc    = 0;
	buffer->last_command_idx = 0;
	buffer->packets_rxd      = 0;
	return BUF_NOERROR;
}

