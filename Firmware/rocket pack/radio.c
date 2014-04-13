#include "radio.h"
#include "timing.h"
#include "UART.h"
#include "buffer.h"


void config_radio(void)
{
	init_radio_UART();
	P3DIR &= ~(STATUS + RESET + SLEEP + CONFIG); //make sure the status pin is an input
	P3DIR |= SLEEP;    //sleep, config pins are outputs
}


void sleep_radio(char mode)
{
	if (mode) P3OUT |= SLEEP; else P3OUT &= ~SLEEP;
}



char radio_enter_CMD(void)
{
	unsigned int output[6];
	UARTERROR error;

	//db_process_packets = FALSE;

	 //radio_command_mode = 1;
	 delay(32768);				//guard time
	 flush_buffer(&radio_rx_buffer);
	 radio_set_term('\r');
	 radio_send_string("+++");
	 delay(32768);				//guard time

	 error = radio_wait(32768);		//wait for the radio to reply

	 if (error == TIMEOUT) return 0;

	 pop_command(&output[0], &radio_rx_buffer);

	 return radio_proc_ack(&output[0]);

}

char radio_proc_ack(unsigned int * buffer)
{
	 if ( buffer[0] == 'O' && buffer[1] == 'K')
	 {
		return 1;
	 }else{
		 return 0;
	 }
}


char radio_set_value(char * reg, char * value)
{
	unsigned int output[6];
	UARTERROR error;

	radio_send_string("AT");
	radio_send_string(reg);
	radio_send_string(value);
	radio_send_string("\r");

	error = radio_wait(32768);		//wait for the radio to reply
    if (error == TIMEOUT) return 0;

	pop_command(&output[0], &radio_rx_buffer);
	return radio_proc_ack(&output[0]);
}


char radio_read_value(char * reg, uint16 * value)
{
	UARTERROR error;
	unsigned int reply[4];
	unsigned int * idx;
	idx = &reply[0];

	//Write command with AT prefix
	radio_send_string("AT");
	radio_send_string(reg);
    radio_send_string("\r");

    //Wait for the radio to reply with a 1s timeout
    error = radio_wait(32768);
    if (error == TIMEOUT) return 0;

    //pull the reply off the buffer
    pop_command(&reply[0], &radio_rx_buffer);

    /*convert the hexstring to a number
     * each char in the string is a 4-bit nibble, so convert it from string to number,
     * and left shift 4 bits if there is another nibble in the stack
     */
    char shift = 0;
    *value = 0;

    while(*(idx++))
    {
    	*value << 4*(shift++);
    	*value += (uint16)str2dec(idx-1);
    }

    return 1;
}


char radio_write_changes(void)
{
	UARTERROR error;
	unsigned int output[6];

	radio_send_string("ATWR\r");

    error = radio_wait(32768);		//wait for the radio to reply
    if (error == TIMEOUT) return 0;

    pop_command(&output[0], &radio_rx_buffer);
    return radio_proc_ack(&output[0]);
}

char radio_leave_CMD(void)
{
	radio_send_string("ATCN\r");
	radio_wait(32768);

	unsigned int output[6];
    pop_command(&output[0], &radio_rx_buffer);
   // db_process_packets = FALSE;
    radio_set_term(0x00);
    return radio_proc_ack(&output[0]);
}

/*****************************************************************************************************/
unsigned int str2dec(unsigned int * string)
{
	if (*string < 'A')
	{
		return *string - 48;
	}else if(*string >= 'a'){
		return *string - 87;
	}else{
		return *string - 55;
	}

}


