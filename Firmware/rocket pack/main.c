#include <msp430FR5739.h>
#include "types.h"
#include "UART.h"
#include "buffer.h"
#include "radio.h"
#include "sensorTag.h"
#include "timing.h"

volatile unsigned char dbRXflag      = 0;
         uint16        radioAddress  = 0xff;
   const unsigned int  magTrigger    = 0;
         unsigned int  sensorMask    = 0;
     static const int  mag_poll_cnt  = 1;

     unsigned char     heartbeat_TO  = 0;


void main(void) {

	uint16 poll_rate = 1100;

    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    mode proc_mode = SLEEPWAIT;

    //Initialize the SensorTag daughter board and hold in reset
    init_sensorTag();

   /*** Configure crystal ****************************/
    PJSEL0 |= BIT4 + BIT5;

    CSCTL0_H = 0xA5;						  //Enable Edit mode
    CSCTL1 = DCOFSEL0 + DCORSEL; // DCOFSEL1 ;  // Set max. DCO setting
    CSCTL2 = 0x0003 | 0x0030;                 // set ACLK = XT1; MCLK = DCO; SMCLK = DCO
    CSCTL3 = 0x0000;				          // set all dividers to 1;
    CSCTL4 = XT1DRIVE0 | XT1DRIVE1;			  // adjust crystal drive current

  /*  do
    {
      CSCTL5 &= ~XT1OFFG;                    // Clear XT1 fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                  // Test oscillator fault flag
   */

    /*** Configure Radio UART ( UCA0 ) *****************/
    config_radio();					        //configure UART
    sleep_radio(TRUE);
    RADIOISR |=  RXIE;					    //enable rx interrupt


    /*** Configure Daughter Board UART ( UCA1 ) *****************/
    init_db_UART();    		    //configure UART
    DBISR |=  RXIE;             //enable rx interrupt

    _enable_interrupts();

    uint8 success = 0;

    /* Read the current address from the radio */

	//while (!success)    //loop until we can communicate with the radio
	//{
		//Read and cache the radio address
		success = radio_enter_CMD();
		if (success)
		{
		  success = radio_read_value(ADDRESS_DST, &radioAddress);
		  if (success) success = radio_leave_CMD();
		}

		//if(!success) delay(32768);
	//}

	 radio_set_term(0x00);
     start_heartbeat();

	 flush_buffer(&radio_rx_buffer);      //flush the command buffer





	 /** Operating modes  *********************************
	  *
	  *  SLEEPWAIT : The board starts in low power sleep wait.  Board is woken up with
	  *              a node select packet from the base station.  Board polls the magnetometer
	  *              and tx's a host discovery packet when triggered.
	  *
	  *  PRESTAGE  : Transition state to active mode.  The board waits for the base to request
	  *              calibration data and command the startup of the sensors
	  *
	  *  SHUTDOWN  : Transition state to sleep mode.  The board is shutdown.
	  *
	  *  ACTIVEMODE: The board listens to the sensor board and transmits everything over
	  *              radio
	  */



    // Main processing loop
    while(1) {

    	/************************************************************************/
    	//if there are incoming commands to process
    	if ( radio_rx_buffer.packets_rxd > 0)
    	{

    		unsigned int packet[4];
    		pop_command(&packet[0],&radio_rx_buffer);     //read the command off the stack

    		//Node select packets are handled the same regardless of operating mode
    		if ( packet[0] == NSEL )
    		{
				 if (packet[1] == radioAddress)  //if the packet is addressed to me
				 {
					 //ACK with my address
					 radio_send_byte(RADIOACK);
					 radio_send_byte(radioAddress);
					 radio_send_byte(0x00);
					 proc_mode    = PRESTAGE;
				 }else{
					 proc_mode    = SHUTDOWN;
				 }
    		}


    		//Host discovery packets are also handled the same regardless
    		if ( packet[0] == HOSTDISC )
    		{
    			//To make sure other radios don't overlap, we need to timeslot these replies
    			//This is done by waiting 128 times (4 ms ) the radio address
    			delay( 128 * radioAddress );
    			radio_send_byte(HOSTDISC);
    			radio_send_byte(radioAddress);
    			radio_send_byte(0x00);
    		}


    		//Only respond to cal requests in active mode
    		if (packet[0] == CAL && proc_mode == ACTIVEMODE )
    		{
    			//request cal data from the sensor board
    			db_send_byte(CALDATA);
    		}

    		//A data packet updates the sensor data mask (including autopoll mode)
    		if (packet[0] == DATA &&  proc_mode == ACTIVEMODE )
    		{
    			sensorMask = packet[1];
    			heartbeat_TO = packet[2];
    			heartbeat_cnt = 0;
    			//db_send_byte( sensorMask );

    			if ( packet[1] & 0x20 )
    			{
    				hs_interval_start(poll_rate);
    			}else{
    				hs_interval_stop();
    			}

    		}

    		//An ACK packet in active mode is the heartbeat ping to keep the board alive
    		if ( packet[0] == RADIOACK && proc_mode == ACTIVEMODE)
    		{
    			heartbeat_cnt = 0;
    		}

    	}

    	/************************************************************************/
    	if ( db_tx_buffer.packets_rxd > 0)
		{
    		switch ( proc_mode )
			{
			case SLEEPWAIT :
				break;
			case ACTIVEMODE :
				break;
			}
		}


    	/************************************************************************/
    	if ( db_rx_buffer.packets_rxd > 0)
		{
    		switch ( proc_mode )
			{

    		//in sleepwait, incoming data is used to determine if we should announce wakeup
			case SLEEPWAIT :
				sensorTag_reset( TRUE );	//sleep the sensor board again

				//todo: process data and send host disc
				break;

			//in active mode, copy the full command to the TX buffer
			case ACTIVEMODE :
				command_copy( &db_rx_buffer, &radio_tx_buffer );
				break;
			}

		}

    	/************************************************************************/
    	if ( radio_tx_buffer.packets_rxd > 0)
		{
    		unsigned int length = command_length(&radio_tx_buffer);
    		unsigned int value;

    		while ( length-- )
			{
			buffer_pop( &value, &radio_tx_buffer );
			radio_send_byte( (char) value);
			}
		}


    	/*** Mode specific functions ********************************************/

    	switch ( proc_mode )
    	{
    	case ACTIVEMODE :
    		//Heartbeat code

    		if ( heartbeat_TO > 0 && heartbeat_cnt > heartbeat_TO )
    		{
    			proc_mode = SHUTDOWN;	//sleep the sensor
        		heartbeat_cnt = 0;
    		}
    		break;

    	// Wake up sensor board and prepare it for action
    	case PRESTAGE   :
    		sensorTag_reset(FALSE);
    		proc_mode = ACTIVEMODE;
    		break;

    	// Shutdown the sensor board
    	case SHUTDOWN   :
    		sensorTag_reset(TRUE);
    		proc_mode = SLEEPWAIT;
    		break;

    	case SLEEPWAIT  :
    		//Magnetometer poll
    		if ( heartbeat_cnt >= mag_poll_cnt )
    		{
    			//wake up the sensor board and request magnetometer data
    			sensorTag_reset( FALSE );
    			db_send_byte( MAGNET );
    			heartbeat_cnt = 0;
    		}
    		break;

    	}




    	/***********************************************************************/
    	//go to sleep (we will wake when something worth messing with happens)
    	if ( !(radio_tx_buffer.packets_rxd || db_tx_buffer.packets_rxd ||
    		   radio_rx_buffer.packets_rxd || db_rx_buffer.packets_rxd ) )
    	{
    		__bis_SR_register(LPM0_bits + GIE);
    	}


    }


}
