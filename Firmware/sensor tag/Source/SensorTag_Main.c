/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
#include <ioCC2541.h>
#include <hal_defs.h>
#include <hal_board_cfg.h>
#include "sleepTimer.h"
#include <hal_i2c.h>
#include <hal_sensor.h>
   
#include "uart.h"
#include "sensors.h"

/*************************************************************************************************
          Globals
**************************************************************************************************/
   volatile uint32 timeout = 0x000F;
  
  //Mode bit mask
  // 0 -> 3  :  Sensors
  //   4     :  Baro calibration data
  //   5     :  Auto poll (1 = auto poll, 0 = ping to poll)

 

   //start up with all sensors running in ping-to-poll mode
   uint8 active_sensors = BV(0) | BV(1) | BV(2) | BV(3) | BV(5);
   
volatile uint8 RXin = 1;
   
/*************************************************************************************************
          Main
**************************************************************************************************/
void main(void)
{
 
  HAL_BOARD_INIT();
  
  UART_init();
  U0CSR &= ~0x04;
  ENABLE_RX();
  
  /*setup sensors*/
  sensors_init();
  sensor_int_init();
  
  /* Setup LED's */  
  P1DIR |= BV(0);
  P0DIR |= BV(4);
  
  P1_0 = 0;
 
   
  start_gyro(); //start the gyro
  init_acc();   //start the accelerometer
  start_mag();
  start_baro();
  
  //zero_mag();
  
  EA = 1;

  SLEEPCMD |= 0x02;  //pm 2
  
  uint8 flag;
  uint8 IDbyte;
  
  
  uint8 baro_stage = 1;

  
   while(1){      
        
     if (RXin)
     {
     //  P0_4 = 1;
    /**************************************************************************/
    /* Read and transmit sensor data                                          */
    
    flush_byte(&active_sensors);
     
    /*---------------------------------------------------------------------*/ 
    //Read accelerometer and gyro
    
     if ( active_sensors & BV(0) )
     {       
       IDbyte = BV(0);
       flush_data(&IDbyte);
      
      while( !( acc_int_status() ) ); //wait for interrupt 
      read_acc();                 //read the accelerometer

      while( !(gyro_int_status() & BV(0) ) ); //wait for interrupt
      read_gyro();

     }
    
    /*---------------------------------------------------------------------*/
    //Read Magnetometer
    //start_interrupts(MAG_INT);
     
     if ( active_sensors & BV(1) )
     {
       
    
       IDbyte = BV(1);
       flush_data(&IDbyte);
       
       //PCON |= 1;
       while( !(mag_status() & 0x08 ) );
       read_mag();
 
      // mag_sleep(TRUE);  
     } 
    
    /*---------------------------------------------------------------------*/
    //Barometer
    
    //uint16 delay_ticks = 0xFA00;
    uint8 baro_res = 2;
    
    
    if ( active_sensors & BV(2) )
    {
       //start_baro();
       //while(delay_ticks--);
      
       IDbyte = BV(2);
       flush_data(&IDbyte);
       
       uint8 nullbyte = 3;
       switch (baro_stage)
       {
       case 1 :
          baro_capture_press(baro_res);
          baro_read_press(FALSE);
          baro_read_temp(FALSE);
          baro_stage++;
          break;
       case 2 :
          baro_read_press(TRUE);
          baro_read_temp(FALSE);
          baro_stage++;
          break;
       case 3 :
          baro_capture_temp();
          baro_read_press(FALSE);
          baro_read_temp(FALSE);
          baro_stage++;
          break;
       case 4 :
         baro_read_press(FALSE);
         baro_read_temp(TRUE);
         baro_stage = 1;
         break;
       }
       /**
       delay_ticks = baro_capture_press(baro_res);
       while(delay_ticks--);
       baro_read_press();

       delay_ticks = baro_capture_temp();
       while(delay_ticks--);
       baro_read_temp();
       **/
      //baro_shutdown();
    }
    
    /*---------------------------------------------------------------------*/
    //Humidity
    
    if ( active_sensors & BV(3) )
    {
      IDbyte = BV(3);
      flush_data(&IDbyte);
      humid_init();
      humid_read_humidity(TRUE);
    }
    
    /*---------------------------------------------------------------------*/

    
    if ( active_sensors & BV(4) )
    {
      IDbyte = BV(4);
      flush_data(&IDbyte);
      baro_read_cal();
    }
    
    //End of line char
    flag = 0x00;
    flush_byte(&flag);
    
   
   // P0_4 = 0;
      
    
    
    if ( !(active_sensors & BV(5)) )        //if autopoll is off
    {
      P0_4 = 1;
      RXin = 0;                             //clear the RX flag
    }else{
      P0_4 = 0;
    }

    }
    
   IEN0 |= 0x04; 
   U0CSR &= ~0x04;
   }
   
}
/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/
_PRAGMA(vector=URX0_VECTOR)
__interrupt void RX0_ISR(void)
{
//  P1_0 = 1;
  IEN0 &= ~0x04;  
  U0CSR &= ~0x04;
  P1_0 ^= 1;
  active_sensors =  U0DBUF;
  RXin = 1;
//  P1_0 = 0;
}

/*Sleep Timer interrupt */

_PRAGMA(vector=ST_VECTOR)
__interrupt void SLEEP_ISR(void)
{
  SLEEP_TIMER_CLEAR();
}


/*
_PRAGMA(vector=P0INT_VECTOR) 
__interrupt void port0_ISR(void)
{
  if ( P0IFG &  BV(0) )         //If side switch
  {
   
  }
  //Clear the CPU interrupt flag for Port_0 PxIFG has to be cleared before PxIF
  P0IFG = 0;
  P0IF = 0;

}
*/