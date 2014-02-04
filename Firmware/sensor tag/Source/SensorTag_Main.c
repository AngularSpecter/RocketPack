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
  //   4     :  Auto poll (1 = auto poll, 0 = ping to poll)
 

   //start up with all sensors running in ping-to-poll mode
   uint8 active_sensors = BV(0) | BV(1) | BV(2) | BV(3) | BV(4);
   
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
  
  P1_0 = 1;
  P0_4 = 1;
  
   
  start_gyro();
   
  init_acc();  //start the accelerometer in sleep mode
 
 // acc_sleep(FALSE);
 // gyro_sleep(FALSE);
 // start_mag();
 // mag_sleep(FALSE);
 // start_baro();
 // humid_init();
  
  acc_int_mode(TRUE);
  
  
  
  
  //zero_mag();
  
  EA = 1;

  SLEEPCMD |= 0x02;  //pm 2
  
  uint8 flag;
  
  
   while(1){      
        
     if (RXin)
     {
    /**************************************************************************/
    /* Read and transmit sensor data                                          */
    
    flush_byte(&active_sensors);
     
    /*---------------------------------------------------------------------*/ 
    //Read accelerometer
    
     if ( active_sensors & BV(0) )
     {
     //  start_interrupts(ACC_INT);  //start the interrupt
       acc_sleep(FALSE);
    
     // PCON |= 1;                  //sleep the chip until the data is ready
      while( !( acc_int_status() ) ); //wait for interrupt 
     // stop_interrupts(ACC_INT); 
    
       read_acc();                 //read the accelerometer
       acc_sleep(TRUE);
     

    
    
    /*---------------------------------------------------------------------*/
    //Read Gyro
    //start_interrupts(GYRO_INT);
     
      gyro_sleep(FALSE);
     
      while( !(gyro_int_status() & BV(0) ) ); //wait for interrupt
   
      read_gyro();
    
      gyro_sleep(TRUE);
     }
    
    /*---------------------------------------------------------------------*/
    //Read Magnetometer
    //start_interrupts(MAG_INT);
     
     if ( active_sensors & BV(1) )
     {
       mag_sleep(FALSE);
    
       //PCON |= 1;
       while( !(mag_status() & 0x08 ) );
       read_mag();
 
       mag_sleep(TRUE);  
     } 
    
    /*---------------------------------------------------------------------*/
    //Barometer
    
    uint8 delay_ticks;
    uint8 baro_res = 8;
    
    if ( active_sensors & BV(2) )
    {
       start_baro();

       delay_ticks = baro_capture_press(baro_res);
       while(delay_ticks--);
       baro_read_press();

     //  delay_ticks = baro_capture_temp(baro_res);
     //  while(delay_ticks--);
     //  baro_read_temp();

      baro_shutdown();
    }
    
    /*---------------------------------------------------------------------*/
    //Humidity
    
    if ( active_sensors & BV(3) )
    {
      humid_init();
      humid_read_humidity();
    }
    
    /*---------------------------------------------------------------------*/

    
    flag = 0x00;
   // flag = '\n';
    flush_byte(&flag);
    
   
    P0_4 ^= 1;
      
    
    
    if ( !(active_sensors & BV(4)) )        //if autopoll is off
    {
      RXin = 0;                             //clear the RX flag
    }else{
   /*   if ( active_sensors & BV(6) )       //if low rate mode, use the timer
      {
        SetSleepTimer(timeout);           //use the sleep timer to time the next
        PCON |= 1;                        //poll and put to sleep
      } */
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
  IEN0 &= ~0x04;  
  U0CSR &= ~0x04;
  P1_0 ^= 1;
  active_sensors =  U0DBUF;
  RXin = 1;
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