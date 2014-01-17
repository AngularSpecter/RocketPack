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
   volatile uint32 timeout = 0xFF00;
  
   uint16 gyro[3];
   uint16 gyro_temp;
   uint16 accelerometer[3];
   uint16 magnetometer[3];
   
   uint16 barometer;
   uint16 baro_temp;
   
/*************************************************************************************************
          Main
**************************************************************************************************/
void main(void)
{
  
  HAL_BOARD_INIT();
  
  UART_init();
  ENABLE_RX();
 
  /*setup sensors*/
  sensors_init();
  sensor_int_init();
  
  /* Setup LED's */  
  P1DIR |= BV(0);
  P0DIR |= BV(4);
  
  P1_0 = 1;
  P0_4 = 1;
  
  SLEEPCMD |= 0x02;  //pm 2
   
  start_gyro();
  //zero_gyro();
  read_gyro(&gyro[0]);
   
  init_acc();  //start the accelerometer in sleep mode
  acc_int_mode(TRUE);
  
  start_mag();
  //zero_mag();
  
  EA = 1;

  unsigned char buff_len = 6;
  
   while(1){ 
     //set sleep timer for next poll
    // SetSleepTimer(timeout);

     char *output_buffer;
     
    /*---------------------------------------------------------------------*/ 
    //Read accelerometer
    
    start_interrupts(ACC_INT);  //start the interrupt
    acc_sleep(FALSE);
    
    PCON |= 1;                  //sleep the chip until the data is ready
     
    read_acc(&accelerometer[0]);  //read the accelerometer
    acc_sleep(TRUE);
    
    
  //  U0DBUF = '\r';
    //U0DBUF = '\n';
    /*
    itoa(accelerometer[0], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    U0DBUF = ':';
    
    itoa(accelerometer[1], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    U0DBUF = ':';
    
    itoa(accelerometer[2], output_buffer, 65535);
    uartSend(output_buffer,buff_len); */
    
    
    /*---------------------------------------------------------------------*/
    //Read Gyro
    //start_interrupts(GYRO_INT);
    //gyro_sleep(FALSE);
     
   // while( !(gyro_int_status() & BV(0) ) ); //wait for interrupt
   
    //read_gyro(&gyro[0]);
    //read_gyro_temp(&gyro_temp);
    
    //U0DBUF = '\r';
    //U0DBUF = '\n';
       
    //char *output_buffer;
    //itoa(gyro[0], output_buffer, 65535);
    //uartSend(output_buffer,buff_len);
    //U0DBUF = ':';
      
    //itoa(gyro[1], output_buffer, 65535);
    //uartSend(output_buffer,buff_len);
    //U0DBUF = ':';

    //itoa(gyro[2], output_buffer, 65535);
    //uartSend(output_buffer,buff_len);
    
    
    /*---------------------------------------------------------------------*/
    //Read Magnetometer
    //start_interrupts(MAG_INT);
    mag_sleep(FALSE);
    
    //PCON |= 1;
    while( !(mag_status() & 0x08 ) );
    read_mag(&magnetometer[0]);

    /*
    U0DBUF = '\r';
    U0DBUF = '\n';
       
    char *output_buffer;
    itoa(magnetometer[0], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    U0DBUF = ':';
      
    itoa(magnetometer[1], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    U0DBUF = ':';

    itoa(magnetometer[2], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    */ 
    mag_sleep(TRUE);  
    
    
    /*---------------------------------------------------------------------*/
    //Barometer
    
    uint8 delay_ticks;
    uint8 baro_res = 2;
    
    start_baro();
    delay_ticks = baro_capture_press(baro_res);
    while(delay_ticks--);
    baro_read_press(&barometer);
    
    delay_ticks = baro_capture_temp(baro_res);
    while(delay_ticks--);
    baro_read_temp(&baro_temp);
    
    baro_shutdown();
  /*
    U0DBUF = '\r';
    U0DBUF = '\n';    
    itoa(barometer, output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    U0DBUF = ':';
    
    itoa(baro_temp, output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    */

    
    
    /*---------------------------------------------------------------------*/
    //Humidity
    
    humid_init();
    humid_read_humidity();
    
    
    /*---------------------------------------------------------------------*/
    //IR Temperature
    
     IR_on();
     start_interrupts(IR_INT);
    
     while(!IR_data_ready());
     //PCON |= 1;
     
     uint16 IR[2];
     
     read_IR(&IR[0]);
     
    
   // T2 = (uint16)SleepReadTimer()- (uint16)T1;
    
  // U0DBUF = T2;
    
    SetSleepTimer(timeout);
    PCON |= 1; 

    }
   
}
/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*Sleep Timer interrupt */
_PRAGMA(vector=ST_VECTOR)
__interrupt void SLEEP_ISR(void)
{
  SLEEP_TIMER_CLEAR();
   P1_0 ^= 1 ;
}

_PRAGMA(vector=P0INT_VECTOR) 
__interrupt void port0_ISR(void)
{
  if ( P0IFG &  BV(0) )         //If side switch
  {
   // sensorReady |= 0x01;
  }
  
 //if ( P0IFG & ACC_INT )  //If Accelerometer interrupt
 // {
 //     stop_interrupts(ACC_INT);  //stop the accelerometer interrupt
 // }
  
/*    if ( P0IFG & GYRO_INT )  //If Gyro interrupt
  {
      stop_interrupts(GYRO_INT);  //stop the gyro interrupt
  }
  
      if ( P0IFG & MAG_INT )  //If Mag interrupt
  {
      stop_interrupts(MAG_INT);  //stop the mag interrupt
  }
*/

  
  //Clear the CPU interrupt flag for Port_0 PxIFG has to be cleared before PxIF
  P0IFG = 0;
  P0IF = 0;

}
