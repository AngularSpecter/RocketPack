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

<<<<<<< HEAD
//I2C addresses
#define GYRO_ON()                           st(P1_1 = 1;);
#define GYRO_OFF()                          st(P1_1 = 0;);


#define MASTER_NAK                          0x00        //00000000
#define MASTER_ACK                          0xff        //11111111

#define MASTER_WRITE                        0xe0        //11100000
#define MASTER_READ                         0xC0        //11000000
#define MASTER_SEL                          0xf0        //11110000

#define DATA_READ                           0x80

#define ACC_I2C_ADDRESS                     0x0F        //00001111
#define GYRO_I2C_ADDRESS                    0x68        //01101000
#define MAG_I2C_ADDRESS          	    0x0E        //00001110
#define BARO_I2C_ADDRESS                    0x77        //01110111
#define HUMID_I2C_ADDRESS                   0x40        //01000000
#define TMP006_I2C_ADDRESS                  0x44        //01000100

#define ACC_SPEED                          i2cClock_267KHZ
#define GYRO_SPEED                         i2cClock_533KHZ
#define MAG_SPEED                          i2cClock_267KHZ
#define BARO_SPEED                         i2cClock_267KHZ
#define HUMID_SPEED                        i2cClock_267KHZ
#define TMP006_SPEED                       i2cClock_533KHZ
      
#define MAX_BYTES                          2

//command format
//Select Dev:   MASTER_SEL, DEVICE ;  MASTER_ACK
//Write reg :   MASTER_WRITE, REGISTER ADDRESS, n_bytes, byte 1, byte 2, nyte n  ; MASTER_ACK
//Read reg  :   MASTER_READ, REGISTER ADDRESS, n_bytes ; byte_1, byte_2, byte_n
=======
>>>>>>> origin/streamlined
/*************************************************************************************************
          Globals
**************************************************************************************************/
   volatile uint32 timeout = 0x000F;
  
  //Mode bit mask
  // 0 -> 3  :  Sensors
  //   4     :  Baro calibration data
  //   5     :  Auto poll (1 = auto poll, 0 = ping to poll)

 

<<<<<<< HEAD
i2cClock_t I2C_CLOCK;
uint8      I2C_DEVICE;
uint8      I2C_REGISTER;
uint8      N_BYTES = 0;
uint8      I2C_DATA[MAX_BYTES];

#define BUFFER_LEN  8
  
volatile uint8 RXbuffer[BUFFER_LEN];
volatile uint8 read_pos  = 0;
volatile uint8 write_pos = 0;
uint8 bytes_to_proc = 0;
  
uint8 current_byte;

uint8 timer_flag = 0;
=======
   //start up with all sensors running in ping-to-poll mode
   uint8 active_sensors = BV(0) | BV(1) | BV(2) | BV(3) | BV(5);
   
volatile uint8 RXin = 1;
   
>>>>>>> origin/streamlined
/*************************************************************************************************
          Main
**************************************************************************************************/
void main(void)
{
 
  HAL_BOARD_INIT();
   P1_1 = 1;
   P0_7 = 1;
  
  /*Configure Side Switch*/
    P0SEL &= ~(BV(0));    /* Set pin function to GPIO */
    P0DIR &= ~(BV(0));    /* Set pin direction to Input */
    P0IEN |=   BV(0);     /* Enable Interrupt */
  
  
  /*Configure sensor interrupts*/
  
    #define GYRO_INT        BV(1)
    #define ACC_INT         BV(2)
    #define MAG_INT         BV(6)
    #define IR_INT          BV(3)
    
    #define GYRO_INT_ON()   st(P0IEN |= GYRO_INT;);
    #define GYRO_INT_OFF()  st(P0IEN &= ~GYRO_INT;);
        
    #define ACC_INT_ON()    st(P0IEN |= ACC_INT;);
    #define ACC_INT_OFF()   st(P0IEN &= ~ACC_INT;);

    #define MAG_INT_ON()    st(P0IEN |= MAG_INT;);
    #define MAG_INT_OFF()   st(P0IEN &= ~MAG_INT;);
    
    #define IR_INT_ON()     st(P0IEN |= IR_INT;);
    #define IR_INT_OFF()    st(P0IEN &= ~IR_INT;);

    
    //Gyro Interrupt  P0.1
    P0SEL &= ~(BV(1));    /* Set pin function to GPIO */
    P0DIR &= ~(BV(1));    /* Set pin direction to Input */
    
    //Accelerometer Interrupt  P0.2
    P0SEL &= ~(BV(2));    /* Set pin function to GPIO */
    P0DIR &= ~(BV(2));    /* Set pin direction to Input */
    
    //Magnetometer Interrupt  P0.6
    P0SEL &= ~(BV(6));    /* Set pin function to GPIO */
    P0DIR &= ~(BV(6));    /* Set pin direction to Input */

    //IR Temp Interrupt  P0.3
    P0SEL &= ~(BV(3));    /* Set pin function to GPIO */
    P0DIR &= ~(BV(3));    /* Set pin direction to Input */
    
    P0INP |= BV(1) | BV(2) | BV(6) | BV(3);  //high-z the pins
    //P2INP |= BV(5);    //enable pull down resistors on P0
    
    /* Clear any pending interrupts */  
    P0IFG = 0;
    P0IF  = 0;
    
    IEN1  |= BV(5);    /* enable CPU interrupt for all of port 0*/ 
  
  
  /*Configure UART*/
  UART_init();
<<<<<<< HEAD

 
  /*setup sensors*/
  sensors_init();
  sensor_int_init();
  
  
  start_gyro();
  init_acc();
  start_mag();
  start_baro();
  humid_init();
=======
  U0CSR &= ~0x04;
  ENABLE_RX();
  
  /*setup sensors*/
  sensors_init();
  sensor_int_init();
>>>>>>> origin/streamlined
  
  /* Setup LED's */  
  P1DIR |= BV(0);
  P0DIR |= BV(4);
  
<<<<<<< HEAD

  
  P1_0 = 1;
  P0_4 = 1;
  
  SLEEPCMD |= 0x02;  //pm 2
  
=======
  P1_0 = 0;
 
   
  start_gyro(); //start the gyro
  init_acc();   //start the accelerometer
  start_mag();
  start_baro();
  
  //zero_mag();
  
>>>>>>> origin/streamlined
  EA = 1;

  SLEEPCMD |= 0x02;  //pm 2
  
<<<<<<< HEAD

  bool success;
  
  
  
  SetSleepTimer(timeout);
  while(1)
  {
    
    if (timer_flag)
    {
    timer_flag = 0;
   // read_sensor(ACC_I2C_ADDRESS);
    read_sensor(GYRO_I2C_ADDRESS);
   // read_sensor(MAG_I2C_ADDRESS);
   // read_sensor(BARO_I2C_ADDRESS);
   // read_sensor(HUMID_I2C_ADDRESS);
    
    
    SetSleepTimer(timeout);
    //PCON |= 1;  
=======
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
>>>>>>> origin/streamlined
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
   
    IEN0 |= BV(2);    
   
  }//end WHILE
}
/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/
_PRAGMA(vector=URX0_VECTOR)
__interrupt void RX0_ISR(void)
{
<<<<<<< HEAD
  IEN0 &= ~BV(2);                               //disable the interrupt
  RXbuffer[write_pos++] = U0DBUF;               //push value to buffer
  bytes_to_proc++;                              //incriment the byte count
  if (write_pos == BUFFER_LEN) write_pos = 0;   //roll over the buffer counter
  IEN0 |= BV(2);                                //restart interrupt
  P1_0 ^= 1 ;
=======
//  P1_0 = 1;
  IEN0 &= ~0x04;  
  U0CSR &= ~0x04;
  P1_0 ^= 1;
  active_sensors =  U0DBUF;
  RXin = 1;
//  P1_0 = 0;
>>>>>>> origin/streamlined
}

/*Sleep Timer interrupt */

_PRAGMA(vector=ST_VECTOR)
__interrupt void SLEEP_ISR(void)
{
  SLEEP_TIMER_CLEAR();
<<<<<<< HEAD
   P1_0 ^= 1 ;
   timer_flag = 1;
=======
>>>>>>> origin/streamlined
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
