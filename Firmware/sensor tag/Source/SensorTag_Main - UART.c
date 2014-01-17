/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
#include <ioCC2541.h>
#include <hal_defs.h>
#include <hal_board_cfg.h>
#include "sleepTimer.h"
//#include <hal_i2c.h>
//#include <hal_sensor.h>
   
#include "uart.h"
#include "sensors.h"

//I2C addresses
#define GYRO_ON()                           st(P1_1 = 1;);
#define GYRO_OFF()                          st(P1_1 = 0;);


#define MASTER_NAK                          0x00        //00000000
#define MASTER_ACK                          0xff        //11111111

#define MASTER_WRITE                        0xe0        //11100000
#define MASTER_READ                         0xC0        //11000000
#define MASTER_SEL                          0xf0        //11110000
 

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
/*************************************************************************************************
          Globals
**************************************************************************************************/
volatile uint32 timeout = 0xFF00;

volatile uint8  byte_number = 0;
volatile bool   byteRX = FALSE;
volatile uint8  mode;

i2cClock_t I2C_CLOCK;
uint8      I2C_DEVICE;
uint8      I2C_REGISTER;
uint8      N_BYTES = 0;
uint8      I2C_DATA[MAX_BYTES];
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
 // sensors_init();
 sensor_int_init();
  
  /* Setup LED's */  
  P1DIR |= BV(0);
  P0DIR |= BV(4);
  
  P1_0 = 1;
  P0_4 = 1;
  
  SLEEPCMD |= 0x02;  //pm 2
  
  IEN0 |= 0x04;  //enable USART0 RX interrupt
  
  EA = 1;
  
  uint8 RXbuffer;
  bool success;
  
  while(1)
  {
    //if we have a byte on the UART to read
    if (byteRX)
    {
      byteRX = FALSE;
      RXbuffer = U0DBUF;
    
      /*************************************************/
      //If  we know how many bytes we have to work with
      if (N_BYTES)
      {
       
        /************************************************/
        //If we are reading, then try and read the bytes from the bus
        if (mode == MASTER_READ)
        {
          //if the register addess is null, then just write straight to the bus
          if (I2C_REGISTER == 0x00)
          {
            success = HalI2CRead(N_BYTES,&I2C_DATA[0]);
          }else{
            success = HalSensorReadReg(I2C_REGISTER,&I2C_DATA[0],N_BYTES);  
          }

        } //end (mode == MASTER_READ)
        
        
        /************************************************/
        //If we are writing data
        if ( (mode == MASTER_WRITE) )
        {
          
          //if all of the data is loaded into the buffer
          if (byte_number-2 == N_BYTES )
          {
            //if the register addess is null, then just write straight to the bus
            if (I2C_REGISTER == 0x00)
            {
              success = HalI2CWrite(N_BYTES,&I2C_DATA[0]);
            }else{
              success = HalSensorWriteReg(I2C_REGISTER,&I2C_DATA[0],N_BYTES);  
            }
          //otherwise, just load the buffer and move on  
          }else{
            I2C_DATA[byte_number-3] = RXbuffer;  
          }
        } //end (mode == MASTER_WRITE)
        
        
      /***************************************************/  
      //If there are no bytes to read, then check for a change in mode  
      }else{
        //if we are switching speeds
        if (RXbuffer == MASTER_SEL){
           byte_number = 0; // reset byte count
           mode        = MASTER_SEL;        
        } 
        
        //if we starting a write
        if (RXbuffer == MASTER_WRITE){
           byte_number = 0; // reset byte count
           mode        = MASTER_WRITE;        
        }
        
        //if we are starting a read
        if (RXbuffer == MASTER_READ){
           byte_number = 0; // reset byte count
           mode        = MASTER_READ;        
        }
      
      } //end if(!N_BYTES)
      
      /***********************************************************************/
      //If we are on byte number 1, we are processing a command
      if (byte_number == 1)
      {
          
        //if we are selecting a new device
        if( mode == MASTER_SEL)
        {
          success = FALSE;
          
          switch (RXbuffer)
          {
          case ACC_I2C_ADDRESS :
              I2C_DEVICE = ACC_I2C_ADDRESS;
              I2C_CLOCK  = ACC_SPEED;
              success = TRUE;
              break;
          case GYRO_I2C_ADDRESS :
              I2C_DEVICE = GYRO_I2C_ADDRESS;
              I2C_CLOCK  = GYRO_SPEED;
              success = TRUE;
              break;
          case MAG_I2C_ADDRESS :
              I2C_DEVICE = MAG_I2C_ADDRESS;
              I2C_CLOCK  = MAG_SPEED;
              success = TRUE;
              break;          
          case BARO_I2C_ADDRESS :
              I2C_DEVICE = BARO_I2C_ADDRESS;
              I2C_CLOCK  = BARO_SPEED;
              success = TRUE;
              break;
          case HUMID_I2C_ADDRESS :
              I2C_DEVICE = HUMID_I2C_ADDRESS;
              I2C_CLOCK  = HUMID_SPEED;
              success = TRUE;
              break;
          case TMP006_I2C_ADDRESS :  
              I2C_DEVICE = TMP006_I2C_ADDRESS;
              I2C_CLOCK  = TMP006_SPEED;
              success = TRUE;
              break;    
          }
          
          if (success){
            HalI2CInit(I2C_DEVICE, I2C_CLOCK);
            //UART_SEND_ACK()
          }
          else
          {
            //UART_SEND_NACK();
          }
          
        } //end if( mode == MASTER_SEL)
        
        
        //IF we are reading or writing a register value
        if( (mode == MASTER_WRITE) || (mode == MASTER_READ) )
        {
          I2C_REGISTER = RXbuffer;
          //UART_SEND_ACK();
        }
        
      }//end if(byte_numer ==1)
      
      
      /***********************************************************************/
      //Byte number 2 is the number of bytes we are going to read or write.
      
        if (byte_number == 2)
       {
         if (RXbuffer <= MAX_BYTES)
         {
           N_BYTES = RXbuffer;
           //UART_SEND_ACK();
         }
         else
         {
           N_BYTES = 0;
           //UART_SEND_NAK();
         }
       }  //end if(byte_number == 2)
      
    }
    
    IEN0 |= 0x04;  //enable USART0 RX interrupt
    //SetSleepTimer(timeout);
    //PCON |= 1; 

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
  byteRX = TRUE;
  byte_number++;  
}

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

  
  //Clear the CPU interrupt flag for Port_0 PxIFG has to be cleared before PxIF
  P0IFG = 0;
  P0IF = 0;

}
