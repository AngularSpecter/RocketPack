#include "msp430g2553.h"
#include "uart.h"
#include "macros.h"


unsigned char select_sensor(unsigned char DEVICE);


#define MASTER_NAK                          0x00        //00000000
#define MASTER_ACK                          0xff        //11111111

#define MASTER_WRITE                        0xe0        //11100000
#define MASTER_READ                         0xC0        //11000000
#define MASTER_SEL                          0xf0        //11110000


#define ACC_I2C_ADDRESS                     0x0F        //00001111
#define GYRO_I2C_ADDRESS                    0x68        //01101000
#define MAG_I2C_ADDRESS          	        0x0E        //00001110
#define BARO_I2C_ADDRESS                    0x77        //01110111
#define HUMID_I2C_ADDRESS                   0x40        //01000000
#define TMP006_I2C_ADDRESS                  0x44        //01000100



//command format
//Select Dev:   MASTER_SEL, DEVICE ;  MASTER_ACK
//Write reg :   MASTER_WRITE, REGISTER ADDRESS, n_bytes, byte 1, byte 2, nyte n  ; MASTER_ACK
//Read reg  :   MASTER_READ, REGISTER ADDRESS, n_bytes ; byte_1, byte_2, byte_n
/*----------------------------------------------------------------------
  Gyro
-------------------------------------------------------------------------*/

#define GYRO_I2C_ADDRESS                    0x68
#define GYRO_DATA_SIZE                      8

/* GYRO register addresses */

#define GYRO_REG_WHOAMI                     0x00 // R/W

// Offset configuration registers
#define GYRO_REG_XOFFS_USRH                 0x0C // R/W
#define GYRO_REG_XOFFS_USRL                 0x0D // R/W
#define GYRO_REG_YOFFS_USRH                 0x0E // R/W
#define GYRO_REG_YOFFS_USRL                 0x0F // R/W
#define GYRO_REG_ZOFFS_USRH                 0x10 // R/W
#define GYRO_REG_ZOFFS_USRL                 0x11 // R/W

// Configuration registers

#define GYRO_REG_AUX_VDDIO                  0x13 // R/W
#define GYRO_REG_AUX_SLV_ADDR               0x14 // R/W
#define GYRO_REG_SMPLRT_DIV                 0x15 // R/W
#define GYRO_REG_DLPF_FS                    0x16 // R/W

#define GYRO_REG_AUX_BURST_ADDR             0x18 // R/W


// Sensor data registers
#define GYRO_REG_TEMP_OUT_H                 0x1B // R
#define GYRO_REG_TEMP_OUT_L                 0x1C // R
#define GYRO_REG_GYRO_XOUT_H                0x1D // R
#define GYRO_REG_GYRO_XOUT_L                0x1E // R
#define GYRO_REG_GYRO_YOUT_H                0x1F // R
#define GYRO_REG_GYRO_YOUT_L                0x20 // R
#define GYRO_REG_GYRO_ZOUT_H                0x21 // R
#define GYRO_REG_GYRO_ZOUT_L                0x22 // R


// FIFO registers
#define GYRO_REG_FIFO_COUNTH                0x3A // R
#define GYRO_REG_FIFO_COUNTL                0x3B // R
#define GYRO_REG_FIFO_R                     0x3C // R

// User control and Power Management registers
#define GYRO_REG_USER_CTRL                  0x3D // R/W


/* GYRO Register Bit masks */
#define GYRO_REG_FIFO_EN_TEMP_OUT           0x80
#define GYRO_REG_FIFO_EN_GYRO_XOUT          0x40
#define GYRO_REG_FIFO_EN_GYRO_YOUT          0x20
#define GYRO_REG_FIFO_EN_GYRO_ZOUT          0x10
#define GYRO_REG_FIFO_EN_AUX_XOUT           0x08
#define GYRO_REG_FIFO_EN_AUX_YOUT           0x04
#define GYRO_REG_FIFO_EN_AUX_ZOUT           0x02
#define GYRO_REG_FIFO_EN_FIFO_FOOTER        0x01

#define GYRO_USER_CTRL_DMP_EN               0x80
#define GYRO_USER_CTRL_FIFO_EN              0x40
#define GYRO_USER_CTRL_AUX_IF_EN            0x20
#define GYRO_USER_CTRL_AUX_IF_RST_EN        0x08
#define GYRO_USER_CTRL_DMP_RST              0x04
#define GYRO_USER_CTRL_FIFO_RST             0x02
#define GYRO_USER_CTRL_GYRO_RST             0x01


/* FIFO config ********************************/
#define GYRO_REG_FIFO_EN                    0x12 // R/W

#define GYRO_FIFO_T                         0x80
#define GYRO_FIFO_X                         0x40
#define GYRO_FIFO_Y                         0x20
#define GYRO_FIFO_Z                         0x10

/* Interrupt Configuration ********************/

//Interrupt
#define GYRO_REG_INT_CFG                    0x17 // R/W

#define GYRO_INT_ACTL                       0x80
#define GYRO_INT_OPEN                       0x40
#define GYRO_INT_LATCH                      0x20
#define GYRO_INT_CLEAR                      0x10
#define GYRO_INT_EXT                        0x08
#define GYRO_INT_PLL                        0x04
#define GYRO_INT_DATA                       0x01

//Interrupt status register
#define GYRO_REG_INT_STATUS                 0x1A // R

#define GYRO_INT_STATUS_FULL                0x80
#define GYRO_INT_STATUS_AUXERR              0x08
#define GYRO_INT_STATUS_PLLRDY              0x04
#define GYRO_INT_STATUS_DATARDY             0x01

/* Power management  *********************************/
#define GYRO_REG_PWR_MGM                    0x3E // R/W

#define GYRO_PWR_MGM_H_RESET                0x80
#define GYRO_PWR_MGM_SLEEP                  0x40
#define GYRO_PWR_MGM_STBY_XG                0x20
#define GYRO_PWR_MGM_STBY_YG                0x10
#define GYRO_PWR_MGM_STBY_ZG                0x08
#define GYRO_PWR_MGM_STBY_ALL               0x38  // All axes

// Clock select
#define GYRO_PWR_MGM_CLOCK_INT_OSC          0x00
#define GYRO_PWR_MGM_CLOCK_PLL_X            0x01
#define GYRO_PWR_MGM_CLOCK_PLL_Y            0x02
#define GYRO_PWR_MGM_CLOCK_PLL_Z            0x03
#define GYRO_PWR_MGM_CLOCK_PLL_32768KHZ     0x04
#define GYRO_PWR_MGM_CLOCK_PLL_19_2MHZ      0x05
#define GYRO_PWR_MGM_CLOCK_STOP             0x07

void start_gyro(void);
void read_gyro(unsigned int *buffer);


/*------------------------------------------------------------------------------
###############################################################################
Accelerometer
------------------------------------------------------------------------------*/
#define ACC_I2C_ADDRESS                         0x0F

//Data output registers
#define  ACC_XOUT_L                             0x06
#define  ACC_XOUT_H                             0x07
#define  ACC_YOUT_L                             0x08
#define  ACC_YOUT_H                             0x09
#define  ACC_ZOUT_L                             0x0A
#define  ACC_ZOUT_H                             0x0B

#define ACC_DCST_RESP                           0x0C

#define ACC_WHO_AM_I                            0x0F

//Interrupt status Registers
#define ACC_INT_SOURCE1                         0x16
  #define ACC_WUFS                                0x04   //woken up from sleep
  #define ACC_INT_DRDY                            0x10   //New Data ready

#define ACC_INT_SOURCE2                         0x17     //motion reporting
  #define ACC_ZPWU                                0x01
  #define ACC_ZNWU                                0x02
  #define ACC_YPWU                                0x04
  #define ACC_YNWU                                0x08
  #define ACC_XPWU                                0x10
  #define ACC_XNWU                                0x20

#define ACC_STATUS                              0x18   //interrupt status
  #define ACC_INT_STATUS                          0x10  //event has occured

#define ACC_INT_REL                             0x1A   //interrupt release

//Control registers (must set PC1 bit in REG1 to 0 first
  /*---------------------------*/
  /*GSEL1  |  GSEL0  |  Range  */
  /*---------------------------*/
  /*  0    |    0    |  +/- 2g  */
  /*  0    |    1    |  +/- 4g  */
  /*  1    |    0    |  +/- 8g  */
  /*  1    |    1    |  +/- 8g 14bit  */

#define ACC_CTRL_REG1                           0x1B
  #define ACC_WUFE                                0x02  //Wake up on motion detect
  #define ACC_GSEL0                               0x08  //Range 1
  #define ACC_GSEL1                               0x10  //Range 2
  #define ACC_DRDYE                               0x20  //interrupt on new data
  #define ACC_RES                                 0x40  //0=low, 1=high resolution
  #define ACC_PC1                                 0x80  // 0=standby, 1=operating


  /*------------------------------------*/
  /*  OWUFA | OWUFB | OWUFC | RATE      */
  /*------------------------------------*/
  /*    0   |   0   |   0   | .781  Hz  */
  /*    0   |   0   |   1   | 1.563 Hz  */
  /*    0   |   1   |   0   | 3.125 Hz  */
  /*    0   |   1   |   1   | 6.25  Hz  */
  /*    1   |   0   |   0   | 12.5  Hz  */
  /*    1   |   0   |   1   | 25    Hz  */
  /*    1   |   1   |   0   | 50    Hz  */
  /*    1   |   1   |   1   | 100   Hz  */

#define ACC_CTRL_REG2                           0x1D
  #define ACC_OWUFC                               0x01  //output data rate
  #define ACC_OWUFB                               0x02  //output data rate
  #define ACC_OWUFA                               0x04  //output data rate
  #define ACC_DCST                                0x10  //self test
  #define ACC_SRST                                0x80  //self reboot (1 until reboot done)


#define ACC_INT_CTRL_REG1                       0x1E
  #define ACC_IEL                                 0x08  //0=latch, 1=pulse
  #define ACC_IEA                                 0x10  //polarity 0=low, 1=high
  #define ACC_IEN                                 0x20  //enable interrupt pin

#define ACC_INT_CTRL_REG2                       0x1F    //which motions interrupt
  #define ACC_ZPWU                                0x01
  #define ACC_ZNWU                                0x02
  #define ACC_YPWU                                0x04
  #define ACC_YNWU                                0x08
  #define ACC_XPWU                                0x10
  #define ACC_XNWU                                0x20
  #define ACC_ALLWU                               0x3F


/*  OSAA | OSAB | OSAC | OSAD | Output Data Rate | LPF Roll-Off  */
/*   1      0      0      0      0.781Hz            0.3905Hz     */
/*   1      0      0      1      1.563Hz            0.781Hz      */
/*   1      0      1      0      3.125Hz            1.563Hz      */
/*   1      0      1      1      6.25Hz             3.125Hz      */
/*   0      0      0      0      12.5Hz             6.25Hz       */
/*   0      0      0      1      25Hz               12.5Hz       */
/*   0      0      1      0      50Hz               25Hz         */
/*   0      0      1      1      100Hz              50Hz         */
/*   0      1      0      0      200Hz              100Hz        */
/*   0      1      0      1      400Hz              200Hz        */
/*   0      1      1      0      800Hz              400Hz        */
/*   0      1      1      1      1600Hz             800Hz        */

#define ACC_DATA_CTRL_REG                       0x21
  #define ACC_OSAD                                0x01
  #define ACC_OSAC                                0x02
  #define ACC_OSAB                                0x04
  #define ACC_OSAA                                0x08

//Time motion must be present before a wake up event
#define ACC_WAKEUP_TIMER                        0x29
#define ACC_WAKEUP_THRESH                       0x6A

#define ACC_SELF_TEST                           0x3A

void init_acc(void);


