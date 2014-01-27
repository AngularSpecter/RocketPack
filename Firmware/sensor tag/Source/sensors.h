#include <ioCC2541.h>
#include <hal_defs.h>
#include <hal_board_cfg.h>
#include <hal_i2c.h>
#include <hal_sensor.h>
#include "uart.h"


void sensors_init(void);
void sensor_int_init(void);
void start_interrupts(uint8 mask);
void stop_interrupts(uint8 mask);
void burstRead(uint8 startADD, uint8 n_buffers);

#define GYRO_INT        BV(1)
#define ACC_INT         BV(2)
#define MAG_INT         BV(6)
#define IR_INT          BV(3)

#define DCDC_ON          st(P0_7 = 1;);
#define DCDC_OFF         st(P0_7 = 0;);

/*----------------------------------------------------------------------
  Gyro
-------------------------------------------------------------------------*/  
#define GYRO_ON()                           st(P1_1 = 1;);
#define GYRO_OFF()                          st(P1_1 = 0;);

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
void read_gyro();
void read_gyro_temp(uint16 *buffer);
uint8 gyro_int_status(void);
void gyro_sleep(bool sleep);
void zero_gyro(void);


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
void acc_int_mode(bool interrupt);
void acc_sleep(bool sleep);
void read_acc();


/*------------------------------------------------------------------------------
###############################################################################
Magnetometer
------------------------------------------------------------------------------*/
#define MAG_REG_READ_ALL_LEN        6
#define MAG_REG_FAST_READ_LEN       3

#define HAL_MAG3110_I2C_ADDRESS 	  0x0E
#define HAL_MAG3110_PERIOD_MASK 	  0x10
#define HAL_MAG3110_ENABLE_MASK 	  0x01
  
#define MAG_WHO_AM_I              0x07 // Read  
  
//Magnetometer Status
#define MAG_DR_STATUS             0x00 // Read
  #define MAG_XDR                   0x01 // x data ready
  #define MAG_YDR                   0x02 // y data ready
  #define MAG_ZDR                   0x04 // z data ready
  #define MAG_ZYXDR                 0x08 // data ready on any axis
  #define MAG_XOW                   0x10 // old x data overwritten before read
  #define MAG_YOW                   0x20 // old y data overwritten before read
  #define MAG_ZOW                   0x40 // old z data overwritten before read
  #define MAG_ZYXOW                 0x80 // old data overwritten before read  
  
//Data access registers   
#define MAG_X_MSB     		  0x01 // Read
#define MAG_X_LSB     		  0x02 // Read
#define MAG_Y_MSB     		  0x03 // Read
#define MAG_Y_LSB     		  0x04 // Read
#define MAG_Z_MSB     		  0x05 // Read
#define MAG_Z_LSB     		  0x06 // Read
  
//System Mode
#define MAG_SYSMOD                0x08 // Read
  // 00 Standby mode
  // 01 Active mode, raw data
  // 11 Active mode, user-corrected data
  
//Magnetometer offsets  
#define MAG_OFF_X_MSB             0x09 // Read/Write
#define MAG_OFF_X_LSB     	  0x0A // Read/Write
#define MAG_OFF_Y_MSB     	  0x0B // Read/Write
#define MAG_OFF_Y_LSB     	  0x0C // Read/Write
#define MAG_OFF_Z_MSB     	  0x0D // Read/Write
#define MAG_OFF_Z_LSB          	  0x0E // Read/Write
  
//Die Temperature (1*C accuracy)  
#define MAG_DIE_TEMP    	  0x0F // Read
  
//Control Registers  
#define MAG_CTRL_1      	  0x10 // Read/Write
  #define MAG_CTRL1_AC              0x01 // Operating mode, 0 = stdby, 1 = active
  #define MAG_CTRL1_TM              0x02 // Trigger measurement
  #define MAG_CTRL1_FR              0x04 // Fast Read, skip LSB
  #define MAG_CTRL1_OS0             0x08 // Oversampling ratio.  
  #define MAG_CTRL1_OS1             0x10 // Number of samples to average
  #define MAG_CTRL1_DR0             0x20 // Output Data rate
  #define MAG_CTRL1_DR1             0x40
  #define MAG_CTRL1_DR2             0x80
   
  /*  DR2 | DR1 | DR0 | ADC Rate
       0     0     0    1280 HZ
       0     0     1    640  HZ
       0     1     0    320  HZ
       0     1     1    160  HZ
       1     0     0    80   HZ  
  */
  
#define MAG_CTRL_2      	  0x11 // Read/Write
  #define MAG_RST                   0x10 // Reset sensor (stays high while in reset)
  #define MAG_RAW                   0x20 // RAW mode, 0=apply user offsets, 1=no offset
  #define MAG_AUTO_MRST             0x80 // auto reset before acquisition
 
#define MAG_READ_START	            MAG_X_MSB

//1280 Hz ADC - 900 uA draw
#define MAG_1280_10HZ                  0x18 // 10Hz 128 osr  .25 uT
#define MAG_1280_20HZ                  0x10 // 20Hz 64  osr  .30 uT
#define MAG_1280_40HZ                  0x08 // 40Hz 32  osr  .35 uT

//640 Hz ADC - 550 uA draw
#define  MAG_640_10HZ                  0x30  // .3 uT
#define  MAG_640_20HZ                  0x28  // .35 uT
#define  MAG_640_40HZ                  0x20  // .4  uT
  
//5Hz data rate
#define MAG_640_5HZ                     0x38  //550 uA,    .25 uT
#define MAG_320_5HZ                     0x50  //275 uA,    .3 uT
#define MAG_160_5HZ                     0x68  //137.5 uA,  .35 uT
#define MAG_80_5Hz                      0x80  //68.8 uA,   .4 uT


void start_mag(void);
void mag_sleep(bool sleep);
void read_mag();
void zero_mag(void);
uint8 mag_status(void);

/*------------------------------------------------------------------------------
###############################################################################
Barometer
------------------------------------------------------------------------------*/
// Sensor I2C address
#define BARO_I2C_ADDRESS                0x77
#define BARO_I2C_SLAVE                  0x88 // Read
  
// C953 data lengths
#define BARO_DATA_LEN                       2
#define BARO_CAL_LEN                        16
 
  
// C953 Register Addresses
#define BARO_IFACE_SETTINGS                 0x87 // Read/Write
#define BARO_ADDR_RESET                     0xF0 // Read/Write
  
  
  
/**
 * Temperature/Pressure measurement cmd, with the following CTRL_CMD bitfields:
 *
 * b0: sco, set to 1 for conversion start
 * b1-b2: pt, set 00 for pressure meas and 01 for temperature meas
 * b3-b4: mode, set values according to enum t5400_op_mode
 * b5: zero, set to 0 for applying the command correctly
 * b6-b7: don't care
 */  
#define BARO_COMMAND_CTRL          	    0xF1 // Read/Write
#define BARO_PRESS                          0x00 // Execute Pressure Measurement Only
#define BARO_TEMP             	            0x02 // Execute Temperature Measurement Only

  // C953 Resolution Setting Bit Masks
#define BARO_RES_2         		    0x00 // 2 ms  - 8.8Pa RMS
#define BARO_RES_8                          0x08 // 8 ms  - 6.4Pa RMS
#define BARO_RES_16         		    0x10 // 16 ms - 5Pa RMS
#define BARO_RES_64      		    0x18 // 64 ms - 4.4Pa RMS  
  
/** Data Read Registers
*/  
#define BARO_TEMP_LSB              0xF3 // Read
#define BARO_TEMP_MSB              0xF4 // Read

#define BARO_PRESS_LSB       	    0xF5 // Read
#define BARO_PRESS_MSB             0xF6 // Read

// C953 Register Addresses for calibration coefficients
#define BARO_CALIBRATION_1_LSB          0x8E
#define BARO_CALIBRATION_1_MSB          0x8F
#define BARO_CALIBRATION_2_LSB          0x90
#define BARO_CALIBRATION_2_MSB          0x91
#define BARO_CALIBRATION_3_LSB          0x92
#define BARO_CALIBRATION_3_MSB          0x93
#define BARO_CALIBRATION_4_LSB          0x94
#define BARO_CALIBRATION_4_MSB          0x95
#define BARO_CALIBRATION_5_LSB          0x96
#define BARO_CALIBRATION_5_MSB          0x97
#define BARO_CALIBRATION_6_LSB          0x98
#define BARO_CALIBRATION_6_MSB          0x99
#define BARO_CALIBRATION_7_LSB          0x9A
#define BARO_CALIBRATION_7_MSB          0x9B
#define BARO_CALIBRATION_8_LSB          0x9C
#define BARO_CALIBRATION_8_MSB          0x9D


// C953 Commands
#define BARO_TEMP_READ_COMMAND		( BARO_TEMP | 0x01 )
#define BARO_PRESS_READ_COMMAND		( BARO_PRESS| 0x01 )
#define BARO_OFF_COMMAND		0x00
#define BARO_RESET_COMMAND              0x73

void start_baro(void);
uint16 baro_capture_press(uint8 resolution);
void baro_read_press();
uint16 baro_capture_temp(uint8 resolution);
void baro_read_temp();
void baro_shutdown(void);


/*------------------------------------------------------------------------------
###############################################################################
Humidity
------------------------------------------------------------------------------*/
  // Sensor I2C address
#define HUMID_I2C_ADDRESS      0x40

#define S_REG_LEN                  2
#define DATA_LEN                   3

// Internal commands
#define HUMID_TEMP_T_H         0xE3 // command trig. temp meas. hold master
#define HUMID_HUMI_T_H         0xE5 // command trig. humidity meas. hold master
#define HUMID_TEMP_T_NH        0xF3 // command trig. temp meas. no hold master
#define HUMID_HUMI_T_NH        0xF5 // command trig. humidity meas. no hold master

#define HUMID_SOFT_RST         0xFE // command soft reset

#define HUMIDITY               0x00
#define TEMPERATURE            0x01

#define HUMID_WRITE_U_R        0xE6 // command write user register
#define HUMID_READ_U_R         0xE7 // command read user register
  #define HUMID_DEFAULT           0x02  // Disable OTP reload
  #define HUMID_HEATER            0x04  //Enable the on chip heater
  #define HUMID_BATTERY           0x40  //0 = VDD > 2.25V, 1=VDD < 2.25V
  
  #define HUMID_RES1              0x00  //12 bit RH, 14 bit T
  #define HUMID_RES2              0x01  //8  bit RH, 12 bit T
  #define HUMID_RES3              0x80  //10 bit RH, 13 bit T
  #define HUMID_RES4              0x81  //11 bit
  #define HUMID_RES_MASK          0x7E  // Only change bits 0 and 7 (meas. res.)
  #define HUMID_MASK              0x38  // Mask off reserved bits (3,4,5)  
  
/*Resolution | RH typ | RH max | T typ | T max | Units 
    14 bit                         66      85    ms 
    13 bit                         33      43    ms 
    12 Bit      22       29        17      22    ms 
    11 bit      12       15         9      11    ms 
    10 bit       7        9                      ms 
     8 bit       3        4                      ms 
*/

void humid_init(void);
void humid_read_humidity(void);
static bool HumidWriteCmd(uint8 cmd);
static bool HumidReadData(uint8 *pBuf, uint8 nBytes);



/****************************************************************************
  IR Temperature
*****************************************************************************/
/* Slave address */
#define TMP006_I2C_ADDRESS              0x44

/* TMP006 register addresses */
#define TMP006_VOLTAGE         0x00
#define TMP006_TEMPERATURE     0x01
#define TMP006_CONFIG          0x02
#define TMP006_MANF_ID              0xFE
#define TMP006_PROD_ID              0xFE

/* TMP006 register values */
#define TMP006_VAL_CONFIG_RESET         0x7400  // Sensor reset state
#define TMP006_VAL_CONFIG_ON            0x7000  // Sensor on state
#define TMP006_VAL_CONFIG_OFF           0x0000  // Sensor off state
#define TMP006_VAL_MANF_ID              0x5449  // Manufacturer ID
#define TMP006_VAL_PROD_ID              0x0067  // Product ID

/*Config register bitmasks */
#define TMP006_MSB_RESET                0x80
#define TMP006_MSB_POWER_ON             0x70

#define TMP006_MSB_CONV_MASK            0x0E  // Bitmask for conversion
#define TMP006_MSB_CONV_1               0x00  // .25 s
#define TMP006_MSB_CONV_2               0x02  // .50 s
#define TMP006_MSB_CONV_4               0x04  // 1   s
#define TMP006_MSB_CONV_8               0x06  // 2   s
#define TMP006_MSB_CONV_16              0x08  // 4   s

#define TMP006_MSB_DRDY_ENABLE          0x01


/* Bit values */
#define DATA_RDY_BIT                    0x8000 // Data ready

/* Register length */
#define IRTEMP_REG_LEN                  2


void IR_on(void);
void IR_off(void);
void read_IR(uint16 *buffer);
void IR_keepalive(bool keepalive);
bool IR_data_ready(void);