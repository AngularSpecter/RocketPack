#include "sensors.h"

void sensor_int_init(void)
{  
    //Gyro Interrupt  P0.1
   // P0SEL &= ~(BV(1));    /* Set pin function to GPIO */
   // P0DIR &= ~(BV(1));    /* Set pin direction to Input */
    //P0IEN |=   BV(1);     /* enable interrupt generation at port */
    
    //Accelerometer Interrupt  P0.2
    //P0SEL &= ~(BV(2));    /* Set pin function to GPIO */
    //P0DIR &= ~(BV(2));    /* Set pin direction to Input */
    //P0IEN |=   BV(2);     /* enable interrupt generation at port */
    
    //Magnetometer Interrupt  P0.6
   // P0SEL &= ~(BV(6));    /* Set pin function to GPIO */
   // P0DIR &= ~(BV(6));    /* Set pin direction to Input */
    //P0IEN |=   BV(6);     /* enable interrupt generation at port */

    //IR Temp Interrupt  P0.3
   // P0SEL &= ~(BV(3));    /* Set pin function to GPIO */
   // P0DIR &= ~(BV(3));    /* Set pin direction to Input */
    //P0IEN |=   BV(3);     /* enable interrupt generation at port */
    
    P0INP |= BV(1) | BV(2) | BV(6) | BV(3);
    //P2INP |= BV(5);    //enable pull down resistors on P0
    
    /* Clear any pending interrupts */
    //P0IFG = ~(BV(1));  //gyro 
   // P0IFG = ~(BV(2));  //accelerometer
    //P0IFG = ~(BV(6));  //magnetometer
    //P0IFG = ~(BV(3));  //IR Therm
    
    P0IFG = 0;
    P0IF  = 0;
    
    IEN1  |= BV(5);    /* enable CPU interrupt for all of port 0*/ 
}


void start_interrupts(uint8 mask)
{
   /* Clear any pending interrupts */
    P0IFG = ~(mask);   
    P0IEN |= mask;
    
    EA = 1;  //make sure interrupts are on
}

void stop_interrupts(uint8 mask)
{
    P0IEN &= ~mask;
}

void sensors_init(void)
{
  //Gyro is powered through P1.1
  P1DIR |= BV(1);
  GYRO_OFF();    //start off
  
  //Make sure the DCDC converter is on
  P0_7 = 1;
}

void burstRead(uint8 startADD, uint8 n_buffers)
{
  uint8 byte;
  uint8 checksum = 0x80;
  
  while(n_buffers--)
  {
    HalSensorReadReg(startADD++,&byte,1);
    checksum |= (byte == 0) << n_buffers;
    flush_data(&byte);
  }
  flush_byte(&checksum);
}
/*------------------------------------------------------------------------------
###############################################################################
Gyro
------------------------------------------------------------------------------*/
void start_gyro(void)
{
  GYRO_ON();
  
  HalI2CInit(GYRO_I2C_ADDRESS,   i2cClock_533KHZ);   //select the gyro
  
   //turn on all axes and lock clock to y
  //uint8 command = ~GYRO_PWR_MGM_STBY_ALL | GYRO_PWR_MGM_CLOCK_INT_OSC | GYRO_PWR_MGM_SLEEP;
  uint8 command = GYRO_PWR_MGM_CLOCK_INT_OSC;
  HalSensorWriteReg(GYRO_REG_PWR_MGM, &command,1);
  
  command = GYRO_INT_LATCH | GYRO_INT_CLEAR| GYRO_INT_DATA;
  HalSensorWriteReg(GYRO_REG_INT_CFG, &command, 1);
  
  
  //set sensor poll rate
  //Fsample = Finternal / (divider+1)
  command = 0 ;         //8 ms sample time
  HalSensorWriteReg(GYRO_REG_SMPLRT_DIV, &command, 1);
  
  //Finternal = 1kHz; Range = +-1000 s^-1; 188Hz LPF
  command = GYRO_DLPF_256 | GYRO_FS_2000;       
  HalSensorWriteReg(GYRO_REG_DLPF_FS, &command, 1);
}

void read_gyro()
{ 
  HalI2CInit(GYRO_I2C_ADDRESS,   i2cClock_533KHZ);   //select the gyro
  burstRead(GYRO_REG_GYRO_XOUT_H, 6);
}
  
void read_gyro_temp(uint16 *buffer)
{
  uint8 lsb, msb;
  
  HalI2CInit(GYRO_I2C_ADDRESS,   i2cClock_533KHZ);   //select the gyro
  
  HalSensorReadReg(GYRO_REG_TEMP_OUT_H,&msb,1);
  HalSensorReadReg(GYRO_REG_TEMP_OUT_L,&lsb,1);
 *buffer = (msb << 8 ) | (lsb);
}

uint8 gyro_int_status(void)
{
  uint8 status;
  HalI2CInit(GYRO_I2C_ADDRESS,   i2cClock_533KHZ);
  HalSensorReadReg(GYRO_REG_INT_STATUS ,&status,1);
  return status;
}

void gyro_sleep(bool sleep)
{
  uint8 power_mode;
  HalI2CInit(GYRO_I2C_ADDRESS,   i2cClock_533KHZ);
  HalSensorReadReg(GYRO_REG_PWR_MGM ,&power_mode,1);
  
  power_mode = (sleep)? power_mode | GYRO_PWR_MGM_SLEEP : power_mode & ~GYRO_PWR_MGM_SLEEP;
  HalSensorWriteReg(GYRO_REG_PWR_MGM , &power_mode, 1);
}

/****************************************************************************
 Accelerometer
*****************************************************************************/
void init_acc(void)
{
   //Select Accelerometer
  HalI2CInit(ACC_I2C_ADDRESS, i2cClock_267KHZ);
    
  uint8 command = 0x00;          //put to sleep
  HalSensorWriteReg(ACC_CTRL_REG1, &command, 1);
    
  //configure CTRL2 - 6.25Hz
  command = ACC_OWUFB | ACC_OWUFC;
  HalSensorWriteReg(ACC_CTRL_REG2, &command, 1);
  
  // Pulse the interrupt, active high and enable
  command = ACC_IEL | ACC_IEA | ACC_IEN;
  HalSensorWriteReg(ACC_INT_CTRL_REG1, &command, 1);
   
  //Enable interrupt for all axes
  command = ACC_ALLWU;
  HalSensorWriteReg(ACC_INT_CTRL_REG2, &command, 1);
  
  //Set the resolution to 8G and power on
  command = ACC_GSEL0 | ACC_GSEL1 | ACC_RES | ACC_PC1 | ACC_DRDYE; 
  HalSensorWriteReg(ACC_CTRL_REG1, &command, 1);
}


// Enable/disable interrupts
void acc_int_mode(bool interrupt)
{
  uint8 int_mode;
  HalI2CInit(ACC_I2C_ADDRESS, i2cClock_267KHZ);
  HalSensorReadReg(ACC_CTRL_REG1,&int_mode,1);
  
  int_mode = (interrupt)? int_mode | ACC_DRDYE : int_mode & ~ACC_DRDYE;
  HalSensorWriteReg(ACC_CTRL_REG1 , &int_mode, 1);
} 

// Sleep/wakeup sensor
void acc_sleep(bool sleep)
{
  uint8 power_mode;
  HalI2CInit(ACC_I2C_ADDRESS, i2cClock_267KHZ);
  HalSensorReadReg(ACC_CTRL_REG1 ,&power_mode,1);
  
  power_mode = (~sleep)? power_mode | ACC_PC1 : power_mode & ~ACC_PC1;
  HalSensorWriteReg(ACC_CTRL_REG1 , &power_mode, 1);
}

void read_acc(void)
{ 

  HalI2CInit(ACC_I2C_ADDRESS, i2cClock_267KHZ);
   burstRead(ACC_XOUT_L, 6);
}

uint8 acc_int_status(void)
{
  uint8 status;
  HalI2CInit(ACC_I2C_ADDRESS, i2cClock_267KHZ);
  HalSensorReadReg(ACC_INT_SOURCE1 ,&status,1);
  return (status & ACC_INT_DRDY);
}

/*------------------------------------------------------------------------------
###############################################################################
Magnetometer
------------------------------------------------------------------------------*/
void start_mag(void)
{
  //Select Magnetometer
  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
    
  uint8 command = MAG_AUTO_MRST;          //enable resets
  HalSensorWriteReg(MAG_CTRL_2, &command, 1);
  
  //command = MAG_CTRL1_TM | MAG_640_5HZ| MAG_CTRL1_AC;  //Triggered updates at 5Hz
  command = MAG_1280_40HZ | MAG_CTRL1_TM | MAG_CTRL1_AC;
  HalSensorWriteReg(MAG_CTRL_1, &command, 1);
}

void read_mag()
{ 
 // uint8 byte;

  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
  
 // byte = HAL_MAG3110_I2C_ADDRESS;
  //flush_data(&byte);
  
  burstRead(MAG_X_MSB, 6);
}

void mag_sleep(bool sleep)
{
  uint8 power_mode;
  
  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
  
  HalSensorReadReg(MAG_CTRL_1,&power_mode,1);  //read old state
  
  power_mode = (~sleep)? power_mode | MAG_CTRL1_AC : power_mode & ~MAG_CTRL1_AC;
  HalSensorWriteReg(MAG_CTRL_1, &power_mode, 1);
  
  
}

void zero_mag(void)
{
  uint8 val;
  uint8 oldState;
  
  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
  
  HalSensorReadReg(MAG_CTRL_1,&val,1);  //read old state
  oldState = val;                       //preserve old state
  val &= ~MAG_CTRL1_AC;                 //Turn off polling
  HalSensorWriteReg(MAG_OFF_X_LSB, &val, 1);
  
  uint8 idx = 6;
  
  uint8 readREG  = MAG_X_MSB;
  uint8 writeREG = MAG_OFF_X_MSB;
  
  while(idx--)
  {
    HalSensorReadReg(readREG++,&val,1);
    HalSensorWriteReg(writeREG++, &val, 1);
  }
  
  HalSensorWriteReg(MAG_OFF_X_LSB, &oldState, 1); //restore the old state
  
  //Make sure that offsets are being applied
  HalSensorReadReg(MAG_CTRL_2,&oldState,1);
  oldState &= MAG_RAW;
  HalSensorWriteReg(MAG_CTRL_2, &oldState, 1);
}

uint8 mag_status(void)
{
  uint8 status;
  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
  HalSensorReadReg(MAG_DR_STATUS,&status,1);
  
  return status;
}

/****************************************************************************
 Barometer
*****************************************************************************/
void start_baro(void)
{
  //Select Barometer
  HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ);
  
  uint8 command = BARO_OFF_COMMAND;
  HalSensorWriteReg(BARO_COMMAND_CTRL, &command, sizeof(command));
}


uint16 baro_capture_press(uint8 resolution)
{
    HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ); 
    
    uint8 command = BARO_PRESS_READ_COMMAND;
    uint16 delay;
    
        switch(resolution)
        {
        case 2  : command |= BARO_RES_2;  delay = 64;  break;
        case 8  : command |= BARO_RES_8;  delay = 256; break;
        case 16 : command |= BARO_RES_16; delay = 512; break;
        case 64 : command |= BARO_RES_64; delay = 2048; break;
        default : command |= BARO_RES_2;  delay = 64; break;
        }
        
    HalSensorWriteReg(BARO_COMMAND_CTRL, &command, 1);
    return delay;
}

uint8 baro_press[2] = {0,0};

void baro_read_press(bool read)
{
   HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ);
   
   uint8 checksum = 0x80;
  
   if (read ){
   HalSensorReadReg(BARO_PRESS_LSB,&baro_press[0],1);
   HalSensorReadReg(BARO_PRESS_MSB,&baro_press[1],1);
   }
   
   checksum |= (baro_press[0] == 0);
   checksum |= (baro_press[1] == 0) << 1;
   
   flush_data(&baro_press[0]);
   flush_data(&baro_press[1]);
   flush_byte(&checksum);
   
}


uint16 baro_capture_temp(void)
{
    HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ); 
    
    uint8 command = BARO_TEMP_READ_COMMAND | BARO_RES_16;      
    HalSensorWriteReg(BARO_COMMAND_CTRL, &command, 1);
    return (uint16) 0xFA00;
}

uint8 baro_temp[2] = {0,0};

void baro_read_temp(bool read)
{
   HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ); 
   
   uint8 checksum = 0x80;
  
   if( read ){
   HalSensorReadReg(BARO_TEMP_LSB,&baro_temp[0],1);
   HalSensorReadReg(BARO_TEMP_MSB,&baro_temp[1],1);
   }
   
   checksum |= (baro_temp[0] == 0);
   checksum |= (baro_temp[1] == 0) << 1;
   
   flush_data(&baro_temp[0]);
   flush_data(&baro_temp[1]);
   flush_byte(&checksum);
}


void baro_read_cal(void)
{
  HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ);
  burstRead(BARO_CALIBRATION_1_LSB, 16); 
}


void baro_shutdown(void)
{
   HalI2CInit(BARO_I2C_ADDRESS, i2cClock_267KHZ); 
   uint8 command = BARO_OFF_COMMAND;
   HalSensorWriteReg(BARO_COMMAND_CTRL, &command, 1);
}


/****************************************************************************
  Humidity
*****************************************************************************/
#define HUMID_RES_DEFAULT      HUMID_RES1
 
uint8 humid_resolution = HUMID_RES_DEFAULT;
static uint8 humid_usr;

void humid_init(void)
{
  HalI2CInit(HUMID_I2C_ADDRESS,i2cClock_267KHZ);

  // Set 11 bit resolution
  HalSensorReadReg(HUMID_READ_U_R,&humid_usr,1);
  humid_usr &= HUMID_RES_MASK;  //zero out resolution bits
  humid_usr |= humid_resolution;
  HalSensorWriteReg(HUMID_WRITE_U_R,&humid_usr,1);
}

 uint8 humid_buffer[3] = {0,0,0};

void humid_read_humidity(uint8 read)
{
  HalI2CInit(HUMID_I2C_ADDRESS,i2cClock_267KHZ);
  
  //uint8 byte = HUMID_I2C_ADDRESS;
  //flush_data(&byte);
  
  //HumidWriteCmd(HUMID_TEMP_T_H);
  //HumidReadData(buffer, DATA_LEN);
  
  //buffer[1] &= ~0x03;
  //flush_data(&buffer[0]);
  //flush_data(&buffer[1]);
  
  if (read)
  {
    HumidWriteCmd(HUMID_HUMI_T_H);
    HumidReadData(&humid_buffer[0], DATA_LEN);
    
    //clear last 2 lsb bits and transmit MSB first
    humid_buffer[1] &= ~0x03;
  }
  
  
  flush_data(&humid_buffer[0]);
  flush_data(&humid_buffer[1]);
  
  //compute checksum
  humid_buffer[2] = (humid_buffer[2] == 0) | (humid_buffer[2] == 0) << 1 | 0x80;
  
  flush_data(&humid_buffer[2]);
  
}


static bool HumidWriteCmd(uint8 cmd)
{
  /* Send command */
  return HalI2CWrite(1,&cmd) == 1;
}

static bool HumidReadData(uint8 *pBuf, uint8 nBytes)
{
  /* Read data */
  return HalI2CRead(nBytes,pBuf ) == nBytes;
}

/****************************************************************************
  IR Temperature
*****************************************************************************/
uint8 IR_CONVERSION_TIME = TMP006_MSB_CONV_1;
bool  IR_live = FALSE;

void IR_on(void)
{
  uint8 command[2] = {IR_CONVERSION_TIME  | 
                      TMP006_MSB_POWER_ON | 
                      TMP006_MSB_DRDY_ENABLE, 0x00};
  
  HalI2CInit(TMP006_I2C_ADDRESS, i2cClock_533KHZ);
  
  HalSensorWriteReg(TMP006_CONFIG, command, IRTEMP_REG_LEN);
  
}

void IR_off(void)
{
  uint8 command[2] = {IR_CONVERSION_TIME   & 
                      ~TMP006_MSB_POWER_ON & 
                      ~TMP006_MSB_DRDY_ENABLE, 0x00};
  
  HalI2CInit(TMP006_I2C_ADDRESS, i2cClock_533KHZ);
  
  HalSensorWriteReg(TMP006_CONFIG, command, IRTEMP_REG_LEN);
  
}

void read_IR(uint16 *buffer)
{
  uint16 voltage = 0x0000;
  uint16 temp    = 0x0000;
  bool   success;
  
  HalI2CInit(TMP006_I2C_ADDRESS, i2cClock_533KHZ);
  
  // Read the sensor registers
  success = HalSensorReadReg(TMP006_VOLTAGE, (uint8 *)&voltage, IRTEMP_REG_LEN );
  if (success)
  {
    success = HalSensorReadReg(TMP006_TEMPERATURE, (uint8 *)&temp,IRTEMP_REG_LEN );
  }
  
  buffer[0] = voltage;
  buffer[1] = temp;
  
  //if (!IR_live) IR_off();
  
}

bool IR_data_ready(void)
{
    uint16 v;

    // Select this sensor on the I2C bus
    HalI2CInit(TMP006_I2C_ADDRESS, i2cClock_533KHZ);

    // Read the data ready bit
    HalSensorReadReg(TMP006_CONFIG, (uint8 *)&v,IRTEMP_REG_LEN );
    if (v & DATA_RDY_BIT) return 1;
    return 0;
}

void IR_keepalive(bool keepalive)
{
  IR_live = keepalive;
}











