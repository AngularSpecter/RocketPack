#include "sensors.h"
#include "uart.h"

//command format
//Select Dev:   MASTER_SEL, DEVICE ;  MASTER_ACK
//Write reg :   MASTER_WRITE, REGISTER ADDRESS, n_bytes, byte 1, byte 2, nyte n  ; MASTER_ACK
//Read reg  :   MASTER_READ, REGISTER ADDRESS, n_bytes ; byte_1, byte_2, byte_n


/*------------------------------------------------------------------------------
###############################################################################
Generics
------------------------------------------------------------------------------*/

unsigned char select_sensor(unsigned char DEVICE)
{
	uart_putc(MASTER_SEL);
	uart_putc(DEVICE);

	unsigned char reply;
	uart_read(&reply);

	return (reply == MASTER_ACK);
}


unsigned char write_reg(unsigned char address, unsigned char* command)
{
   uart_putc(MASTER_WRITE);
   uart_putc(address);
   uart_putc(1);
   uart_putc(*command);

   unsigned char reply;
   uart_read(&reply);

   return (reply == MASTER_ACK);

}

unsigned char read_reg(unsigned char address, unsigned char* data, unsigned char n_bytes)
{
	uart_putc(MASTER_READ);
	uart_putc(address);
	uart_putc(n_bytes);

	while (n_bytes--)
	{
		uart_read(data++);
	}
}
/*------------------------------------------------------------------------------
###############################################################################
Gyro
------------------------------------------------------------------------------*/
void start_gyro(void)
{

  select_sensor(GYRO_I2C_ADDRESS);

   //turn on all axes and lock clock to x
  unsigned char command = ~GYRO_PWR_MGM_STBY_ALL | GYRO_PWR_MGM_CLOCK_PLL_X | GYRO_PWR_MGM_SLEEP;
  write_reg(GYRO_REG_PWR_MGM, &command);

  command = GYRO_INT_LATCH | GYRO_INT_CLEAR| GYRO_INT_DATA;
  write_reg(GYRO_REG_INT_CFG, &command);

  //set sensor poll rate
  command = 0xff;
  write_reg(GYRO_REG_SMPLRT_DIV, &command);
}



void read_gyro(unsigned int *buffer)
{
  unsigned char lsb, msb;

  select_sensor(GYRO_I2C_ADDRESS);

  read_reg(GYRO_REG_GYRO_XOUT_H,&msb,1);
  read_reg(GYRO_REG_GYRO_XOUT_L,&lsb,1);
  buffer[0] = (msb << 8 ) | (lsb);

  read_reg(GYRO_REG_GYRO_YOUT_H,&msb,1);
  read_reg(GYRO_REG_GYRO_YOUT_L,&lsb,1);
  buffer[1] = (msb << 8 ) | (lsb);

  read_reg(GYRO_REG_GYRO_ZOUT_H,&msb,1);
  read_reg(GYRO_REG_GYRO_ZOUT_L,&lsb,1);
  buffer[2] = (msb << 8 ) | (lsb);
}


/*------------------------------------------------------------------------------
###############################################################################
Accelerometer
------------------------------------------------------------------------------*/
void init_acc(void)
{
   //Select Accelerometer
  select_sensor(ACC_I2C_ADDRESS);

  unsigned char command = 0x00;          //put to sleep
  write_reg(ACC_CTRL_REG1, &command);

    //configure CTRL2 - 6.25Hz
  command = ACC_OWUFB | ACC_OWUFC;
  write_reg(ACC_CTRL_REG2, &command);

  // Pulse the interrupt, active high and enable
  command = ACC_IEL | ACC_IEA | ACC_IEN;
  write_reg(ACC_INT_CTRL_REG1, &command);

  //Enable interrupt for all axes
  command = ACC_ALLWU;
  write_reg(ACC_INT_CTRL_REG2, &command);
}







