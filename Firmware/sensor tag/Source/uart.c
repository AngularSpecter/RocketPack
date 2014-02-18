#include "uart.h"

 unsigned int BAUD_RATE = 96;

void UART_init(void)
{
  /* Set UART0 to alternate (UART) mode */  
  PERCFG |= BV(0);
  
  /*Give the UART priority on the pins */
  P1DIR |= BV(5);
  P1SEL |= BV(4)|BV(5);
   
  /*UART Mode */
  U0CSR |= BV(7);
  
  unsigned char M;
  unsigned char E;
  
  calcBAUD(BAUD_RATE, &M, &E);
  
  /*BAUD rate setting : 9600 ... baud_m = 59  baud_e = 8 for 32 MHz */
  U0GCR |= E;  //BAUD_E
  U0BAUD |= M; //BAUD_M  
  
  //U0GCR |= BV(5);
  
  U0UCR |= BV(7);   //Flush the UART
  
  ENABLE_RX();   //Start the receiver
  
  IEN0 |= BV(2);    //Enable interrupt
}


void calcBAUD(unsigned int baud, unsigned char *M, unsigned char *E)
{
  switch (baud)
  {
  case 24   :  *M = 59;   *E = 6;   break;
  case 48   :  *M = 59;   *E = 7;   break;
  case 96   :  *M = 59;   *E = 8;   break;
  case 144  :  *M = 216;  *E = 8;   break;
  case 192  :  *M = 59;   *E = 9;   break;
  case 288  :  *M = 216;  *E = 9;   break;
  case 384  :  *M = 59;   *E = 10;  break;
  case 576  :  *M = 216;  *E = 10;  break;
  case 768  :  *M = 59;   *E = 11;  break;
  case 1152 :  *M = 216;  *E = 11;  break;
  case 2304 :  *M = 216;  *E = 12;  break;
  default     :  *M = 59;   *E = 8;   break;   //default to 9600
  }
}
    
void sendByte(uint8* txbuffer)
{
    U0DBUF =  (unsigned char)*txbuffer; 
}

void repeatBytes(uint8 byte, uint8 count)
{
  while(count--) flush_byte(&byte);
}

void flush_data(uint8* txbuffer)
{
   unsigned char buffer = (unsigned char)*txbuffer;
   buffer = (buffer == 0) ? 0x01 : buffer;
   U0DBUF =  buffer; 
    while(U0CSR & BV(0));                        //wait until tx is finished
}

void flush_byte(uint8* txbuffer)
{
    U0DBUF =  (unsigned char)*txbuffer; 
    while(U0CSR & BV(0));                        //wait until tx is finished
}

void uartSend(char *pucData, unsigned char ucLength) 
{
  while(ucLength--)
  {
    // Wait for TX buffer to be ready for new data
    while(U0CSR & BV(0));  

    // Push data to TX buffer
    U0DBUF = *pucData;

    // Update variables
    //ucLength--;
    pucData++;
  }

  // Wait until the last byte is completely sent
  while(U0CSR & BV(0));  
}

void uart_send_data(uint16 *buffer, uint8 len)
{
  
    char *output_buffer;
    unsigned char buff_len = 6;
    uint8 i;
    
    while(len--)
    {
    itoa(buffer[len+1], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    
    if (len > 1) U0DBUF = ':';
    }
       
    U0DBUF = '\r';
    U0DBUF = '\n';   
}
