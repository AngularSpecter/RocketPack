#include "uart.h"

 unsigned int BAUD_RATE = 96;

void UART_init(void)
{
  /* Set UART0 to alternate 2 (UART) mode */  
  PERCFG |= BV(0);
 
  /*Give the UART priority on the pins */
  //P1.4 = RX
  //P1.5 = TX

  P1DIR |= BV(5);        //TX pin as output
  P1SEL |= BV(4)|BV(5);  //Set pins as special mode
   
  
  /*UART Mode */
  U0CSR |= BV(7);
  
  unsigned char M;
  unsigned char E;
  
  calcBAUD(BAUD_RATE, &M, &E);
  
  /*BAUD rate setting : 9600 ... baud_m = 59  baud_e = 8 for 32 MHz */
  U0GCR |= E;  //BAUD_E
  U0BAUD |= M; //BAUD_M  
  
  //U0GCR |= BV(5);  //Set to MSB first
  
  //U0CSR &= ~BV(2);  //Clear the buffer
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


void flushByte(uint8* txbuffer)
{
    U0DBUF =  (unsigned char)*txbuffer; 
    while(U0CSR & BV(0));                        //wait until tx is finished
}

void send_ACK(void)
{
  U0DBUF = 0xff;
  while(U0CSR & BV(0)); 
}

void send_NACK(void)
{
  U0DBUF = 0x00;
  while(U0CSR & BV(0)); 
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

void uart_send_data(uint16 data)
{
  
    char *output_buffer;
    unsigned char buff_len = 6;
    
    itoa(data, output_buffer, 65535);
    uartSend(output_buffer,buff_len);
       
    U0DBUF = '\r';
    U0DBUF = '\n';   
}

void uart_send_buffer(uint16 *buffer, uint8 len)
{
  
    char *output_buffer;
    unsigned char buff_len = 6;
    
    while(len--)
    {
    itoa(buffer[len+1], output_buffer, 65535);
    uartSend(output_buffer,buff_len);
    
    if (len > 1) U0DBUF = ':';
    }
       
    U0DBUF = '\r';
    U0DBUF = '\n';   
}

void itoa(uint16 val, char *str, uint16 limit)
{
  int temploc = 0;
  int digit = 0;
  int strloc = 0;
  char tempstr[5]; //16-bit number can be at most 5 ASCII digits;
  char *outstr;
  
  if(val>limit)
    val %= limit;
 
  do
  {
    digit = val % 10;
    tempstr[temploc++] = digit + '0';
    val /= 10;
  } while (val > 0);
 
  // reverse the digits back into the output string
  while(temploc>0)
    str[strloc++] = tempstr[--temploc];
 
  str[strloc]=0;
}