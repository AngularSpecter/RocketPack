#include <ioCC2541.h>
#include <hal_defs.h>
#include <hal_board_cfg.h>

#define ENABLE_RX()       st( U0CSR |= BV(6););
#define DISABLE_RX()      st( U0CSR &= ~BV(6););


void UART_init(void);
void calcBAUD(unsigned int baud, unsigned char *M, unsigned char *E);
void flushByte(uint8* txbuffer);
void sendByte(uint8* txbuffer);
void uartSend( char *pucData, unsigned char ucLength);
void uart_send_data(uint16 *buffer, uint8 len);
void itoa(uint16 val, char *str, uint16 limit);