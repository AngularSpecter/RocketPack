void uart_init(void);

unsigned char uart_status(void);
void uart_putc(unsigned char c);
void uart_puts(char *str);

unsigned char last_RXd(void);
unsigned char pop(void);

unsigned char shift(void);

void uart_read(unsigned char *buffer);
