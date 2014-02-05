#include <msp430FR5739.h>
#include "types.h"

#define STATUS  BIT4
#define SLEEP   BIT5
#define CONFIG  BIT6
#define RESET	BIT7

void config_radio(void);
void sleep_radio(char mode);
