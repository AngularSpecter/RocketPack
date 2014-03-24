#ifndef STAG_h
#define STAG_h

#include <msp430FR5739.h>
#include "types.h"

#define KINEMAT	  0x01
#define MAGNET	  0x02
#define BARO	  0x04
#define HUMID     0x08
#define CALDATA	  0x10
#define AUTOPOLL  0x20


void init_sensorTag(void);
void sensorTag_reset(char mode);



#endif
