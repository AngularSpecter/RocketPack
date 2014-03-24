#ifndef TIMING_h
#define TIMING_h

#include <msp430FR5739.h>
#include "types.h"

//redefine macros to make them readable

#define CNTL_16   CNTL_0    /* Counter length: 16 bit */
#define CNTL_12   CNTL_1    /* Counter length: 12 bit */
#define CNTL_10   CNTL_2    /* Counter length: 10 bit */
#define CNTL_8    CNTL_3    /* Counter length:  8 bit */

#define TBSSEL_TBCLK     TBSSEL_0    /* Clock Source: TBCLK */
#define TBSSEL_ACLK      TBSSEL_1    /* Clock Source: ACLK  */
#define TBSSEL_SMCLK     TBSSEL_2    /* Clock Source: SMCLK */
#define TBSSEL_INCLK     TBSSEL_3    /* Clock Source: INCLK */

inline void init_delay(void);
void delay(uint16 ticks);
void delay_cancel(void);



/*** Watchdog timer heartbeat ************************/
extern unsigned int heartbeat_cnt;

#define WDTINT_181216    0x00
#define WDTINT_010816    0x01
#define WDTINT_000416    0x02
#define WDTINT_000016    0x03
#define WDTINT_1s        0x04
#define WDTINT_250ms     0x05
#define WDTINT_15.625ms  0x06
#define WDTINT_1.95ms    0x07

void start_heartbeat(void);
void stop_heartbeat(void);
void wdt_force_puc(void);

#endif
