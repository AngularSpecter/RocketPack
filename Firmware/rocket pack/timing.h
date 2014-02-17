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
