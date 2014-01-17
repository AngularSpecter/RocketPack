#include <ioCC2541.h>
#include <hal_defs.h>
#include <hal_board_cfg.h>

#define SLEEP_TIMER_ENABLE()        st(IEN0 |= BV(5);)
#define SLEEP_TIMER_DISABLE()       st(IEN0 &= ~BV(5);) 
#define SLEEP_TIMER_CLEAR()         st(IRCON &= ~0x80;) 

#define SLEEP_ADJ_TICKS              25

#define UINT32_NDX0                         0
#define UINT32_NDX1                         1
#define UINT32_NDX2                         2
#define UINT32_NDX3                         3

void SetSleepTimer(uint32 timeout );
uint32 SleepReadTimer( void );