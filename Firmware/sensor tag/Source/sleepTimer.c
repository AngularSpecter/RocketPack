#include "sleepTimer.h"

 uint32 sleepTimer;


void SetSleepTimer(uint32 timeout)
{
   SLEEP_TIMER_DISABLE();

  uint32 sleepTimer = SleepReadTimer();

  // compute sleep timer compare value
  sleepTimer += timeout;

  // subtract the processing time spent in function halSleep()
  //sleepTimer -= SLEEP_ADJ_TICKS;
 
  // set sleep timer compare; ST0 must be written last
  ST2 = ((uint8 *)&sleepTimer)[UINT32_NDX2];
  ST1 = ((uint8 *)&sleepTimer)[UINT32_NDX1];
  ST0 = ((uint8 *)&sleepTimer)[UINT32_NDX0];

  SLEEP_TIMER_CLEAR();
  SLEEP_TIMER_ENABLE();

  return;
}

uint32 SleepReadTimer( void )
{
  // read the sleep timer
  // Note: Read of ST0 latches ST1 and ST2.
  ((uint8 *)&sleepTimer)[UINT32_NDX0] = ST0;
  ((uint8 *)&sleepTimer)[UINT32_NDX1] = ST1;
  ((uint8 *)&sleepTimer)[UINT32_NDX2] = ST2;
  ((uint8 *)&sleepTimer)[UINT32_NDX3] = 0;

  return( sleepTimer );
}