

/*
 * delay.cpp
 *
 *  Created on: 18.08.2017
 *      Author: Maximilian
 */


#include "em_device.h"
#include "em_timer.h"
#include "em_cmu.h"

#define RTC_FREQ 32768
#define TIMER_FREQ 48000000

/**********************************************************
 * Enables clocks used for delay functions.
 **********************************************************/
void initDelay(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}


/**********************************************************
 * Delay a number of milliseconds
 **********************************************************/
void delay(int ms)
{
  uint32_t endValue = ms * RTC_FREQ / 1000;
  RTC->CNT = 0;

  RTC->CTRL |= RTC_CTRL_EN;

  while ( RTC->CNT < endValue );

  RTC->CTRL &= ~RTC_CTRL_EN;
}

/**********************************************************
 * Delay a number of microseconds
 **********************************************************/
void delayUs(int us)
{
  uint32_t endValue = us * (TIMER_FREQ / 1000000);
  TIMER0->CNT = 0;

  TIMER0->CMD = TIMER_CMD_START;

  while ( TIMER0->CNT < endValue );

  TIMER0->CMD = TIMER_CMD_STOP;
}

