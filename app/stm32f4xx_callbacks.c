#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "app_common.h"
#include "event_dispatcher.h"
#include "event_list.h"

#include "tim.h"
#include "micros.h"

volatile uint32_t     __uptime  = 0;
volatile uint32_t     __msec    = 0;

void HAL_SYSTICK_Callback(void)
{
  static uint16_t   count = 0;

  __msec++;
  count++;
  if(count >= 1000)
  {
    __uptime++;
    count = 0;
  }

  event_set(1 << DISPATCH_EVENT_TIMER_TICK);
}

void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim7)
  {
    micros_update_callback();
  }
}
