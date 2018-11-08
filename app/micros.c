#include "stm32f4xx_hal.h"
#include "tim.h"
#include "micros.h"

static TIM_HandleTypeDef*         _htim = &htim7;
static volatile uint16_t          _update_cnt;

void
micros_init(void)
{
  _update_cnt = 0;
  HAL_TIM_Base_Start_IT(_htim);
}

uint32_t
micros_get(void)
{
  uint16_t  usec = __HAL_TIM_GET_COUNTER(_htim);
  uint32_t  ret;

  ret = (_update_cnt << 16 ) | usec;
  return ret;
}

void
micros_update_callback(void)
{
  _update_cnt++;
}
