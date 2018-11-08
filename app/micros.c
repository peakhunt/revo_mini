#include "stm32f4xx_hal.h"
#include "tim.h"
#include "micros.h"

static TIM_HandleTypeDef*         _htim = &htim7;
static volatile uint16_t          _mil_sec;

void
micros_init(void)
{
  _mil_sec = 0;
  HAL_TIM_Base_Start_IT(_htim);
}

uint32_t
micros_get(void)
{
  uint32_t  usec = __HAL_TIM_GET_COUNTER(_htim);
  uint32_t  ret;

  ret = _mil_sec * 1000 + usec;
  return ret;
}

void
micros_1ms_callback(void)
{
  _mil_sec++;
}
