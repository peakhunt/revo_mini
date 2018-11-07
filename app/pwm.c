#include "stm32f4xx_hal.h"
#include "tim.h"
#include "pwm.h"

////////////////////////////////////////////////////////////////////////////////
//
// module private definitions
//
////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  TIM_HandleTypeDef*  htim;
  uint32_t            tim_chnl;
} pwm_channel_t;

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

static TIM_HandleTypeDef* _pwm_timers[] = 
{
  &htim2,
  &htim3,
  &htim5,
  &htim9,
};

static pwm_channel_t    _pwm_chnls[] = 
{
  {   &htim3,   TIM_CHANNEL_3   },
  {   &htim3,   TIM_CHANNEL_4   },
  {   &htim9,   TIM_CHANNEL_2   },
  {   &htim2,   TIM_CHANNEL_3   },
  {   &htim5,   TIM_CHANNEL_2   },
  {   &htim5,   TIM_CHANNEL_1   },
};

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
pwm_start(void)
{
  for(int i = 0; i < sizeof(_pwm_chnls)/sizeof(pwm_channel_t); i++)
  {
    __HAL_TIM_SET_COMPARE(_pwm_chnls[i].htim, _pwm_chnls[i].tim_chnl, PWM_OUT_MIN_DUTY_CYCLE);
  }

  for(int i = 0; i < sizeof(_pwm_chnls)/sizeof(pwm_channel_t); i++)
  {
    HAL_TIM_PWM_Start(_pwm_chnls[i].htim, _pwm_chnls[i].tim_chnl);
  }

  for(int i = 0; i < sizeof(_pwm_timers)/sizeof(TIM_HandleTypeDef*); i++)
  {
    __HAL_TIM_SET_COUNTER(_pwm_timers[i], 0);
    HAL_TIM_Base_Start(_pwm_timers[i]);
  }
}

void
pwm_stop(void)
{
  for(int i = 0; i < sizeof(_pwm_chnls)/sizeof(pwm_channel_t); i++)
  {
    HAL_TIM_PWM_Stop(_pwm_chnls[i].htim, _pwm_chnls[i].tim_chnl);
  }

  for(int i = 0; i < sizeof(_pwm_timers)/sizeof(TIM_HandleTypeDef*); i++)
  {
    HAL_TIM_Base_Stop(_pwm_timers[i]);
  }
}

void
pwm_set_duty(pwm_channel_enum_t chnl, uint16_t duty)
{
  pwm_channel_t*  c = &_pwm_chnls[chnl];

  __HAL_TIM_SET_COMPARE(c->htim, c->tim_chnl, duty);
}
