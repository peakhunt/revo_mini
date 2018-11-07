#ifndef __PWM_DEF_H__
#define __PWM_DEF_H__

#include "app_common.h"

#define PWM_OUT_MIN_DUTY_CYCLE        800
#define PWM_OUT_MAX_DUTY_CYCLE        2000

typedef enum
{
  pwm_channel_0 = 0,
  pwm_channel_1,
  pwm_channel_2,
  pwm_channel_3,
  pwm_channel_4,
  pwm_channel_5,
} pwm_channel_enum_t;

extern void pwm_start(void);
extern void pwm_stop(void);
extern void pwm_set_duty(pwm_channel_enum_t chnl, uint16_t duty);

#endif /* !__PWM_DEF_H__ */
