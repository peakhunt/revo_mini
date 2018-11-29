#include "pwm.h"
#include "motor.h"
#include "config.h"

void
motor_init(void)
{
  pwm_start();

  for(int i = 0; i < MOTOR_MAX_NUM; i++)
  {
    motor_set(i, GCFG->motor_min);
  }
}

void
motor_set(motor_ndx_t ndx, uint16_t v)
{
  pwm_set_duty(GCFG->motor_ndx[ndx], v);
}
