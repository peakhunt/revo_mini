#include "pid.h"

void
pid_control_init(pid_control_t* pidc)
{
  pid_control_reset(pidc);
}

void
pid_control_reset(pid_control_t* pidc)
{
  pidc->prev_error      = 0.0f;
  pidc->integral        = 0.0f;
}

float
pid_control_run(pid_control_t* pidc, const float target, const float feed, const float dt, const float k[3])
{
  float error = target - feed;
  float p,
        i,
        d;

  p = k[0] * error;

  pidc->integral = pidc->integral + error * dt;
  i = k[1] * pidc->integral;

  d = k[2] * (error - pidc->prev_error);
  pidc->prev_error = error;

  return p + i + d;
}
