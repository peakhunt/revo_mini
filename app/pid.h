#ifndef __PIF_DEF_H__
#define __PIF_DEF_H__

#include "app_common.h"

typedef struct
{
  float       prev_error;
  float       integral;
} pid_control_t;

extern void pid_control_init(pid_control_t* pidc);
extern void pid_control_reset(pid_control_t* pidc);
extern float pid_control_run(pid_control_t* pidc, const float input, const float target, const float dt, const float k[3]);

#endif /* !__PIF_DEF_H__ */
