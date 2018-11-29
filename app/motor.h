#ifndef __MOTOR_DEF_H__
#define __MOTOR_DEF_H__

#include "app_common.h"

#define MOTOR_MAX_NUM         6

typedef enum
{
  motor_1 = 0,
  motor_2,
  motor_3,
  motor_4,
  motor_5,
  motor_6,
} motor_ndx_t;

extern void motor_init(void);
extern void motor_set(motor_ndx_t ndx, uint16_t v);

#endif /* !__MOTOR_DEF_H__ */
