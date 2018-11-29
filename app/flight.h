#ifndef __FLIGHT_DEF_H__
#define __FLIGHT_DEF_H__

#include "app_common.h"

typedef enum
{
  flight_state_disarmed,
  flight_state_arming,
  flight_state_armed,
  flight_state_disarming,
} flight_state_t;

extern void flight_init(void);
extern void flight_arm(void);
extern void flight_disarm(void);

extern float pid_out[3];
extern float pid_target[3];
extern uint16_t pid_motor[4];
extern flight_state_t flight_state;

#endif /* !__FLIGHT_DEF_H__ */
