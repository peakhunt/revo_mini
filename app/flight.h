#ifndef __FLIGHT_DEF_H__
#define __FLIGHT_DEF_H__

#include "app_common.h"

typedef enum
{
  flight_state_disarmed,
  flight_state_arming,
  flight_state_armed,
} flight_state_t;

extern void flight_init(void);

#endif /* !__FLIGHT_DEF_H__ */
