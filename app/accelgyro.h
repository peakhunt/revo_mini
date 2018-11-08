#ifndef __ACCELGYRO_DEF_H__
#define __ACCELGYRO_DEF_H__

#include "app_common.h"

extern void accelgyro_init(void);
extern void accelgyro_start(void);
extern void accelgyro_stop(void);
extern void accelgyro_get(int16_t accel[3], int16_t gyro[3]);
extern uint16_t accelgyro_sample_rate(void);

#endif /* !__ACCELGYRO_DEF_H__ */
