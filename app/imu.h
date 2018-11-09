#ifndef __IMU_DEF_H__
#define __IMU_DEF_H__

#include "app_common.h"
#include "madgwick.h"

typedef struct
{
  //
  // AHRS filter
  //
  madgwick_t    filter;
  //
  // orientation
  //
  int16_t   orient[3];    // in degrees * 10
} imu_t;

extern int16_t           attitude[3];

extern void imu_init(void);
extern imu_t* imu_get(void);

#endif /* !__IMU_DEF_H__ */
