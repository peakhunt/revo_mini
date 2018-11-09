#ifndef __SENSOR_ALIGN_DEF_H__
#define __SENSOR_ALIGN_DEF_H__

#include "app_common.h"

typedef enum
{
  sensor_align_cw_0,
  sensor_align_cw_90,
  sensor_align_cw_180,
  sensor_align_cw_270,
  sensor_align_cw_0_flip,
  sensor_align_cw_90_flip,
  sensor_align_cw_180_flip,
  sensor_align_cw_270_flip
} sensor_align_t;

static inline void
sensor_align_values(int16_t values[3], sensor_align_t align)
{
    const int16_t x = values[0],
                  y = values[1],
                  z = values[2];

  switch(align)
  {
  case sensor_align_cw_0:
    values[0] = x;
    values[1] = y;
    values[2] = z;
    break;

  case sensor_align_cw_90:
    values[0] = y;
    values[1] = -x;
    values[2] = z;
    break;

  case sensor_align_cw_180:
    values[0] = -x;
    values[1] = -y;
    values[2] = z;
    break;

  case sensor_align_cw_270:
    values[0] = -y;
    values[1] = x;
    values[2] = z;
    break;

  case sensor_align_cw_0_flip:
    values[0] = -x;
    values[1] = y;
    values[2] = -z;
    break;

  case sensor_align_cw_90_flip:
    values[0] = y;
    values[1] = x;
    values[2] = -z;
    break;

  case sensor_align_cw_180_flip:
    values[0] = x;
    values[1] = -y;
    values[2] = -z;
    break;

  case sensor_align_cw_270_flip:
    values[0] = -y;
    values[1] = -x;
    values[2] = -z;
    break;
  }
}

#endif /* !__SENSOR_ALIGN_DEF_H__ */
