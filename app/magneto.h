#ifndef __MAGNETO_DEF_H__
#define __MAGNETO_DEF_H__

#include "app_common.h"
#include "sensor_align.h"

extern int16_t mag_raw[3];
extern int16_t mag_value[3];
extern int16_t mag_offset[3];

extern void magneto_init(sensor_align_t align);
extern void magneto_start(void);
extern void magneto_stop(void);

typedef void (*magneto_calibrate_callback)(int16_t offset[3], void* cb_arg);
extern bool magneto_calibrate(magneto_calibrate_callback cb, void* cb_arg);

#endif /* __MAGNETO_DEF_H__ */
