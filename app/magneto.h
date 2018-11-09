#ifndef __MAGNETO_DEF_H__
#define __MAGNETO_DEF_H__

#include "app_common.h"

#define MAGNETO_CALIBRATION_TIMEOUT   60

extern void magneto_init(void);
extern void magneto_start(void);
extern void magneto_stop(void);
extern void magneto_get(int16_t mag[3]);

typedef void (*magneto_calibrate_callback)(int16_t offset[3], void* cb_arg);
extern bool magneto_calibrate(magneto_calibrate_callback cb, void* cb_arg);

#endif /* __MAGNETO_DEF_H__ */
