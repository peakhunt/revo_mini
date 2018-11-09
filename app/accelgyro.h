#ifndef __ACCELGYRO_DEF_H__
#define __ACCELGYRO_DEF_H__

#include "app_common.h"

#define ACCELGYRO_GYRO_CALIBRATION_TIMEOUT        20

extern void accelgyro_init(void);
extern void accelgyro_start(void);
extern void accelgyro_stop(void);
extern void accelgyro_get(int16_t accel[3], int16_t gyro[3]);
extern uint16_t accelgyro_sample_rate(void);

typedef void (*accelgyro_gyro_calib_callback)(int16_t offset[3], void* cb_arg);
extern bool accelgyro_gyro_calibrate(accelgyro_gyro_calib_callback cb, void* cb_arg);

#endif /* !__ACCELGYRO_DEF_H__ */
