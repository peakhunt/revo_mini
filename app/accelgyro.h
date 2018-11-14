#ifndef __ACCELGYRO_DEF_H__
#define __ACCELGYRO_DEF_H__

#include "app_common.h"
#include "sensor_align.h"

//
// accelerometer range is +- 8G. 
// gyro range is +- 2000 degrees per sec
//
#define ACCELGYRO_ACCEL_LSB       (1.0f / 4096.f)
#define ACCELGYRO_GYRO_LSB        (1.0f / 32.8f)

//
// accelerometer is seto to +- 8G. 
// so 1G is 4096
//
#define ACCELGYRO_1G_VALUE                            4096

extern int16_t accel_raw[3];
extern int16_t accel_value[3];
extern int16_t gyro_raw[3];
extern int16_t gyro_value[3];
extern float gyro_dps[3];

extern void accelgyro_init(sensor_align_t aalign, sensor_align_t galign);
extern void accelgyro_start(void);
extern void accelgyro_stop(void);
extern uint16_t accelgyro_sample_rate(void);

typedef void (*accelgyro_gyro_calib_callback)(int16_t offset[3], void* cb_arg);
extern bool accelgyro_gyro_calibrate(accelgyro_gyro_calib_callback cb, void* cb_arg);

typedef void (*accelgyro_accel_calib_step_callback)(int ndx, void* cb_arg);
typedef void (*accelgyro_accel_calib_done_callback)(int16_t offset[3], int16_t gain[3], void* cb_arg);
extern bool accelgyro_accel_calibrate(
    accelgyro_accel_calib_step_callback step_cb,
    accelgyro_accel_calib_done_callback done_cb,
    void* cb_arg);

#endif /* !__ACCELGYRO_DEF_H__ */
