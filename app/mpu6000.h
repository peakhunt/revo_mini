#ifndef __MPU_6000_DEF_H__
#define __MPU_6000_DEF_H__

#include "app_common.h"

/* MPU6000 registers */
#define MPU6000_AUX_VDDIO             0x01
#define MPU6000_SMPLRT_DIV            0x19
#define MPU6000_REG_ACCEL_XOFFS_H     0x06
#define MPU6000_REG_ACCEL_XOFFS_L     0x07
#define MPU6000_REG_ACCEL_YOFFS_H     0x08
#define MPU6000_REG_ACCEL_YOFFS_L     0x09
#define MPU6000_REG_ACCEL_ZOFFS_H     0x0A
#define MPU6000_REG_ACCEL_ZOFFS_L     0x0B
#define MPU6000_REG_GYRO_XOFFS_H      0x13
#define MPU6000_REG_GYRO_XOFFS_L      0x14
#define MPU6000_REG_GYRO_YOFFS_H      0x15
#define MPU6000_REG_GYRO_YOFFS_L      0x16
#define MPU6000_REG_GYRO_ZOFFS_H      0x17
#define MPU6000_REG_GYRO_ZOFFS_L      0x18
#define MPU6000_CONFIG                0x1A
#define MPU6000_GYRO_CONFIG           0x1B
#define MPU6000_ACCEL_CONFIG          0x1C
#define MPU6000_MOTION_THRESH         0x1F
#define MPU6000_INT_PIN_CFG           0x37
#define MPU6000_INT_ENABLE            0x38
#define MPU6000_INT_STATUS            0x3A
#define MPU6000_ACCEL_XOUT_H          0x3B
#define MPU6000_ACCEL_XOUT_L          0x3C
#define MPU6000_ACCEL_YOUT_H          0x3D
#define MPU6000_ACCEL_YOUT_L          0x3E
#define MPU6000_ACCEL_ZOUT_H          0x3F
#define MPU6000_ACCEL_ZOUT_L          0x40
#define MPU6000_TEMP_OUT_H            0x41
#define MPU6000_TEMP_OUT_L            0x42
#define MPU6000_GYRO_XOUT_H           0x43
#define MPU6000_GYRO_XOUT_L           0x44
#define MPU6000_GYRO_YOUT_H           0x45
#define MPU6000_GYRO_YOUT_L           0x46
#define MPU6000_GYRO_ZOUT_H           0x47
#define MPU6000_GYRO_ZOUT_L           0x48
#define MPU6000_MOT_DETECT_STATUS     0x61
#define MPU6000_SIGNAL_PATH_RESET     0x68
#define MPU6000_MOT_DETECT_CTRL       0x69
#define MPU6000_USER_CTRL             0x6A
#define MPU6000_PWR_MGMT_1            0x6B
#define MPU6000_PWR_MGMT_2            0x6C
#define MPU6000_FIFO_COUNTH           0x72
#define MPU6000_FIFO_COUNTL           0x73
#define MPU6000_FIFO_R_W              0x74
#define MPU6000_WHO_AM_I              0x75

/* Gyro sensitivities in °/s */
#define MPU6000_GYRO_SENS_250       ((float) 131)
#define MPU6000_GYRO_SENS_500       ((float) 65.5)
#define MPU6000_GYRO_SENS_1000      ((float) 32.8)
#define MPU6000_GYRO_SENS_2000      ((float) 16.4)

/* Acce sensitivities in g */
#define MPU6000_ACCE_SENS_2         ((float) 16384)
#define MPU6000_ACCE_SENS_4         ((float) 8192)
#define MPU6000_ACCE_SENS_8         ((float) 4096)
#define MPU6000_ACCE_SENS_16        ((float) 2048)

typedef struct 
{
  float       Temperature;      /*!< Temperature in degrees */
} MPU6000_t;

extern void mpu6000_init(MPU6000_t* mpu);
extern void mpu6000_read_all(MPU6000_t* mpu, int16_t a[3], int16_t g[3]);
extern uint8_t mpu6000_test(MPU6000_t* mpu, uint8_t reg);

#endif //!__MPU_6000_DEF_H__
