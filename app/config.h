#ifndef __CONFIG_DEF_H__
#define __CONFIG_DEF_H__

#include "app_common.h"
#include "rx.h"

#define CONFIG_VERSION          1
#define CONFIG_MAGIC            0x63149654

typedef struct
{
  // header
  int32_t     version;
  int32_t     magic;

  // calibration parameters
  int16_t     mag_offset[3];
  float       mag_scale[3];
  int16_t     accel_gain[3];
  int16_t     accel_offset[3];
  int16_t     gyro_offset[3];
  int16_t     mag_decl;

  float       roll_kX[3];     // KP/KI/KD for roll
  float       pitch_kX[3];    // KP/KI/KD for pitch
  float       yaw_kX[3];      // KP/KI/KD for yaw

  int16_t     roll_max;       // max roll angle in decidegree
  int16_t     pitch_max;      // max pitch angle in decidegree
  int16_t     yaw_rate_max;   // max yaw rate in decidegree per sec

  uint16_t    motor_min;
  uint16_t    motor_max;
  uint16_t    min_flight_throttle;

  uint8_t     rx_cmd_ndx[RX_MAX_CHANNELS];
} config_t;

typedef struct
{
  config_t    cfg;
  uint32_t    crc;
} config_internal_t;

extern void config_init(void);
extern void config_save(void);

extern config_internal_t    _config;

#define GCFG       (&(_config.cfg))

#endif /* !__CONFIG_DEF_H__ */
