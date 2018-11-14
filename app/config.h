#ifndef __CONFIG_DEF_H__
#define __CONFIG_DEF_H__

#include "app_common.h"

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
