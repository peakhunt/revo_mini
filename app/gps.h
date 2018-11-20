#ifndef __GPS_DEF_H__
#define __GPS_DEF_H__

#include "app_common.h"

typedef struct
{
  uint16_t      year;       // full year
  uint8_t       month;      // 1-12
  uint8_t       day;        // 1-31
  uint8_t       hours;      // 0-23
  uint8_t       minutes;    // 0-59
  uint8_t       seconds;    // 0-59
  uint16_t      millis;     // 0-9999
} date_time_t;

typedef struct
{
  int32_t     lat;
  int32_t     lon;
  int32_t     alt;
} gps_location_t;

typedef enum
{
  GPS_NO_FIX      = 0,
  GPS_FIX_2D,
  GPS_FIX_3D
} gps_fixtype_t;

typedef enum 
{
  SBAS_AUTO = 0,
  SBAS_EGNOS,
  SBAS_WAAS,
  SBAS_MSAS,
  SBAS_GAGAN,
  SBAS_NONE
} gps_sbad_mode_t;

typedef enum 
{
  gps_state_configuring_baud,
  gps_state_configuring_gps,
  gps_state_receiving,
} gps_state_t;

typedef struct
{
  gps_location_t      llh;
  uint16_t            eph;      // horizontal accuracy (cm)
  uint16_t            epv;      // vertical accuracy (cm)

  gps_fixtype_t       fix_type;

  int16_t             ground_speed;
  int16_t             ground_course;
  int16_t             vel_ned[3];

  uint8_t             num_sat;
  uint16_t            hdop;

  date_time_t         time;

  struct
  {
    bool      gps_heartbeat;
    bool      valid_vel_ne;
    bool      valid_vel_d;
    bool      valid_mag;
    bool      valid_epe;
    bool      valid_time;
    bool      rx_receiving;
  } flags;

  gps_state_t state;
  uint32_t    rx_bytes;
  uint32_t    rx_msgs;
  uint32_t    rx_crc_err;
  uint32_t    rx_unsync;
} gps_data_t;

extern gps_data_t     gps_data;

extern void gps_init(void);

#endif /* !__GPS_DEF_H__ */
