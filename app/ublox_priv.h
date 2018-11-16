#ifndef __UBLOX_PRIV_DEF_H__
#define __UBLOX_PRIV_DEF_H__

#include "app_common.h"

typedef enum
{
  PREAMBLE1 = 0xB5,
  PREAMBLE2 = 0x62,
  CLASS_NAV = 0x01,
  CLASS_ACK = 0x05,
  CLASS_CFG = 0x06,
  CLASS_MON = 0x0A,
  MSG_CLASS_UBX = 0x01,
  MSG_CLASS_NMEA = 0xF0,
  MSG_VER = 0x04,
  MSG_ACK_NACK = 0x00,
  MSG_ACK_ACK = 0x01,
  MSG_NMEA_GGA = 0x0,
  MSG_NMEA_GLL = 0x1,
  MSG_NMEA_GSA = 0x2,
  MSG_NMEA_GSV = 0x3,
  MSG_NMEA_RMC = 0x4,
  MSG_NMEA_VGS = 0x5,
  MSG_POSLLH = 0x2,
  MSG_STATUS = 0x3,
  MSG_SOL = 0x6,
  MSG_PVT = 0x7,
  MSG_VELNED = 0x12,
  MSG_TIMEUTC = 0x21,
  MSG_SVINFO = 0x30,
  MSG_CFG_PRT = 0x00,
  MSG_CFG_RATE = 0x08,
  MSG_CFG_SET_RATE = 0x01,
  MSG_CFG_NAV_SETTINGS = 0x24,
  MSG_CFG_SBAS = 0x16,
  MSG_CFG_GNSS = 0x3e
} ubx_msg_id_t;

typedef struct
{
  uint8_t mode;
  uint8_t usage;
  uint8_t maxSBAS;
  uint8_t scanmode2;
  uint32_t scanmode1;
} ubx_sbas_t;

typedef struct
{
  uint8_t _class;
  uint8_t id;
  uint8_t rate;
} ubx_msg_t;

typedef struct
{
  uint16_t meas;
  uint16_t nav;
  uint16_t time;
} ubx_rate_t;

typedef union
{
  uint8_t bytes[60]; // sizeof Galileo config
  ubx_sbas_t sbas;
  ubx_msg_t msg;
  ubx_rate_t rate;
} ubx_payload_t;

typedef struct
{
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
} ubx_header_t;

typedef struct
{
  ubx_header_t  header;
  ubx_payload_t payload;
} __attribute__((packed)) ubx_message;

typedef struct
{
  char swVersion[30];      // Zero-terminated Software Version String
  char hwVersion[10];      // Zero-terminated Hardware Version String
} ubx_mon_ver_t;

typedef struct
{
  uint32_t time;              // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
} ubx_nav_posllh_t;

typedef struct
{
  uint32_t time;              // GPS msToW
  uint8_t fix_type;
  uint8_t fix_status;
  uint8_t differential_status;
  uint8_t res;
  uint32_t time_to_first_fix;
  uint32_t uptime;            // milliseconds
} ubx_nav_status_t;

typedef struct
{
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
} ubx_nav_solution_t;

typedef struct
{
  uint32_t time;              // GPS msToW
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
} ubx_nav_velned_t;

typedef struct
{
  uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
  uint8_t svid;               // Satellite ID
  uint8_t flags;              // Bitmask
  uint8_t quality;            // Bitfield
  uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
  uint8_t elev;               // Elevation in integer degrees
  int16_t azim;               // Azimuth in integer degrees
  int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel_t;

typedef struct
{
  uint32_t time;              // GPS Millisecond time of week
  uint8_t numCh;              // Number of channels
  uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
  uint16_t reserved2;         // Reserved
  ubx_nav_svinfo_channel_t channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo_t;

typedef struct
{
  uint32_t time;              // GPS msToW
  uint32_t tAcc;
  int32_t nano;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
} ubx_nav_timeutc_t;

typedef struct
{
  uint32_t time; // GPS msToW
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t fix_status;
  uint8_t reserved1;
  uint8_t satellites;
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  int32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
  uint16_t position_DOP;
  uint16_t reserved2;
  uint16_t reserved3;
} ubx_nav_pvt_t;

#endif /* !__UBLOX_PRIV_DEF_H__ */
