#include <stdlib.h>
#include <string.h>
#include "usart.h"
#include "circ_buffer.h"
#include "event_dispatcher.h"
#include "event_list.h"
#include "ublox_priv.h"
#include "gps.h"
#include "mainloop_timer.h"

#define UBX_VALID_GPS_DATE(valid) (valid & 1 << 0)
#define UBX_VALID_GPS_TIME(valid) (valid & 1 << 1)
#define UBX_VALID_GPS_DATE_TIME(valid) (UBX_VALID_GPS_DATE(valid) && UBX_VALID_GPS_TIME(valid))

#define UBLOX_MAX_PAYLOAD_SIZE          256
#define UBLOX_MAX_TX_BUF_SIZE           64      

#define NAV_STATUS_FIX_VALID            0x01

#define UBX_DYNMODEL_PEDESTRIAN 3
#define UBX_DYNMODEL_AIR_1G     6
#define UBX_DYNMODEL_AIR_4G     8

#define UBX_FIXMODE_2D_ONLY 1
#define UBX_FIXMODE_3D_ONLY 2
#define UBX_FIXMODE_AUTO    3

static void ublox_enter_critical(CircBuffer* cb);
static void ublox_leave_critical(CircBuffer* cb);
static void ublox_start_baud_config(void);
static void ublox_config_baud_step(void);
static void ublox_gps_config_step(void);

static volatile uint8_t    _rx_buf[UBLOX_MAX_PAYLOAD_SIZE];

static CircBuffer _rx_circ;

static UART_HandleTypeDef*  _huart = &huart3;
static int                  _irqn  = USART3_IRQn;

static uint8_t      _rx_char;

//
// ublox rx state machine
//
static uint8_t            _rx_step = 0;
static uint8_t            _class;
static uint8_t            _msg_id;
static uint8_t            _ck_b, _ck_a;
static uint16_t           _payload_length;
static uint16_t           _payload_counter;

static uint8_t            _next_fix_type;
static uint32_t           _hw_version;

static uint32_t           _target_baud    = 115200;
static uint32_t           _target_cmd     = 0;

static uint8_t            _next_baud_index;
static uint32_t           _avail_bauds[] =
{
  115200,
  57600,
  38400,
  19200,
  9600
};
static const char*        _baud_set_cmds[] =
{
  "$PUBX,41,1,0003,0001,115200,0*1E\r\n",
  "$PUBX,41,1,0003,0001,57600,0*2D\r\n",
  "$PUBX,41,1,0003,0001,38400,0*26\r\n",
  "$PUBX,41,1,0003,0001,19200,0*23\r\n",
  "$PUBX,41,1,0003,0001,9600,0*16\r\n"
};

static uint8_t          _config_step;

static const uint8_t default_payload[] = {
  // CFG-NAV5 - Set engine settings (original MWII code)
  0xFF, 0xFF, 0x03, 0x03, 0x00,
  // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
  0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
  // capturing the data from the U-Center binary console.
  0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
  0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// SBAS_AUTO, SBAS_EGNOS, SBAS_WAAS, SBAS_MSAS, SBAS_GAGAN, SBAS_NONE
static const uint32_t ubloxScanMode1[] = {
  0x00000000, 0x00000851, 0x0004E004, 0x00020200, 0x00000180, 0x00000000,
};

static union
{
  ubx_message_t   msg;
  uint8_t         bytes[58];
} _send_buffer;

static union
{
  ubx_nav_posllh_t posllh;
  ubx_nav_status_t status;
  ubx_nav_solution_t solution;
  ubx_nav_velned_t velned;
  ubx_nav_pvt_t pvt;
  ubx_nav_svinfo_t svinfo;
  ubx_mon_ver_t ver;
  ubx_nav_timeutc_t timeutc;
  uint8_t bytes[UBLOX_MAX_PAYLOAD_SIZE];
} _payload;

static SoftTimerElem    _rx_timeout;

////////////////////////////////////////////////////////////////////////////////
//
// utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline uint8_t 
ublox_gps_fix_type(bool valid, uint8_t ublox_fix_type)
{
  if (valid && ublox_fix_type == FIX_2D)
    return GPS_FIX_2D;

  if (valid && ublox_fix_type == FIX_3D)
    return GPS_FIX_3D;

  return GPS_NO_FIX;
}

static inline uint16_t
ublox_gps_contrain_hdop(uint32_t hdop)
{
    return (hdop > 9999) ? 9999 : hdop; // max 99.99m error
}

static inline uint16_t
ublox_gps_contrain_epe(uint32_t epe)
{
    return (epe > 9999) ? 9999 : epe; // max 99.99m error
}

static inline void
ublox_reset_data(void)
{
  _rx_step          = 0;
  _payload_length   = 0;
  _payload_counter  = 0;

  _next_fix_type    = GPS_NO_FIX;
  _hw_version       = 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// rx timeout callback
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_rx_timeout_callback(SoftTimerElem* te)
{
  gps_data.flags.rx_receiving = false;
  ublox_start_baud_config();
}

////////////////////////////////////////////////////////////////////////////////
//
// IRQ handlers
//
////////////////////////////////////////////////////////////////////////////////
void
ublox_rx_irq(void)
{
  if(circ_buffer_enqueue(&_rx_circ, &_rx_char, 1, true) == false)
  {
    // fucked up. overflow mostly.
    // do something here
  }
  event_set(1 << DISPATCH_EVENT_UBLOX_RX);
  HAL_UART_Receive_IT(_huart, &_rx_char, 1);
}

void
ublox_tx_irq(void)
{
  if(gps_data.state == gps_state_configuring_baud)
  {
    ublox_config_baud_step();
    return;
  }

  if(gps_data.state == gps_state_configuring_gps)
  {
    ublox_gps_config_step();
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// circular buffer locking
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_enter_critical(CircBuffer* cb)
{
  NVIC_DisableIRQ(_irqn);
  __DSB();
  __ISB();
}

static void
ublox_leave_critical(CircBuffer* cb)
{
  NVIC_EnableIRQ(_irqn);
}

////////////////////////////////////////////////////////////////////////////////
//
// UBLOX core
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_handle_msg(void)
{
  gps_data.flags.rx_receiving = true;
  gps_data.rx_msgs++;
  mainloop_timer_reschedule(&_rx_timeout, 2000);

  switch(_msg_id)
  {
  case MSG_POSLLH:
    gps_data.llh.lon  = _payload.posllh.longitude;
    gps_data.llh.lat  = _payload.posllh.latitude;
    gps_data.llh.alt  = _payload.posllh.altitude_msl / 10;    // in cm
    gps_data.eph      = ublox_gps_contrain_epe(_payload.posllh.horizontal_accuracy / 10);
    gps_data.epv      = ublox_gps_contrain_epe(_payload.posllh.vertical_accuracy / 10);

    gps_data.flags.valid_epe    = true;
    if(_next_fix_type != GPS_NO_FIX)
    {
      gps_data.fix_type = _next_fix_type;
    }
    break;

  case MSG_STATUS:
    _next_fix_type = ublox_gps_fix_type(
        _payload.status.fix_status & NAV_STATUS_FIX_VALID,
        _payload.status.fix_type);

    if(_next_fix_type == GPS_NO_FIX)
    {
      gps_data.fix_type = GPS_NO_FIX;
    }
    break;

  case MSG_SOL:
    _next_fix_type = ublox_gps_fix_type(
        _payload.solution.fix_status & NAV_STATUS_FIX_VALID, 
        _payload.solution.fix_type);
    if(_next_fix_type == GPS_NO_FIX)
    {
      gps_data.fix_type = GPS_NO_FIX;
    }

    gps_data.num_sat  = _payload.solution.satellites;
    gps_data.hdop     = ublox_gps_contrain_hdop(_payload.solution.position_DOP);
    break;

  case MSG_VELNED:
    gps_data.ground_speed   = _payload.velned.speed_2d;             // cm/s
    gps_data.ground_course  = _payload.velned.heading_2d / 10000;   // deg * 10
    gps_data.vel_ned[0]     = _payload.velned.ned_north;
    gps_data.vel_ned[1]     = _payload.velned.ned_east;
    gps_data.vel_ned[2]     = _payload.velned.ned_down;

    gps_data.flags.valid_vel_ne = true;
    gps_data.flags.valid_vel_d  = true;
    break;

  case MSG_TIMEUTC:
    if(UBX_VALID_GPS_DATE_TIME(_payload.timeutc.valid))
    {
      gps_data.time.year      = _payload.timeutc.year;
      gps_data.time.month     = _payload.timeutc.month;
      gps_data.time.day       = _payload.timeutc.day;
      gps_data.time.hours     = _payload.timeutc.hour;
      gps_data.time.minutes   = _payload.timeutc.min;
      gps_data.time.seconds   = _payload.timeutc.sec;
      gps_data.time.millis    = _payload.timeutc.nano / (1000*1000);

      gps_data.flags.valid_time   = true;
    }
    else
    {
      gps_data.flags.valid_time   = false;
    }
    break;

  case MSG_PVT:
    _next_fix_type = ublox_gps_fix_type(
        _payload.pvt.fix_status & NAV_STATUS_FIX_VALID, 
        _payload.pvt.fix_type);
    gps_data.fix_type       = _next_fix_type;
    gps_data.llh.lon        = _payload.pvt.longitude;
    gps_data.llh.lat        = _payload.pvt.latitude;
    gps_data.llh.alt        = _payload.pvt.altitude_msl / 10;  //alt in cm
    gps_data.vel_ned[0]     = _payload.pvt.ned_north / 10;  // to cm/s
    gps_data.vel_ned[1]     = _payload.pvt.ned_east / 10;   // to cm/s
    gps_data.vel_ned[2]     = _payload.pvt.ned_down / 10;   // to cm/s
    gps_data.ground_speed   = _payload.pvt.speed_2d / 10;    // to cm/s
    gps_data.ground_course  = (uint16_t) (_payload.pvt.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
    gps_data.num_sat        = _payload.pvt.satellites;
    gps_data.eph            = ublox_gps_contrain_epe(_payload.pvt.horizontal_accuracy / 10);
    gps_data.epv            = ublox_gps_contrain_epe(_payload.pvt.vertical_accuracy / 10);
    gps_data.hdop           = ublox_gps_contrain_hdop(_payload.pvt.position_DOP);

    gps_data.flags.valid_vel_ne = true;
    gps_data.flags.valid_vel_d  = true;
    gps_data.flags.valid_epe    = true;

    if (UBX_VALID_GPS_DATE_TIME(_payload.pvt.valid))
    {
      gps_data.time.year    = _payload.pvt.year;
      gps_data.time.month   = _payload.pvt.month;
      gps_data.time.day     = _payload.pvt.day;
      gps_data.time.hours   = _payload.pvt.hour;
      gps_data.time.minutes = _payload.pvt.min;
      gps_data.time.seconds = _payload.pvt.sec;
      gps_data.time.millis  = _payload.pvt.nano / (1000*1000);

      gps_data.flags.valid_time = true;
    }
    else
    {
      gps_data.flags.valid_time = false;
    }
    break;

  case MSG_VER:
    if (_class == CLASS_MON)
    {
      _hw_version = atoi(_payload.ver.hwVersion);
#if 0
      capGalileo = ((gpsState.hwVersion >= 80000) && (_buffer.ver.swVersion[9] > '2')); // M8N and SW major 3 or later
#endif
    }
    break;

  default:
    // ignored
    break;
  }
}

static void
ublox_rx(uint8_t data)
{
  switch(_rx_step)
  {
  case 0:   // preamble 0x85
    if(data != PREAMBLE1)
    {
      gps_data.rx_unsync++;
      break;
    }
    _rx_step++;
    break;

  case 1:   // preamble 0x62
    if(data != PREAMBLE2)
    {
      gps_data.rx_unsync++;
      _rx_step = 0;
      break;
    }
    _rx_step++;
    break;

  case 2:   // class
    _class = data;
    _ck_b = _ck_a = data;
    _rx_step++;
    break;

  case  3:  // id
    _msg_id = data;
    _ck_b += (_ck_a += data);
    _rx_step++;
    break;

  case 4:   // payload length  low
    _payload_length = data;
    _ck_b += (_ck_a += data);
    _rx_step++;
    break;

  case 5:   // payload length high
    _payload_length |= (data << 8);
    if(_payload_length > UBLOX_MAX_PAYLOAD_SIZE)
    {
      _rx_step = 0;
      gps_data.rx_unsync++;
      break;
    }
    _ck_b += (_ck_a += data);
    _rx_step++;

    _payload_counter = 0;
    if(_payload_length == 0)
    {
      _rx_step = 7;
    }
    break;

  case 6:   // receiving payload
    _ck_b += (_ck_a += data);
    _payload.bytes[_payload_counter++] = data;
    if(_payload_counter == _payload_length)
    {
      _rx_step++;
    }
    break;

  case 7:   // csum ck_a
    _rx_step++;
    if(_ck_a != data)
    {
      gps_data.rx_crc_err++;
      _rx_step = 0;
    }
    break;

  case 8:   // csum ck_b
    _rx_step = 0;
    if(_ck_b != data)
    {
      gps_data.rx_crc_err++;
      break;
    }

    // RX handling
    //
    ublox_handle_msg();
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// ublox gps config
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_start_receiving(void)
{
  //
  // enable receiver
  // and start rx timer
  //
  soft_timer_init_elem(&_rx_timeout);
  _rx_timeout.cb    = ublox_rx_timeout_callback;

  mainloop_timer_schedule(&_rx_timeout, 2000);

  HAL_UART_Receive_IT(_huart, &_rx_char, 1);

  gps_data.state  = gps_state_receiving;
}

static void
ublox_update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
  while(len--)
  {
    *ck_a += *data;
    *ck_b += *ck_a;
    data++;
  }
}

static void
ublox_send_config_msg(void)
{
  uint8_t ck_a=0, ck_b=0;

  _send_buffer.msg.header.preamble1=PREAMBLE1;
  _send_buffer.msg.header.preamble2=PREAMBLE2;

  ublox_update_checksum(&_send_buffer.bytes[2], _send_buffer.msg.header.length+4, &ck_a, &ck_b);

  _send_buffer.bytes[_send_buffer.msg.header.length+6] = ck_a;
  _send_buffer.bytes[_send_buffer.msg.header.length+7] = ck_b;

  HAL_UART_Transmit_IT(_huart,
      _send_buffer.bytes,
      _send_buffer.msg.header.length + 8);

  //check ack/nack here
}

static void
ublox_config_nav5(uint8_t dyn_model, uint8_t fix_mode)
{
  _send_buffer.msg.header.msg_class   = CLASS_CFG;
  _send_buffer.msg.header.msg_id      = MSG_CFG_NAV_SETTINGS;
  _send_buffer.msg.header.length      = 0x24;
  memcpy(_send_buffer.msg.payload.bytes, default_payload, sizeof(default_payload));
  _send_buffer.msg.payload.bytes[2]   = dyn_model;
  _send_buffer.msg.payload.bytes[3]   = fix_mode;

  ublox_send_config_msg();
}

static void
ublox_config_msg(uint8_t _class, uint8_t id, uint8_t rate)
{
  _send_buffer.msg.header.msg_class     = CLASS_CFG;
  _send_buffer.msg.header.msg_id        = MSG_CFG_SET_RATE;
  _send_buffer.msg.header.length        = 3;
  _send_buffer.msg.payload.msg._class   = _class;
  _send_buffer.msg.payload.msg.id       = id;
  _send_buffer.msg.payload.msg.rate     = rate;

  ublox_send_config_msg();
}

static void
ublox_cofig_rate(uint16_t rate)
{
  _send_buffer.msg.header.msg_class   = CLASS_CFG;
  _send_buffer.msg.header.msg_id      = MSG_CFG_RATE;
  _send_buffer.msg.header.length      = 6;
  _send_buffer.msg.payload.rate.meas  = rate;
  _send_buffer.msg.payload.rate.nav   =1;
  _send_buffer.msg.payload.rate.time  =1;

  ublox_send_config_msg();
}

static void
ublox_config_sbas(void)
{
  _send_buffer.msg.header.msg_class         = CLASS_CFG;
  _send_buffer.msg.header.msg_id            = MSG_CFG_SBAS;
  _send_buffer.msg.header.length            = 8;
  //_send_buffer.msg.payload.sbas.mode=(gpsState.gpsConfig->sbasMode == SBAS_NONE?2:3);
  _send_buffer.msg.payload.sbas.mode        = 2;
  _send_buffer.msg.payload.sbas.usage       = 3;
  _send_buffer.msg.payload.sbas.maxSBAS     = 3;
  _send_buffer.msg.payload.sbas.scanmode2   = 0;
  _send_buffer.msg.payload.sbas.scanmode1   = ubloxScanMode1[SBAS_NONE];

  ublox_send_config_msg();
}

static void
ublox_do_gps_config(int step)
{
  switch(step)
  {
  case 0:   // air 1G setup
    ublox_config_nav5(UBX_DYNMODEL_AIR_1G, UBX_FIXMODE_AUTO);
    break;

  // disable MNEA
  case 1: 
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_GGA, 0);
    break;

  case 2:
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_GLL, 0);
    break;

  case 3:
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_GSA, 0);
    break;

  case 4:
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_GSV, 0);
    break;

  case 5:
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_RMC, 0);
    break;

  case 6:
    ublox_config_msg(MSG_CLASS_NMEA, MSG_NMEA_VGS, 0);
    break;

  // enable UBX
  case 7:
    ublox_config_msg(MSG_CLASS_UBX, MSG_POSLLH, 1);
    break;
  
  case 8:
    ublox_config_msg(MSG_CLASS_UBX, MSG_STATUS, 1);
    break;

  case 9:
    ublox_config_msg(MSG_CLASS_UBX, MSG_SOL,    1);
    break;

  case 10:
    ublox_config_msg(MSG_CLASS_UBX, MSG_VELNED, 1);
    break;

  case 11:
    ublox_config_msg(MSG_CLASS_UBX, MSG_SVINFO, 0);
    break;

  case 12:
    ublox_config_msg(MSG_CLASS_UBX, MSG_TIMEUTC,10);
    break;

  case 13:   // rate
    ublox_cofig_rate(200);    // 5Hz
    break;

  case 14:   // sbas
    ublox_config_sbas();
    break;

  case 15:   // galileo
    break;
  }
}

static void
ublox_gps_config_step(void)
{
  _config_step++;
  if(_config_step >= 15)
  {
    //
    // enable receiver
    // and start rx timer
    //
    ublox_start_receiving();
    return;
  }

  ublox_do_gps_config(_config_step);
}

static void
ublox_start_gps_config(void)
{
  gps_data.state  = gps_state_configuring_gps;
  _config_step    = 0;
  ublox_do_gps_config(_config_step);
}

////////////////////////////////////////////////////////////////////////////////
//
// auto baud configure
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_reset_baud(uint32_t   baud)
{
  HAL_UART_DeInit(_huart);
  _huart->Init.BaudRate = baud;
  HAL_UART_Init(_huart);

  ublox_reset_data();

  circ_buffer_init(&_rx_circ, _rx_buf, UBLOX_MAX_PAYLOAD_SIZE,
      ublox_enter_critical,
      ublox_leave_critical);
}

static void
ublox_config_baud(uint8_t    target_index)
{
  ublox_reset_baud(_avail_bauds[target_index]);

  HAL_UART_Transmit_IT(_huart, (uint8_t*)_baud_set_cmds[_target_cmd],
      strlen(_baud_set_cmds[_target_cmd]));
}

static void
ublox_config_baud_step(void)
{
  _next_baud_index++;

  if(_next_baud_index >= NARRAY(_avail_bauds))
  {
    // done configuring baud
    ublox_reset_baud(_target_baud);
    ublox_start_gps_config();
    return;
  }

  ublox_config_baud(_next_baud_index);
}

static void
ublox_start_baud_config(void)
{
  gps_data.state = gps_state_configuring_baud;
  _next_baud_index  = 0;
  ublox_config_baud(_next_baud_index);
}

////////////////////////////////////////////////////////////////////////////////
//
// USART event handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_rx_event(uint32_t event)
{
  uint8_t   data;

  while(circ_buffer_dequeue(&_rx_circ, &data, 1, false) == true)
  {
    gps_data.rx_bytes++;
    ublox_rx(data);
  }
}

static void
ublox_tx_event(uint32_t event)
{
  if(gps_data.state == gps_state_configuring_baud)
  {
    ublox_config_baud_step();
    return;
  }

  if(gps_data.state == gps_state_configuring_gps)
  {
    ublox_gps_config_step();
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
ublox_init(void)
{
  circ_buffer_init(&_rx_circ, _rx_buf, UBLOX_MAX_PAYLOAD_SIZE,
      ublox_enter_critical,
      ublox_leave_critical);

  event_register_handler(ublox_rx_event, DISPATCH_EVENT_UBLOX_RX);
  event_register_handler(ublox_tx_event, DISPATCH_EVENT_UBLOX_TX);

  ublox_reset_data();

  ublox_start_baud_config();
}
