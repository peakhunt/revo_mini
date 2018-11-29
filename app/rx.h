#ifndef __RX_DEF_H__
#define __RX_DEF_H__

#include "app_common.h"

#define RX_MAX_CHANNELS       16

#define RX_CMD_MIN            1000
#define RX_CMD_MAX            2000

typedef enum
{
  RX_CMD_ROLL         = 0,
  RX_CMD_PITCH,
  RX_CMD_YAW,
  RX_CMD_THROTTLE,
  RX_CMD_AUX1,
  RX_CMD_AUX2,
  RX_CMD_AUX3,
  RX_CMD_AUX4,
  RX_CMD_AUX5,
  RX_CMD_AUX6,
  RX_CMD_AUX7,
  RX_CMD_AUX8,
  RX_CMD_AUX9,
  RX_CMD_AUX10,
  RX_CMD_AUX11,
  RX_CMD_AUX12,
} rx_cmd_ndx_t;

//
// 1000~2000 when normal
// 
extern volatile uint32_t   rx_count;
extern volatile uint32_t   rx_timeout;
extern volatile uint32_t   rx_sync_err;
extern volatile uint32_t   rx_crc_err;

extern void rx_init(void);
extern bool rx_status(void);

extern uint16_t rx_cmd_get(rx_cmd_ndx_t ndx);

#endif /* !__RX_DEF_H__ */
