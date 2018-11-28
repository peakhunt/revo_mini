#ifndef __RX_DEF_H__
#define __RX_DEF_H__

#include "app_common.h"

#define RX_MAX_CHANNELS       16

#define RX_CMD_MIN            1000
#define RX_CMD_MAX            2000

//
// FIXME
//
#define RX_CMD_ROLL           0
#define RX_CMD_PITCH          1
#define RX_CMD_YAW            2
#define RX_CMD_THROTTLE       3

//
// 1000~2000 when normal
// 
extern volatile uint16_t    rx_cmd[RX_MAX_CHANNELS];

extern volatile uint32_t   rx_count;
extern volatile uint32_t   rx_timeout;
extern volatile uint32_t   rx_sync_err;
extern volatile uint32_t   rx_crc_err;

extern void rx_init(void);
extern bool rx_status(void);

#endif /* !__RX_DEF_H__ */
