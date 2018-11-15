#ifndef __RX_DEF_H__
#define __RX_DEF_H__

#include "app_common.h"

#define RX_MAX_CHANNELS       16

extern volatile uint16_t    rx_cmd[RX_MAX_CHANNELS];

extern void rx_init(void);
extern bool rx_status(void);

#endif /* !__RX_DEF_H__ */
