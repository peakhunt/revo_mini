#ifndef __UBLOX_DEF_H__
#define __UBLOX_DEF_H__

#include "app_common.h"

extern void ublox_init(void);

extern void ublox_rx_irq(void);
extern void ublox_tx_irq(void);

#endif /* !__UBLOX_DEF_H__ */
