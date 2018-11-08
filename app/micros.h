#ifndef __MICROS_DEF_H__
#define __MICROS_DEF_H__

#include "app_common.h"

extern void micros_init(void);
extern uint32_t micros_get(void);
extern void micros_update_callback(void);

#endif /* !__MICROS_DEF_H__ */
