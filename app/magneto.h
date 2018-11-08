#ifndef __MAGNETO_DEF_H__
#define __MAGNETO_DEF_H__

#include "app_common.h"

extern void magneto_init(void);
extern void magneto_start(void);
extern void magneto_stop(void);
extern void magneto_get(int16_t mag[3]);

#endif /* __MAGNETO_DEF_H__ */
