#ifndef __BARO_DEF_H__
#define __BARO_DEF_H__

#include "app_common.h"

extern void baro_init(void);
extern void baro_start(void);
extern void baro_stop(void);

extern int32_t     baroPressure;
extern int32_t     baroTemperature;
extern float       baroGroundPressure;
extern float       baroGroundAltitude;
extern int32_t     baroAltitude;

#endif /* !__BARO_DEF_H__ */
