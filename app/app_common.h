#ifndef __APP_COMMON_DEF_H__
#define __APP_COMMON_DEF_H__

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define bool      uint8_t
#define true      1
#define false     0

#define CRASH()                 \
{                               \
  char*   __bullshit = NULL;    \
  *__bullshit = 0;              \
}

#define NARRAY(a)       (sizeof(a)/sizeof(a[0]))
// #define UNUSED(a)       (void)(a)

#define HI_BYTE(w)      ((uint8_t)((w >> 8) & 0xff))
#define LO_BYTE(w)      ((uint8_t)(w  & 0xff))

#define BUFFER_TO_U16(b)              ((uint16_t)b[0] << 8 | (uint16_t)b[1])

#define U16_TO_BUFFER(w, b)           \
  b[0] = HI_BYTE(w);                  \
  b[1] = LO_BYTE(w);

//#define MIN(a,b)      ((a) < (b) ? (a) : (b))
//#define MAX(a,b)      ((a) > (b) ? (a) : (b))

#define X       0
#define Y       1
#define Z       2

#define ROLL    0
#define PITCH   1
#define YAW     2

////////////////////////////////////////////////////////////////////////////////
//
// system uptime. defined in stm32f1xx_callback.c
//
////////////////////////////////////////////////////////////////////////////////
extern volatile uint32_t     __uptime;
extern volatile uint32_t     __msec;

////////////////////////////////////////////////////////////////////////////////
//
// misc utilities
//
////////////////////////////////////////////////////////////////////////////////

#endif //!__APP_COMMON_DEF_H__
