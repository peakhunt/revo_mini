#ifndef __IBUS_DEF_H__
#define __IBUS_DEF_H__

#include "usart.h"
#include "app_common.h"
#include "mainloop_timer.h"

#define IBUS_NUM_CHANNELS         14
#define IBUS_RX_BUFFER_SIZE       32

typedef struct
{
  uint16_t          csum;
  volatile uint16_t*chnl_data_ptr;
  uint8_t           rx_buf[IBUS_RX_BUFFER_SIZE];
  volatile uint8_t  data_ndx;

  uint8_t           rx_char;
  bool              rx_ok;
  
  SoftTimerElem     rx_ok_timer;
} ibus_t;

extern void ibus_init(ibus_t* ibus, volatile uint16_t* chnl_data_ptr);
extern void ibus_rx_callback(UART_HandleTypeDef* huart);

#endif /* !__IBUS_DEF_H__ */
