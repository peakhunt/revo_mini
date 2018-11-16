#include "usart.h"
#include "ibus.h"
#include "event_dispatcher.h"
#include "event_list.h"

#define IBUS_PUSH(ib, d)      \
  ib->rx_buf[ib->data_ndx++] = d

static UART_HandleTypeDef*    _huart = &huart1;
static ibus_t*                _ibus;      // for multi ibus interfaces mapping. for now only single receiver

////////////////////////////////////////////////////////////////////////////////
//
// utilities
//
////////////////////////////////////////////////////////////////////////////////
static void
ibus_reset_state(ibus_t* ibus)
{
  ibus->data_ndx = 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// IRQ rx handling
//
////////////////////////////////////////////////////////////////////////////////
static void
ibus_rx_data(ibus_t* ibus)
{
  uint16_t  csum;

  csum = ibus->rx_buf[30] | (ibus->rx_buf[31] << 8);

  if(csum == ibus->csum)
  {
    //
    // got valid data
    // udpate
    // FIXME check failsafe data here
    // if RF connection is lost, iBUS receiver seems
    // to send failsafe data if it is set up to do so
    // otherwise. it keeps sending previous data.
    //
    for(int i = 0; i < IBUS_NUM_CHANNELS; i++)
    {
      ibus->chnl_data_ptr[i] = ibus->rx_buf[i * 2 + 2] |
                               ibus->rx_buf[i * 2 + 2 + 1] << 8;
    }
    event_set(1 << DISPATCH_EVENT_RC_RX);
  }
  ibus_reset_state(ibus);
}

static void
ibus_handle_rx(ibus_t* ibus, uint8_t data)
{
  switch(ibus->data_ndx)
  {
  case 0:
    if(data == 0x20)
    {
      IBUS_PUSH(ibus, data);
      ibus->csum  = 0xffff - 0x20;
    }
    else
    {
      ibus_reset_state(ibus);
    }
    break;

  case 1:
    if(data == 0x40)
    {
      IBUS_PUSH(ibus, data);
      ibus->csum -= data;
    }
    else
    {
      ibus_reset_state(ibus);
    }
    break;

  case 30:    // csum low
    IBUS_PUSH(ibus, data);
    break;

  case 31:    // csum high
    IBUS_PUSH(ibus, data);
    ibus_rx_data(ibus);
    break;

  default:    // data
    IBUS_PUSH(ibus, data);
    ibus->csum -= data;
    break;
  }
}

void
ibus_rx_callback(UART_HandleTypeDef* huart)
{
  ibus_handle_rx(_ibus, _ibus->rx_char);
  HAL_UART_Receive_IT(_huart, &_ibus->rx_char, 1);
}

////////////////////////////////////////////////////////////////////////////////
//
// RX ok check related. runs in mainloop context
//
////////////////////////////////////////////////////////////////////////////////
static void
ibus_rx_ok_timeout(SoftTimerElem* te)
{
  ibus_t* ibus  = container_of(te, ibus_t, rx_ok_timer);
  ibus->rx_ok     = false;
  
  ibus_reset_state(ibus);
}

static void
ibus_rx_received(uint32_t event)
{
  ibus_t* ibus  = _ibus;

  ibus->rx_ok = true;

  mainloop_timer_reschedule(&ibus->rx_ok_timer, 2000);
}

////////////////////////////////////////////////////////////////////////////////
//
// public services
//
////////////////////////////////////////////////////////////////////////////////
void
ibus_init(ibus_t* ibus, volatile uint16_t* chnl_data_ptr)
{
  soft_timer_init_elem(&ibus->rx_ok_timer);
  ibus->rx_ok_timer.cb    = ibus_rx_ok_timeout;
  ibus->rx_ok             = false;

  ibus->chnl_data_ptr = chnl_data_ptr;
  ibus_reset_state(ibus);

  _ibus = ibus;

  event_register_handler(ibus_rx_received, DISPATCH_EVENT_RC_RX);

  HAL_UART_Receive_IT(_huart, &ibus->rx_char, 1);
}
