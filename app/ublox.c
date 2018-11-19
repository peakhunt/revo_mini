#include "usart.h"
#include "circ_buffer.h"
#include "event_dispatcher.h"
#include "event_list.h"
#include "ublox_priv.h"

#define UBLOX_MAX_PAYLOAD_SIZE          256
#define UBLOX_MAX_TX_BUF_SIZE           64      

static volatile uint8_t    _rx_buf[UBLOX_MAX_PAYLOAD_SIZE];
static volatile uint8_t    _tx_buf[UBLOX_MAX_TX_BUF_SIZE];

static CircBuffer _rx_circ,
                  _tx_circ;

static UART_HandleTypeDef*  _huart = &huart3;
static int                  _irqn  = USART3_IRQn;

static uint8_t      _rx_char;

//
// ublox rx state machine
//
static uint8_t            _rx_step = 0;
static uint8_t            _class;
static uint8_t            _msg_id;
static uint8_t            _ck_b, _ck_a;
static uint16_t           _payload_length;
static uint16_t           _payload_counter;

static union
{
  ubx_nav_posllh_t posllh;
  ubx_nav_status_t status;
  ubx_nav_solution_t solution;
  ubx_nav_velned_t velned;
  ubx_nav_pvt_t pvt;
  ubx_nav_svinfo_t svinfo;
  ubx_mon_ver_t ver;
  ubx_nav_timeutc_t timeutc;
  uint8_t bytes[UBLOX_MAX_PAYLOAD_SIZE];
} _payload;

////////////////////////////////////////////////////////////////////////////////
//
// IRQ handlers
//
////////////////////////////////////////////////////////////////////////////////
void
ublox_rx_irq(void)
{
  if(circ_buffer_enqueue(&_rx_circ, &_rx_char, 1, true) == false)
  {
    // fucked up. overflow mostly.
    // do something here
  }
  event_set(1 << DISPATCH_EVENT_UBLOX_RX);
  HAL_UART_Receive_IT(_huart, &_rx_char, 1);
}

void
ublox_tx_irq(void)
{
}

////////////////////////////////////////////////////////////////////////////////
//
// circular buffer locking
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_enter_critical(CircBuffer* cb)
{
  NVIC_DisableIRQ(_irqn);
  __DSB();
  __ISB();
}

static void
ublox_leave_critical(CircBuffer* cb)
{
  NVIC_EnableIRQ(_irqn);
}

////////////////////////////////////////////////////////////////////////////////
//
// UBLOX core
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_handle_msg(void)
{
  switch(_msg_id)
  {
  case MSG_POSLLH:
    break;
  case MSG_STATUS:
    break;
  case MSG_SOL:
    break;
  case MSG_VELNED:
    break;
  case MSG_TIMEUTC:
    break;
  case MSG_PVT:
    break;
  case MSG_VER:
    break;
  default:
    // ignored
    break;
  }
}

static void
ublox_rx(uint8_t data)
{
  switch(_rx_step)
  {
  case 0:   // preamble 0x85
    if(data == 0x85)
    {
      _rx_step++;
    }
    break;

  case 1:   // preamble 0x62
    if(data != 0x62)
    {
      _rx_step = 0;
      break;
    }
    _rx_step++;
    break;

  case 2:   // class
    _class = data;
    _ck_b = _ck_a = data;
    _rx_step++;
    break;

  case  3:  // id
    _msg_id = data;
    _ck_b += (_ck_a += data);
    _rx_step++;
    break;

  case 4:   // payload length  low
    _payload_length = data;
    _ck_b += (_ck_a += data);
    _rx_step++;
    break;

  case 5:   // payload length high
    _payload_length |= (data << 8);
    if(_payload_length > UBLOX_MAX_PAYLOAD_SIZE)
    {
      _rx_step = 0;
      break;
    }
    _ck_b += (_ck_a += data);
    _rx_step++;

    _payload_counter = 0;
    if(_payload_counter == 0)
    {
      _rx_step = 7;
    }
    break;

  case 6:   // receiving payload
    _ck_b += (_ck_a += data);
    _payload.bytes[_payload_counter++] = data;
    if(_payload_counter == _payload_length)
    {
      _rx_step++;
    }
    break;

  case 7:   // csum ck_a
    _rx_step++;
    if(_ck_a != data)
    {
      // error
      _rx_step = 0;
    }
    break;

  case 8:   // csum ck_b
    _rx_step = 0;
    if(_ck_b != data)
    {
      // error
      break;
    }

    // FIXME
    // RX handling
    //
    ublox_handle_msg();
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// USART event handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
ublox_rx_event(uint32_t event)
{
  uint8_t   data;

  while(circ_buffer_dequeue(&_rx_circ, &data, 1, false) == true)
  {
    ublox_rx(data);
  }
}

static void
ublox_tx_event(uint32_t event)
{
  //
  // if circular buffer is not empty
  // reinitiate TX
  //
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
ublox_init(void)
{
  circ_buffer_init(&_rx_circ, _rx_buf, UBLOX_MAX_PAYLOAD_SIZE,
      ublox_enter_critical,
      ublox_leave_critical);

  circ_buffer_init(&_tx_circ, _tx_buf, UBLOX_MAX_TX_BUF_SIZE,
      ublox_enter_critical,
      ublox_leave_critical);

  event_register_handler(ublox_rx_event, DISPATCH_EVENT_UBLOX_RX);
  event_register_handler(ublox_tx_event, DISPATCH_EVENT_UBLOX_TX);

  HAL_UART_Receive_IT(_huart, &_rx_char, 1);
}
