#include "rx.h"
#include "ibus.h"
#include "config.h"

static volatile uint16_t    _rx_cmd[RX_MAX_CHANNELS];

volatile uint32_t   rx_count;
volatile uint32_t   rx_timeout;
volatile uint32_t   rx_sync_err;
volatile uint32_t   rx_crc_err;

static ibus_t       _ibus;

void
rx_init(void)
{
  ibus_init(&_ibus, _rx_cmd);
}

bool
rx_status(void)
{
  return _ibus.rx_ok;
}

uint16_t
rx_cmd_get(rx_cmd_ndx_t ndx)
{
  return _rx_cmd[GCFG->rx_cmd_ndx[ndx]];
}
