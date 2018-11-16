#include "rx.h"
#include "ibus.h"

volatile uint16_t    rx_cmd[RX_MAX_CHANNELS];

volatile uint32_t   rx_count;
volatile uint32_t   rx_timeout;
volatile uint32_t   rx_sync_err;
volatile uint32_t   rx_crc_err;

static ibus_t       _ibus;

void
rx_init(void)
{
  ibus_init(&_ibus, rx_cmd);
}

bool
rx_status(void)
{
  return _ibus.rx_ok;
}
