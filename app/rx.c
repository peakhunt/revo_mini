#include "rx.h"
#include "ibus.h"

volatile uint16_t    rx_cmd[RX_MAX_CHANNELS];

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
