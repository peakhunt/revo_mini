#include "gps.h"
#include "ublox.h"

gps_data_t        gps_data;

void
gps_init(void)
{
  ublox_init();
}
