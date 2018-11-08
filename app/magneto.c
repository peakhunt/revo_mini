#include "magneto.h"
#include "hmc5883.h"
#include "mainloop_timer.h"

static hmc5883Mag       _mag;
static SoftTimerElem    _sample_timer;

static void
mag_sample_timer_callback(SoftTimerElem* te)
{
  hmc5883_read(&_mag);
  mainloop_timer_schedule(&_sample_timer, 100);
}

void
magneto_init(void)
{
  hmc5883_init(&_mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_4_0);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb = mag_sample_timer_callback;
}

void
magneto_start(void)
{
  mainloop_timer_schedule(&_sample_timer, 100);
}

void
magneto_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
}

void
magneto_get(int16_t mag[3])
{
  mag[0] = _mag.rx;
  mag[1] = _mag.ry;
  mag[2] = _mag.rz;
}
