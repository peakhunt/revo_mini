#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "app.h"
#include "event_dispatcher.h"
#include "shell.h"
#include "mainloop_timer.h"
#include "blinky.h"
#include "pwm.h"
#include "accelgyro.h"
#include "micros.h"

void
app_init_f(void)
{
  event_dispatcher_init();
  mainloop_timer_init();
}

void
app_init(void)
{
  __disable_irq();
  shell_init();
  __enable_irq();

  blinky_init();
  micros_init();
}

void
app_start(void)
{
  pwm_start();

  accelgyro_init();
  accelgyro_start();
}

void
app_loop(void)
{
  while(1)
  {
    event_dispatcher_dispatch();
  }
}
