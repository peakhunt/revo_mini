#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "app.h"

#include "event_dispatcher.h"
#include "shell.h"

#include "mainloop_timer.h"
#include "blinky.h"

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
}

void
app_start(void)
{
}

void
app_loop(void)
{
  while(1)
  {
    event_dispatcher_dispatch();
  }
}
