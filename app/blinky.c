#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "app_common.h"
#include "blinky.h"
#include "mainloop_timer.h"

#define BLINKY1_INTERVAL         50
#define BLINKY2_INTERVAL         100

static SoftTimerElem    _blinky_timer1;
static SoftTimerElem    _blinky_timer2;

static void
blinky_callback1(SoftTimerElem* te)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
  mainloop_timer_schedule(&_blinky_timer1, BLINKY1_INTERVAL);
}

static void
blinky_callback2(SoftTimerElem* te)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  mainloop_timer_schedule(&_blinky_timer2, BLINKY2_INTERVAL);
}

void
blinky_init(void)
{
  soft_timer_init_elem(&_blinky_timer1);
  _blinky_timer1.cb    = blinky_callback1;

  soft_timer_init_elem(&_blinky_timer2);
  _blinky_timer2.cb    = blinky_callback2;

  mainloop_timer_schedule(&_blinky_timer1, BLINKY1_INTERVAL);
  mainloop_timer_schedule(&_blinky_timer2, BLINKY2_INTERVAL);
}
