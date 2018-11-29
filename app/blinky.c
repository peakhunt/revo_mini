#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "app_common.h"
#include "blinky.h"
#include "mainloop_timer.h"

#define GREEN_LED_PORT          GPIOB
#define GREEN_LED_PIN           GPIO_PIN_4

#define RED_LED_PORT            GPIOB
#define RED_LED_PIN             GPIO_PIN_5

static SoftTimerElem            _blinky_green_timer;
static blinky_green_state_t     _green_state;

static inline void
blinky_green_timer_do_state_change(void)
{
  switch(_green_state)
  {
  case blinky_green_state_disarmed_not_ready:
    mainloop_timer_schedule(&_blinky_green_timer, BLINKY_GREEN_SLOW_BLINK_PERIOD);
    break;

  case blinky_green_state_armed:
    mainloop_timer_schedule(&_blinky_green_timer, BLINKY_GREEN_FAST_BLINK_PERIOD);
    break;

  case blinky_green_state_disarmed_ready:
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    break;
  }
}

static void
blinky_green_timer_callback(SoftTimerElem* te)
{
  HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);

  blinky_green_timer_do_state_change();
}

void
blinky_init(void)
{
  soft_timer_init_elem(&_blinky_green_timer);
  _blinky_green_timer.cb    = blinky_green_timer_callback;

  _green_state = blinky_green_state_disarmed_not_ready;
  blinky_green_timer_do_state_change();

  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
}

void
blinky_change_state(blinky_green_state_t new_state)
{
  if(new_state == _green_state)
  {
    return;
  }

  mainloop_timer_cancel(&_blinky_green_timer);

  _green_state = new_state;
  blinky_green_timer_do_state_change();
}
