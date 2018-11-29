#ifndef __BLINKY_DEF_H__
#define __BLINKY_DEF_H__

//
// green led is used to arming/flight status
// red led is used for flight mode indication
//

#define BLINKY_GREEN_SLOW_BLINK_PERIOD          500
#define BLINKY_GREEN_FAST_BLINK_PERIOD          100

#define BLINKY_RED_MODE1_BLINK_PERIOD           200

typedef enum
{
  blinky_green_state_disarmed_not_ready,      // green slow blinking
  blinky_green_state_disarmed_ready,          // green steady
  blinky_green_state_armed,                   // green fast blinking
} blinky_green_state_t;

extern void blinky_init(void);
extern void blinky_change_state(blinky_green_state_t new_state);

#endif /* !__BLINKY_DEF_H__ */
