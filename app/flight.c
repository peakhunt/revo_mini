#include "flight.h"
#include "pid.h"
#include "mainloop_timer.h"
#include "imu.h"
#include "rx.h"
#include "pwm.h"
#include "config.h"
#include "math_helper.h"

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static pid_control_t      _pidc_roll,
                          _pidc_pitch,
                          _pidc_yaw;

static SoftTimerElem      _loop_timer;

////////////////////////////////////////////////////////////////////////////////
//
// visibles. module output
//
////////////////////////////////////////////////////////////////////////////////
float                     pid_out[3];
float                     pid_target[3];        // RP - deci degree. Y - dps
uint16_t                  pid_motor[4];
flight_state_t            flight_state;

////////////////////////////////////////////////////////////////////////////////
//
// utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
flight_control_set_motor(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  pid_motor[0] = m1;
  pid_motor[1] = m2;
  pid_motor[2] = m3;
  pid_motor[3] = m4;

  //
  // FIXME
  // motor mapping
  //
  for(int i = 0; i < 4; i++)
  {
    pwm_set_duty(i, pid_motor[i]);
  }
}

static inline void
flight_control_set_motor_to_min(void)
{
  flight_control_set_motor(
      GCFG->motor_min,
      GCFG->motor_min,
      GCFG->motor_min,
      GCFG->motor_min);
}

static inline bool
flight_is_arming_position(void)
{
  static const uint8_t deadband = 20;
  //
  // arming/disarming stick
  //
  // throttle down
  // yaw left
  // roll right
  // pitch down
  //
  if(rx_cmd_get(RX_CMD_THROTTLE) < (RX_CMD_MIN + deadband) &&
     rx_cmd_get(RX_CMD_YAW) < (RX_CMD_MIN + deadband) &&
     rx_cmd_get(RX_CMD_PITCH) < (RX_CMD_MIN + deadband) &&
     rx_cmd_get(RX_CMD_ROLL) > (RX_CMD_MAX - deadband))
  {
    return true;
  }
  return false;
}

static inline void
flight_reset(void)
{
  flight_control_set_motor_to_min();

  pid_control_init(&_pidc_roll);
  pid_control_init(&_pidc_pitch);
  pid_control_init(&_pidc_yaw);
}

////////////////////////////////////////////////////////////////////////////////
//
// flight core
//
////////////////////////////////////////////////////////////////////////////////
static void
flight_control_update_command_target(void)
{
  // roll   1000~2000 ->  +- roll_max
  pid_target[0] = lerp(rx_cmd_get(RX_CMD_ROLL),  RX_CMD_MIN, RX_CMD_MAX, -GCFG->roll_max, GCFG->roll_max);

  // pitch  1000-2000 ->  +- pitch_max
  pid_target[1] = lerp(rx_cmd_get(RX_CMD_PITCH), RX_CMD_MIN, RX_CMD_MAX, -GCFG->pitch_max, GCFG->pitch_max);

  // yaw    1000-2000 ->  +- yaw_rate_max
  pid_target[2] = lerp(rx_cmd_get(RX_CMD_YAW), RX_CMD_MIN, RX_CMD_MAX, -GCFG->yaw_rate_max, GCFG->yaw_rate_max);
}

/*
   motor order

      M4          M2
     <----\    / --->
           \  /
            \/
            /\
           /  \
      M3  /    \  M1
    ---->/      \<---
*/
static void
flight_control_update_motor_out(void)
{
  float   m[4];
  float   throttle  = rx_cmd_get(RX_CMD_THROTTLE),
          roll      = pid_out[0],
          pitch     = pid_out[1],
          yaw       = pid_out[2];

  if(throttle >= GCFG->min_flight_throttle)
  {
    m[0] = throttle - roll - pitch + yaw; 
    m[1] = throttle - roll + pitch - yaw; 
    m[2] = throttle + roll - pitch - yaw; 
    m[3] = throttle + roll + pitch + yaw; 

    for(int i = 0; i < 4; i++)
    {
      clamp(&m[i], GCFG->motor_min, GCFG->motor_max);
    }
  }
  else
  {
    m[0] = m[1] = m[2] = m[3] = throttle;
  }

  flight_control_set_motor((uint16_t)m[0],
      (uint16_t)m[1],
      (uint16_t)m[2],
      (uint16_t)m[3]);
}

static void
flight_control_run(void)
{
  flight_control_update_command_target();

  pid_out[0] = pid_control_run(&_pidc_roll,   pid_target[0], attitude[0],   1, GCFG->roll_kX);
  pid_out[1] = pid_control_run(&_pidc_pitch,  pid_target[1], attitude[1],   1, GCFG->pitch_kX);
  pid_out[2] = pid_control_run(&_pidc_yaw,    pid_target[2], gyro_body[2],  1, GCFG->yaw_kX);

  flight_control_update_motor_out();
}

static void
flight_control_handle_command(void)
{
  static uint32_t       arm_disarm_start_time;

  switch(flight_state)
  {
  case flight_state_disarmed:
    if(flight_is_arming_position())
    {
      flight_state = flight_state_arming;
      arm_disarm_start_time = __msec;
    }
    break;

  case flight_state_arming:
    if(flight_is_arming_position())
    {
      if((__msec - arm_disarm_start_time) >= 3000U)
      {
        flight_reset();
        flight_state = flight_state_armed;
      }
    }
    else
    {
      flight_state = flight_state_disarmed;
    }
    break;

  case flight_state_armed:
    if(flight_is_arming_position())
    {
      flight_state = flight_state_disarming;
      arm_disarm_start_time = __msec;
    }
    break;

  case flight_state_disarming:
    if(flight_is_arming_position())
    {
      if((__msec - arm_disarm_start_time) >= 3000U)
      {
        flight_reset();
        flight_state = flight_state_disarmed;
      }
    }
    else
    {
      flight_state = flight_state_disarmed;
    }
    break;
  }
}

static void
flight_loop_timer_callback(SoftTimerElem* te)
{
  flight_control_handle_command();

  switch(flight_state)
  {
  case flight_state_armed:
  case flight_state_disarming:    // ???
    flight_control_run();
    break;

  default:
    break;
  }

  mainloop_timer_schedule(&_loop_timer, 1);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
flight_init(void)
{
  flight_state = flight_state_disarmed;

  flight_reset();

  soft_timer_init_elem(&_loop_timer);
  _loop_timer.cb    = flight_loop_timer_callback;

  mainloop_timer_schedule(&_loop_timer, 1);
}
