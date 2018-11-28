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
// flight core
//
////////////////////////////////////////////////////////////////////////////////
static void
flight_control_update_command_target(void)
{
  // roll   1000~2000 ->  +- roll_max
  pid_target[0] = lerp(rx_cmd[RX_CMD_ROLL],  RX_CMD_MIN, RX_CMD_MAX, -GCFG->roll_max, GCFG->roll_max);

  // pitch  1000-2000 ->  +- pitch_max
  pid_target[1] = lerp(rx_cmd[RX_CMD_PITCH], RX_CMD_MIN, RX_CMD_MAX, -GCFG->pitch_max, GCFG->pitch_max);

  // yaw    1000-2000 ->  +- yaw_rate_max
  pid_target[2] = lerp(rx_cmd[RX_CMD_YAW], RX_CMD_MIN, RX_CMD_MAX, -GCFG->yaw_rate_max, GCFG->yaw_rate_max);
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
  float   throttle  = rx_cmd[RX_CMD_THROTTLE],
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

  for(int i = 0; i < 4; i++)
  {
    pid_motor[i] = (uint16_t)m[i];
    pwm_set_duty(i, pid_motor[i]);
  }
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
flight_loop_timer_callback(SoftTimerElem* te)
{
  flight_control_run();

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

  pid_control_init(&_pidc_roll);
  pid_control_init(&_pidc_pitch);
  pid_control_init(&_pidc_yaw);

  soft_timer_init_elem(&_loop_timer);
  _loop_timer.cb    = flight_loop_timer_callback;

  mainloop_timer_schedule(&_loop_timer, 1);
}
