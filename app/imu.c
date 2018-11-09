#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "imu.h"
#include "accelgyro.h"
#include "magneto.h"
#include "mainloop_timer.h"

#define IMU_SAMPLE_FREQ       1000

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static imu_t            _imu;
static SoftTimerElem    _sample_timer;
int16_t                 attitude[3];

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
static void
imu_run(imu_t* imu)
{
  // very weird
  //
  // it looks like this is the required coord convertion
  // for Madgwick. but I am so confused.
  ///
  // for accel
  //     +x is up
  //     +y is left
  //     +z is up
  // for magnetometer
  //    +x is up
  //    +y is left
  //    +z is up
  // for gyro
  //    +x is left roll
  //    +y is down pitch
  //    +z is compass rotation
  //
  madgwick_update(&imu->filter,
      -gyro_dps[1],   -gyro_dps[0],   -gyro_dps[2],
      accel_value[1], accel_value[0], accel_value[2],
      mag_value[1],   mag_value[0],   mag_value[2]);

  madgwick_get_roll_pitch_yaw(&imu->filter,
      attitude, 0.0f);
}

static void
imu_sample_timer_callback(SoftTimerElem* te)
{
  imu_run(&_imu);

  mainloop_timer_schedule(&_sample_timer, 1);
}

////////////////////////////////////////////////////////////////////////////////
//
// module publics
//
////////////////////////////////////////////////////////////////////////////////
void
imu_init(void)
{
  memset(&_imu, 0, sizeof(_imu));

  madgwick_init(&_imu.filter, IMU_SAMPLE_FREQ);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb = imu_sample_timer_callback;

  mainloop_timer_schedule(&_sample_timer, 1);
}

imu_t*
imu_get(void)
{
  return &_imu;
}
