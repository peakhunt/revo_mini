#include <math.h>
#include <stdlib.h>
#include "accelgyro.h"
#include "mpu6000.h"
#include "mainloop_timer.h"

#include "micros.h"
#include "sensor_calib.h"

//
// accelerometer is seto to +- 8G. 
// so 1G is 4096
//
#define ACCELGYRO_1G_VALUE          4096

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void accgyro_sample_timer_callback(SoftTimerElem* te);
static void accgyro_gyro_cal_timer_callback(SoftTimerElem* te);
static void accgyro_gyro_cal_update(int16_t gx, int16_t gy, int16_t gz);
static void accgyro_accel_cal_update(int16_t ax, int16_t ay, int16_t az);

////////////////////////////////////////////////////////////////////////////////
//
// accel/gyro main
//
////////////////////////////////////////////////////////////////////////////////
static MPU6000_t        _mpu;
static SoftTimerElem    _sample_timer;

static uint16_t         _sample_rate;
static uint16_t         _sample_count;

static uint32_t last_msec;

////////////////////////////////////////////////////////////////////////////////
//
// gyro calibration related
//
////////////////////////////////////////////////////////////////////////////////
static SoftTimerElem    _gyro_cal_timer;
static bool             _gyro_cal_in_prog;
static float            _gyro_sum[3];
static int32_t          _gyro_cal_sum_count;

static accelgyro_gyro_calib_callback    _gyro_cal_cb;
static void*  _gyro_cal_cb_arg;

////////////////////////////////////////////////////////////////////////////////
//
// accel calibration related
//
////////////////////////////////////////////////////////////////////////////////
#define ACCELGYRO_ACCEL_CALIBRATE_SAMPLE_COUNT        8000

static bool             _accel_cal_in_prog;
static accelgyro_accel_calib_step_callback  _accel_cal_step_cb;
static accelgyro_accel_calib_done_callback  _accel_cal_done_cb;
static void*  _accel_cal_cb_arg;

static int32_t    _accel_cal_sample_count[6];
static int32_t    _accel_cal_sum[6][3];
static bool       _accel_cal_done[6];

////////////////////////////////////////////////////////////////////////////////
//
// accel/gyro main functions
//
////////////////////////////////////////////////////////////////////////////////
static void
accgyro_sample_timer_callback(SoftTimerElem* te)
{
  mpu6000_read_all(&_mpu);
  mainloop_timer_schedule(&_sample_timer, 1);
  _sample_count++;

  if(_gyro_cal_in_prog)
  {
    accgyro_gyro_cal_update(_mpu.Gyroscope_X, _mpu.Gyroscope_Y, _mpu.Gyroscope_Z);
  }

  if(_accel_cal_in_prog)
  {
    accgyro_accel_cal_update(_mpu.Accelerometer_X, _mpu.Accelerometer_Y, _mpu.Accelerometer_Z);
  }

  if((__msec - last_msec) >= 1000)
  {
    _sample_rate = _sample_count;
    _sample_count = 0;
    last_msec = __msec;
  }
}

void
accelgyro_init(void)
{
  mpu6000_init(&_mpu);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb    = accgyro_sample_timer_callback;

  _gyro_cal_in_prog   = false;
  _accel_cal_in_prog  = false;
}

void
accelgyro_start(void)
{
  mainloop_timer_schedule(&_sample_timer, 1);

  _sample_count = 0;
  _sample_rate = 0;
  last_msec = __msec;
}

void
accelgyro_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
}

void
accelgyro_get(int16_t accel[3], int16_t gyro[3])
{
  accel[0] = _mpu.Accelerometer_X;
  accel[1] = _mpu.Accelerometer_Y;
  accel[2] = _mpu.Accelerometer_Z;

  gyro[0] = _mpu.Gyroscope_X;
  gyro[1] = _mpu.Gyroscope_Y;
  gyro[2] = _mpu.Gyroscope_Z;
}

uint16_t
accelgyro_sample_rate(void)
{
  return _sample_rate;
}

////////////////////////////////////////////////////////////////////////////////
//
// gyro calibration functions
//
////////////////////////////////////////////////////////////////////////////////
static void
accgyro_gyro_cal_timer_callback(SoftTimerElem* te)
{
  int16_t   offset[3];

  offset[0] = (int16_t)(_gyro_sum[0] / _gyro_cal_sum_count);
  offset[1] = (int16_t)(_gyro_sum[1] / _gyro_cal_sum_count);
  offset[2] = (int16_t)(_gyro_sum[2] / _gyro_cal_sum_count);

  _gyro_cal_in_prog = false;

  _gyro_cal_cb(offset, _gyro_cal_cb_arg);
}

static void
accgyro_gyro_cal_update(int16_t gx, int16_t gy, int16_t gz)
{
  _gyro_sum[0] += gx;
  _gyro_sum[1] += gy;
  _gyro_sum[2] += gz;

  _gyro_cal_sum_count++;
}

bool
accelgyro_gyro_calibrate(accelgyro_gyro_calib_callback cb, void* cb_arg)
{
  if(_gyro_cal_in_prog || _accel_cal_in_prog)
  {
    return false;
  }

  _gyro_cal_cb      = cb;
  _gyro_cal_cb_arg  = cb_arg;

  soft_timer_init_elem(&_gyro_cal_timer);
  _gyro_cal_timer.cb    = accgyro_gyro_cal_timer_callback;

  _gyro_sum[0]    = 
  _gyro_sum[1]    = 
  _gyro_sum[2]    = 0.0f;

  _gyro_cal_sum_count = 0;
  _gyro_cal_in_prog = true;

  mainloop_timer_schedule(&_gyro_cal_timer, ACCELGYRO_GYRO_CALIBRATION_TIMEOUT * 1000);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
//
// accel calibration functions
//
////////////////////////////////////////////////////////////////////////////////
static int
getPrimaryAxisIndex(const int16_t ax, const int16_t ay, const int16_t az)
{
  int32_t   sample[3];

  sample[X] = ax;
  sample[Y] = ay;
  sample[Z] = az;

  // FIXME sensor alignment

  // tolerate up to atan(1 / 1.5) = 33 deg tilt (in worst case 66 deg separation between points)
  if ((abs(sample[Z]) / 1.5f) > abs(sample[X]) && (abs(sample[Z]) / 1.5f) > abs(sample[Y]))
  {
    //z-axis
    return (sample[Z] > 0) ? 0 : 1;
  }
  else if ((abs(sample[X]) / 1.5f) > abs(sample[Y]) && (abs(sample[X]) / 1.5f) > abs(sample[Z]))
  {
    //x-axis
    return (sample[X] > 0) ? 2 : 3;
  }
  else if ((abs(sample[Y]) / 1.5f) > abs(sample[X]) && (abs(sample[Y]) / 1.5f) > abs(sample[Z]))
  {
    //y-axis
    return (sample[Y] > 0) ? 4 : 5;
  }

  return -1;
}

static bool
accgyro_is_accel_cal_done(void)
{
  for(int i = 0; i < 6; i++)
  {
    if(_accel_cal_done[i] == false)
    {
      return false;
    }
  }
  return true;
}

static void
accgyro_accel_cal_finish(void)
{
  sensor_calib_t    cal_state;
  float             tmp[3];
  int16_t           offset[3],
                    gain[3];

  // offset 
  sensorCalibrationResetState(&cal_state);
  for (int axis = 0; axis < 6; axis++) 
  {
    sensorCalibrationPushSampleForOffsetCalculation(&cal_state, _accel_cal_sum[axis]);
  }
  sensorCalibrationSolveForOffset(&cal_state, tmp);

  offset[X] = lrintf(tmp[X]);
  offset[Y] = lrintf(tmp[Y]);
  offset[Z] = lrintf(tmp[Z]);

  // gain
  sensorCalibrationResetState(&cal_state);

  for (int axis = 0; axis < 6; axis++)
  {
    int32_t samples[3];

    samples[X] = _accel_cal_sum[axis][X] - offset[X];
    samples[Y] = _accel_cal_sum[axis][Y] - offset[Y];
    samples[Z] = _accel_cal_sum[axis][Z] - offset[Z];

    //
    // accelerometer is seto to +- 8G. 
    // so 1G is 4096
    //
    sensorCalibrationPushSampleForScaleCalculation(&cal_state, axis / 2, samples, ACCELGYRO_1G_VALUE);
  }
  sensorCalibrationSolveForScale(&cal_state, tmp);

  for (int axis = 0; axis < 3; axis++) 
  {
    gain[axis] = lrintf(tmp[axis] * ACCELGYRO_1G_VALUE);
  }

  _accel_cal_in_prog = false;
  _accel_cal_done_cb(offset, gain, _accel_cal_cb_arg);
}

static void
accgyro_accel_cal_update(int16_t ax, int16_t ay, int16_t az)
{
  int32_t index = getPrimaryAxisIndex(ax, ay, az);

  if(index < 0)
  {
    return;
  }

  if(accgyro_is_accel_cal_done())
  {
    accgyro_accel_cal_finish();
    return;
  }

  if(_accel_cal_done[index])
  {
    return;
  }

  _accel_cal_sum[index][X] += ax;
  _accel_cal_sum[index][Y] += ay;
  _accel_cal_sum[index][Z] += az;
  _accel_cal_sample_count[index]++;

  if(_accel_cal_sample_count[index] > ACCELGYRO_ACCEL_CALIBRATE_SAMPLE_COUNT)
  {
    _accel_cal_done[index] = true;

    _accel_cal_sum[index][X] /= ACCELGYRO_ACCEL_CALIBRATE_SAMPLE_COUNT;
    _accel_cal_sum[index][Y] /= ACCELGYRO_ACCEL_CALIBRATE_SAMPLE_COUNT;
    _accel_cal_sum[index][Z] /= ACCELGYRO_ACCEL_CALIBRATE_SAMPLE_COUNT;

    _accel_cal_step_cb(index, _accel_cal_cb_arg);
  }
}

bool
accelgyro_accel_calibrate( accelgyro_accel_calib_step_callback step_cb,
    accelgyro_accel_calib_done_callback done_cb,
    void* cb_arg)
{
  if(_gyro_cal_in_prog || _accel_cal_in_prog)
  {
    return false;
  }

  _accel_cal_step_cb = step_cb;
  _accel_cal_done_cb = done_cb;
  _accel_cal_cb_arg = cb_arg;

  _accel_cal_in_prog = true;

  for(int i = 0; i < 6; i++)
  {
    _accel_cal_sample_count[i] = 0;
    _accel_cal_sum[i][X] = 0;
    _accel_cal_sum[i][Y] = 0;
    _accel_cal_sum[i][Z] = 0;
    _accel_cal_done[i] = false;
  }

  return true;
}