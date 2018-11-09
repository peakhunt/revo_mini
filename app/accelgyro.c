#include "accelgyro.h"
#include "mpu6000.h"
#include "mainloop_timer.h"

#include "micros.h"

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void accgyro_sample_timer_callback(SoftTimerElem* te);
static void accgyro_gyro_cal_timer_callback(SoftTimerElem* te);
static void accgyro_gyro_cal_update(int16_t gx, int16_t gy, int16_t gz);

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
  if(_gyro_cal_in_prog)
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
