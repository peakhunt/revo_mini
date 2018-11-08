#include "accelgyro.h"
#include "mpu6000.h"
#include "mainloop_timer.h"

static MPU6000_t        _mpu;
static SoftTimerElem    _sample_timer;
static SoftTimerElem    _one_sec_timer;

static uint16_t         _sample_rate;
static uint16_t         _sample_count;

static void
sample_timer_callback(SoftTimerElem* te)
{
  mpu6000_read_all(&_mpu);
  mainloop_timer_schedule(&_sample_timer, 1);
  _sample_count++;
}

static void
one_sec_timer_callback(SoftTimerElem* te)
{
  mainloop_timer_schedule(&_one_sec_timer, 1000);
  _sample_rate  = _sample_count;
  _sample_count = 0;
}

void
accelgyro_init(void)
{
  mpu6000_init(&_mpu);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb    = sample_timer_callback;

  soft_timer_init_elem(&_one_sec_timer);
  _one_sec_timer.cb   = one_sec_timer_callback;
}

void
accelgyro_start(void)
{
  mainloop_timer_schedule(&_sample_timer, 1);
  mainloop_timer_schedule(&_one_sec_timer, 1000);
}

void
accelgyro_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
  mainloop_timer_cancel(&_one_sec_timer);
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
