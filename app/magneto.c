#include <math.h>
#include "magneto.h"
#include "hmc5883.h"
#include "mainloop_timer.h"
#include "sensor_calib.h"

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void mag_sample_timer_callback(SoftTimerElem* te);
static void mag_calib_timer_callback(SoftTimerElem* te);
static void mag_calib_update(int16_t mx, int16_t my, int16_t mz);

////////////////////////////////////////////////////////////////////////////////
//
// magnetometer main
//
////////////////////////////////////////////////////////////////////////////////
static hmc5883Mag       _mag;
static SoftTimerElem    _sample_timer;

////////////////////////////////////////////////////////////////////////////////
//
// calibration related
//
////////////////////////////////////////////////////////////////////////////////
static bool             _mag_calib_in_prog;
static int16_t          _mag_prev[3];
static sensor_calib_t   _cal_state;
static SoftTimerElem    _calib_timer;

static magneto_calibrate_callback   _cb;
static void*                        _cb_arg;

////////////////////////////////////////////////////////////////////////////////
//
// magnetometer related functions
//
////////////////////////////////////////////////////////////////////////////////
static void
mag_sample_timer_callback(SoftTimerElem* te)
{
  hmc5883_read(&_mag);
  mainloop_timer_schedule(&_sample_timer, 100);

  if(_mag_calib_in_prog)
  {
    mag_calib_update(_mag.rx, _mag.ry, _mag.rz);
  }
}

void
magneto_init(void)
{
  hmc5883_init(&_mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_4_0);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb = mag_sample_timer_callback;

  _mag_calib_in_prog = false;
}

void
magneto_start(void)
{
  mainloop_timer_schedule(&_sample_timer, 100);
}

void
magneto_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
}

void
magneto_get(int16_t mag[3])
{
  mag[0] = _mag.rx;
  mag[1] = _mag.ry;
  mag[2] = _mag.rz;
}

////////////////////////////////////////////////////////////////////////////////
//
// calibration related functions
//
////////////////////////////////////////////////////////////////////////////////
static void
mag_calib_timer_callback(SoftTimerElem* te)
{
  float magZerof[3];
  int16_t   offsets[3];

  sensorCalibrationSolveForOffset(&_cal_state, magZerof);

  for (int axis = 0; axis < 3; axis++)
  {
    offsets[axis] = lrintf(magZerof[axis]);
  }

  _mag_calib_in_prog = false;

  _cb(offsets, _cb_arg);
}

static void
mag_calib_update(int16_t mx, int16_t my, int16_t mz)
{
  float     diffMag = 0;
  float     avgMag = 0;
  int32_t   mag_data[3];

  mag_data[0] = mx;
  mag_data[1] = my;
  mag_data[2] = mz;

  for (int axis = 0; axis < 3; axis++) {
    diffMag += (mag_data[axis] - _mag_prev[axis]) * (mag_data[axis] - _mag_prev[axis]);
    avgMag += (mag_data[axis] + _mag_prev[axis]) * (mag_data[axis] + _mag_prev[axis]) / 4.0f;
  }

  // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and _mag_prev. tan(8 deg) = 0.14
  if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f))) {
    sensorCalibrationPushSampleForOffsetCalculation(&_cal_state, mag_data);

    for (int axis = 0; axis < 3; axis++) {
      _mag_prev[axis] = mag_data[axis];
    }
  }
}

bool
magneto_calibrate(magneto_calibrate_callback cb, void* cb_arg)
{
  if(_mag_calib_in_prog)
  {
    return false;
  }

  soft_timer_init_elem(&_calib_timer);
  _calib_timer.cb = mag_calib_timer_callback;

  _cb     = cb;
  _cb_arg = cb_arg;

  _mag_prev[0] = 
  _mag_prev[1] = 
  _mag_prev[2] =  0;

  sensorCalibrationResetState(&_cal_state);

  _mag_calib_in_prog = true;
  mainloop_timer_schedule(&_calib_timer, 60 * 1000);   // 1 minute timer

  return true;
}
