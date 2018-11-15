#include <math.h>
#include "magneto.h"
#include "hmc5883.h"
#include "mainloop_timer.h"
#include "sensor_calib.h"
#include "config.h"

#define SAMPLE_INTERVAL   100
#define MAGNETOMETER_CALIBRATE_SAMPLE_COUNT           (10*60)     // 10 samples for 60 seconds

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void mag_sample_timer_callback(SoftTimerElem* te);
static void mag_calib_update(int16_t mx, int16_t my, int16_t mz);

////////////////////////////////////////////////////////////////////////////////
//
// magnetometer main
//
////////////////////////////////////////////////////////////////////////////////
static hmc5883Mag       _mag;
static SoftTimerElem    _sample_timer;
static sensor_align_t   _align;

////////////////////////////////////////////////////////////////////////////////
//
// visible to externals
//
////////////////////////////////////////////////////////////////////////////////
int16_t                 mag_raw[3];
int16_t                 mag_value[3];

////////////////////////////////////////////////////////////////////////////////
//
// calibration related
//
////////////////////////////////////////////////////////////////////////////////
static bool             _mag_calib_in_prog;
#ifndef MAGNETO_CAL_SCALE
static int16_t          _mag_prev[3];
static sensor_calib_t   _cal_state;
#else
static int16_t          _mag_min[3],
                        _mag_max[3];
#endif
static uint32_t         _mag_cal_sample_count;

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
  hmc5883_read(&_mag, mag_raw);

#ifndef MAGNETO_CAL_SCALE
  mag_value[0] = mag_raw[0] - GCFG->mag_offset[0];
  mag_value[1] = mag_raw[1] - GCFG->mag_offset[1];
  mag_value[2] = mag_raw[2] - GCFG->mag_offset[2];
#else
  mag_value[0] = (mag_raw[0] - GCFG->mag_offset[0]) * GCFG->mag_scale[0];
  mag_value[1] = (mag_raw[1] - GCFG->mag_offset[1]) * GCFG->mag_scale[1];
  mag_value[2] = (mag_raw[2] - GCFG->mag_offset[2]) * GCFG->mag_scale[2];
#endif

  sensor_align_values(mag_value, _align);

  mainloop_timer_schedule(&_sample_timer, SAMPLE_INTERVAL);

  if(_mag_calib_in_prog)
  {
    mag_calib_update(mag_raw[0], mag_raw[1], mag_raw[2]);
  }
}

void
magneto_init(sensor_align_t align)
{
  _align = align;

  hmc5883_init(&_mag, HMC5883_ADDRESS_MAG, HMC5883_MAGGAIN_1_3);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb = mag_sample_timer_callback;

  _mag_calib_in_prog = false;
}

void
magneto_start(void)
{
  mainloop_timer_schedule(&_sample_timer, SAMPLE_INTERVAL);
}

void
magneto_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
}

////////////////////////////////////////////////////////////////////////////////
//
// calibration related functions
//
////////////////////////////////////////////////////////////////////////////////
static void
mag_calib_update(int16_t mx, int16_t my, int16_t mz)
{
#ifndef MAGNETO_CAL_SCALE
  float     diffMag = 0;
  float     avgMag = 0;
  int32_t   mag_data[3];

  _mag_cal_sample_count++;

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

  if(_mag_cal_sample_count > MAGNETOMETER_CALIBRATE_SAMPLE_COUNT)
  {
    float     magZerof[3];

    sensorCalibrationSolveForOffset(&_cal_state, magZerof);

    for (int axis = 0; axis < 3; axis++)
    {
      GCFG->mag_offset[axis] = lrintf(magZerof[axis]);
    }

    _mag_calib_in_prog = false;

    _cb(GCFG->mag_offset, _cb_arg);
  }
#else
  int16_t   m[3];

  _mag_cal_sample_count++;

  m[0] = mx;
  m[1] = my;
  m[2] = mz;

  for(int i = 0; i < 3; i++)
  {
    if(m[i] < _mag_min[i])
    {
      _mag_min[i] = m[i];
    }

    if(m[i] > _mag_max[i])
    {
      _mag_max[i] = m[i];
    }
  }

  if(_mag_cal_sample_count > MAGNETOMETER_CALIBRATE_SAMPLE_COUNT)
  {
    int16_t   diff,
              off;

    for(int i = 0; i < 3; i++)
    {
      diff = _mag_max[i] - _mag_min[i];
      off  = _mag_max[i] - diff / 2;

      GCFG->mag_offset[i] = off;
      GCFG->mag_scale[i] = 1000.0f / (diff/2.0f);
    }
    
    _mag_calib_in_prog = false;

    _cb(GCFG->mag_offset, _cb_arg);
  }
#endif
}

bool
magneto_calibrate(magneto_calibrate_callback cb, void* cb_arg)
{
  if(_mag_calib_in_prog)
  {
    return false;
  }

  _cb     = cb;
  _cb_arg = cb_arg;

#ifndef MAGNETO_CAL_SCALE
  _mag_prev[0] = 
  _mag_prev[1] = 
  _mag_prev[2] =  0;

  sensorCalibrationResetState(&_cal_state);
#else
  _mag_min[0] = _mag_min[1] = _mag_min[2] = 32764;
  _mag_max[0] = _mag_max[1] = _mag_max[2] =-32763;
#endif

  _mag_calib_in_prog = true;
  _mag_cal_sample_count = 0;

  return true;
}
