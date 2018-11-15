#include <math.h>
#include "baro.h"
#include "ms5611.h"
#include "mainloop_timer.h"

#define BARO_PRESSURE_SAMPLE_TIME         10
#define BARO_TEMPERATURE_SAMPLE_TIME      10
#define BARO_CALIBRATE_SAMPLE_COUNT       50*3      // 1 sample per 20ms. 3 sec

typedef enum
{
  baro_op_state_reading_pressure,
  baro_op_state_reading_temperature,
} baro_op_state_t;

////////////////////////////////////////////////////////////////////////////////
//
// privates
//
////////////////////////////////////////////////////////////////////////////////
static ms5611_t           _ms5611;
static baro_op_state_t    _baro_state;
static SoftTimerElem      _sample_timer;
static bool               _cal_in_prog;
static uint32_t           _cal_sample_count;

////////////////////////////////////////////////////////////////////////////////
//
// publics
//
////////////////////////////////////////////////////////////////////////////////
int32_t     baroPressure;                       // in pascal
int32_t     baroTemperature;                    // divide by 100
float       baroGroundPressure = 101325.0f;     // in pascal
float       baroGroundAltitude = 0.0f;          // cm
int32_t     baroAltitude = 0;                   // cm

////////////////////////////////////////////////////////////////////////////////
//
// utilities
//
////////////////////////////////////////////////////////////////////////////////
static float
pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

////////////////////////////////////////////////////////////////////////////////
//
// core
//
////////////////////////////////////////////////////////////////////////////////
static void
baro_calibrate(void)
{
  const float baroGroundPressureError = baroPressure - baroGroundPressure;
  
  baroGroundPressure += baroGroundPressureError * 0.15f;

  if (fabs(baroGroundPressureError) < (baroGroundPressure * 0.00005f))
  {    // 0.005% calibration error (should give c. 10cm calibration error)
    _cal_sample_count++;

    if(_cal_sample_count >= BARO_CALIBRATE_SAMPLE_COUNT)
    {
      baroGroundAltitude = pressureToAltitude(baroGroundPressure);
      _cal_in_prog = false;
    }
  }
}

static void
baro_update(void)
{
  ms5611_calc(&_ms5611, &baroPressure, &baroTemperature);

  if(_cal_in_prog)
  {
    baro_calibrate();
  }
  else
  {
    baroAltitude = pressureToAltitude(baroPressure) - baroGroundAltitude;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// baro sampling
//
////////////////////////////////////////////////////////////////////////////////
static void
baro_sample_timeout(SoftTimerElem* te)
{
  switch(_baro_state)
  {
  case baro_op_state_reading_pressure:
    ms5611_read_pressure(&_ms5611);

    _baro_state = baro_op_state_reading_temperature;

    mainloop_timer_schedule(&_sample_timer, BARO_TEMPERATURE_SAMPLE_TIME);
    ms5611_start_read_temp(&_ms5611);
    break;

  case baro_op_state_reading_temperature:
    ms5611_read_temp(&_ms5611);

    _baro_state = baro_op_state_reading_pressure;
    mainloop_timer_schedule(&_sample_timer, BARO_PRESSURE_SAMPLE_TIME);

    ms5611_start_read_pressure(&_ms5611);

    baro_update();
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
baro_init(void)
{
  _baro_state = baro_op_state_reading_pressure;
  ms5611_init(&_ms5611);

  soft_timer_init_elem(&_sample_timer);
  _sample_timer.cb    = baro_sample_timeout;
}

void
baro_start(void)
{
  _baro_state = baro_op_state_reading_pressure;

  mainloop_timer_schedule(&_sample_timer, BARO_PRESSURE_SAMPLE_TIME);
  ms5611_start_read_pressure(&_ms5611);

  _cal_in_prog = true;
  _cal_sample_count = 0;
}

void
baro_stop(void)
{
  mainloop_timer_cancel(&_sample_timer);
}
