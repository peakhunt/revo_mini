#include <string.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "app_common.h"
#include "hmc5883.h"

////////////////////////////////////////////////////////////////////////////////
//
// internal defines
//
////////////////////////////////////////////////////////////////////////////////
#define SENSORS_GAUSS_TO_MICROTESLA       (100)  
#define I2C_DEFAULT_TIMEOUT               100

static I2C_HandleTypeDef*   hi2c = &hi2c1;

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
hmc5883_write_reg(hmc5883Mag* mag, uint8_t reg, uint8_t data)
{
  uint8_t   buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  HAL_I2C_Master_Transmit(hi2c, (mag->address << 1), buffer, 2, I2C_DEFAULT_TIMEOUT);
}

static inline void
hmc5883_read_reg(hmc5883Mag* mag, uint8_t reg, uint8_t* data, uint8_t len)
{
  HAL_I2C_Master_Transmit(hi2c, (mag->address << 1), &reg, 1, I2C_DEFAULT_TIMEOUT);
  HAL_I2C_Master_Receive(hi2c, (mag->address << 1), data, len, I2C_DEFAULT_TIMEOUT);
}


////////////////////////////////////////////////////////////////////////////////
//
// public utilities
//
////////////////////////////////////////////////////////////////////////////////
void
hmc5883_init(hmc5883Mag* mag, uint8_t address, hmc5883MagGain gain)
{
  mag->address    = address;

  //
  // enable magnetometer
  //
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);
  hmc5883_set_mag_gain(mag, gain);
}

void
hmc5883_set_mag_gain(hmc5883Mag* mag, hmc5883MagGain gain)
{
  //
  // cra setup
  // 7    : 0       , no temperature sensor
  // 6/5  : 00,     , 1 sample per measure
  // 4/2  : 111     , 220 Hz output rate
  // 1/0  : 00      , normal
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_CRA_REG_M, 0x1c);

  //
  // crb setup
  // gain 
  // hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_CRB_REG_M, gain);

  // mode register
  // continuous measure mode
  hmc5883_write_reg(mag, HMC5883_REGISTER_MAG_MR_REG_M, 0);
}

void
hmc5883_read(hmc5883Mag* mag, int16_t m[3])
{
  uint8_t   data[6];

  hmc5883_read_reg(mag, HMC5883_REGISTER_MAG_OUT_X_H_M, data, 6);

  m[0] = (int16_t)(data[1] | ((int16_t)(data[0] << 8)));
  m[2] = (int16_t)(data[3] | ((int16_t)(data[2] << 8)));
  m[1] = (int16_t)(data[5] | ((int16_t)(data[4] << 8)));
}
