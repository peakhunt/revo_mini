#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "ms5611.h"

#define I2C_DEFAULT_TIMEOUT               1000

static const uint8_t ms56xx_osr = CMD_ADC_4096;
static I2C_HandleTypeDef*   hi2c = &hi2c1;

////////////////////////////////////////////////////////////////////////////////
//
// private read/write
//
////////////////////////////////////////////////////////////////////////////////
static inline void
ms5611_write_reg(ms5611_t* dev, uint8_t reg, uint8_t data)
{
  uint8_t   buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  HAL_I2C_Master_Transmit(hi2c, (dev->dev_addr << 1), buffer, 2, I2C_DEFAULT_TIMEOUT);
}

static inline void
ms5611_read_reg(ms5611_t* dev, uint8_t reg, uint8_t* data, int len)
{
  HAL_I2C_Master_Transmit(hi2c, (dev->dev_addr << 1), &reg, 1, I2C_DEFAULT_TIMEOUT);
  HAL_I2C_Master_Receive(hi2c, (dev->dev_addr << 1), data, len, I2C_DEFAULT_TIMEOUT);
}

static inline uint32_t
ms5611_read_adc(ms5611_t* dev)
{
  uint8_t   rxbuf[3];

  ms5611_read_reg(dev, CMD_ADC_READ, rxbuf, 3);
  return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static void
ms5611_device_init(ms5611_t* dev)
{
  ms5611_write_reg(dev, CMD_RESET, 1);
  
  HAL_Delay(5);

  // read all coefficients
  for (int i = 0; i < PROM_NB; i++)
  {
    uint8_t rxbuf[2] = { 0, 0 };

    ms5611_read_reg(dev, CMD_PROM_RD + i * 2, rxbuf, 2);
    dev->coef[i] = (rxbuf[0] << 8 | rxbuf[1]);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// public services
//
////////////////////////////////////////////////////////////////////////////////
void
ms5611_init(ms5611_t* dev)
{
  dev->dev_addr = MS5611_I2C_ADDR;
  ms5611_device_init(dev);
}

void
ms5611_start_read_pressure(ms5611_t* dev)
{
  ms5611_write_reg(dev, CMD_ADC_CONV + CMD_ADC_D1 + ms56xx_osr, 1);
}

void
ms5611_read_pressure(ms5611_t* dev)
{
  dev->up = ms5611_read_adc(dev);
}

void
ms5611_start_read_temp(ms5611_t* dev)
{
  ms5611_write_reg(dev, CMD_ADC_CONV + CMD_ADC_D2 + ms56xx_osr, 1);
}

void
ms5611_read_temp(ms5611_t* dev)
{
  dev->ut   = ms5611_read_adc(dev);
}

void
ms5611_calc(ms5611_t* dev, int32_t* pressure, int32_t* temperature)
{
  uint32_t press;
  int64_t temp;
  int64_t delt;
  int64_t dT = (int64_t)dev->ut - ((uint64_t)dev->coef[5] * 256);
  int64_t off = ((int64_t)dev->coef[2] << 16) + (((int64_t)dev->coef[4] * dT) >> 7);
  int64_t sens = ((int64_t)dev->coef[1] << 15) + (((int64_t)dev->coef[3] * dT) >> 8);
  temp = 2000 + ((dT * (int64_t)dev->coef[6]) >> 23);

  if (temp < 2000)
  { // temperature lower than 20degC
    delt = temp - 2000;
    delt = 5 * delt * delt;
    off -= delt >> 1;
    sens -= delt >> 2;
    if (temp < -1500)
    { // temperature lower than -15degC
      delt = temp + 1500;
      delt = delt * delt;
      off -= 7 * delt;
      sens -= (11 * delt) >> 1;
    }
    temp -= ((dT * dT) >> 31);
  }
  press = ((((int64_t)dev->up * sens) >> 21) - off) >> 15;

  if (pressure)
    *pressure = press;

  if (temperature)
    *temperature = temp;
}
