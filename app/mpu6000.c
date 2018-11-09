#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include "app_common.h"
#include "mpu6000.h"

#define BIT_H_RESET                 0x80
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define BIT_I2C_IF_DIS              0x10
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

static SPI_HandleTypeDef* hspi = &hspi1;

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
mpu6000_write_reg(MPU6000_t* mpu, uint8_t reg, uint8_t data)
{
  uint8_t buffer[2];

  buffer[0] = reg;
  buffer[1] = data;

  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, buffer, 2, 1000);
  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_SET);
}

static inline void
mpu6000_write_reg16(MPU6000_t* mpu, uint8_t reg, uint16_t data)
{
  uint8_t buffer[3];

  buffer[0] = reg;
  buffer[1] = (data >> 8 ) & 0xff;
  buffer[2] = data & 0xff;

  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, buffer, 3, 1000);
  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_SET);
}

static inline uint8_t
mpu6000_read_reg(MPU6000_t* mpu, uint8_t reg)
{
  uint8_t    ret;

  reg |= 0x80;

  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &reg, 1, 1000);
  HAL_SPI_Receive(hspi, &ret, 1, 1000);
  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_SET);

  return ret;
}

static inline void
mpu6000_read_data(MPU6000_t* mpu, uint8_t reg, uint8_t* data, uint8_t len)
{
  reg |= 0x80;

  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &reg, 1, 1000);
  HAL_SPI_Receive(hspi, data, len, 1000);
  HAL_GPIO_WritePin(MPU6000_SS_GPIO_Port, MPU6000_SS_Pin, GPIO_PIN_SET);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
mpu6000_init(MPU6000_t* mpu)
{
  // bus device reset
  mpu6000_write_reg(mpu, MPU6000_PWR_MGMT_1, BIT_H_RESET);
  HAL_Delay(150);

  mpu6000_write_reg(mpu, MPU6000_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  HAL_Delay(150);

  // Clock Source PPL with Z axis gyro reference
  mpu6000_write_reg(mpu, MPU6000_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  HAL_Delay(1);

  // Disable Primary I2C Interface
  mpu6000_write_reg(mpu, MPU6000_USER_CTRL, BIT_I2C_IF_DIS);
  HAL_Delay(1);

  mpu6000_write_reg(mpu, MPU6000_PWR_MGMT_2, 0x00);
  HAL_Delay(1);

  //
  // accel rate is always 1K Hz
  // gyro rate is set to 1K Hz with DPLF is enabled
  // target LPF
  // for accel : 184Hz
  // for gyro  : 188Hz
  //
  // EXT SYNC disabled
  //
  mpu6000_write_reg(mpu, MPU6000_SMPLRT_DIV, 0);
  mpu6000_write_reg(mpu, MPU6000_CONFIG, (0x0 << 3 | 0x1));
  HAL_Delay(1);

  // accelerometer range
  // +- 8G scale
  mpu6000_write_reg(mpu, MPU6000_ACCEL_CONFIG, (0x02 << 3));
  HAL_Delay(1);

  // gyro range
  // +- 1000 degrees per sec
  mpu6000_write_reg(mpu, MPU6000_GYRO_CONFIG, (0x02 << 3));
  HAL_Delay(1);
}

void
mpu6000_read_all(MPU6000_t* mpu, int16_t a[3], int16_t g[3])
{
  uint8_t data[14];
  int16_t temp;

  /* Read full raw data, 14bytes */
  mpu6000_read_data(mpu, MPU6000_ACCEL_XOUT_H, data, 14);

  /* Format accelerometer data */
  a[0] = (int16_t)(data[0] << 8 | data[1]);
  a[1] = (int16_t)(data[2] << 8 | data[3]);
  a[2] = (int16_t)(data[4] << 8 | data[5]);

  /* Format temperature */
  temp = (data[6] << 8 | data[7]);
  mpu->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

  /* Format gyroscope data */
  g[0] = (int16_t)(data[8] << 8 | data[9]);
  g[1] = (int16_t)(data[10] << 8 | data[11]);
  g[2] = (int16_t)(data[12] << 8 | data[13]);
}

uint8_t
mpu6000_test(MPU6000_t* mpu, uint8_t reg)
{
  return mpu6000_read_reg(mpu, reg);
}
