#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "config.h"

#define CONFIG_START_ADDR         0x080E0000
#define CONFIG_END_ADDR           (0x080E0000 + 128*1024)

config_internal_t    _config =
{
  .cfg = 
  {
    .version            = CONFIG_VERSION,
    .magic              = CONFIG_MAGIC,
    .mag_offset[0]      = 0,
    .mag_offset[1]      = 0,
    .mag_offset[2]      = 0,
    .mag_scale[0]       = 1.0f,
    .mag_scale[1]       = 1.0f,
    .mag_scale[2]       = 1.0f,
    .accel_gain[0]      = 4096,
    .accel_gain[1]      = 4096,
    .accel_gain[2]      = 4096,
    .accel_offset[0]    = 0,
    .accel_offset[1]    = 0,
    .accel_offset[2]    = 0,
    .gyro_offset[0]     = 0,
    .gyro_offset[1]     = 0,
    .gyro_offset[2]     = 0,

    .mag_decl           = 0,

    .roll_kX[0]         = 1.0f,
    .roll_kX[1]         = 1.0f,
    .roll_kX[2]         = 1.0f,
    .pitch_kX[0]        = 1.0f,
    .pitch_kX[1]        = 1.0f,
    .pitch_kX[2]        = 1.0f,
    .yaw_kX[0]          = 1.0f,
    .yaw_kX[1]          = 1.0f,
    .yaw_kX[2]          = 1.0f,

    .roll_max           = 200,        // 20 degree
    .pitch_max          = 200,        // 20 degree
    .yaw_rate_max       = 100,        // 10 degree per sec

    .motor_min          = 1000,
    .motor_max          = 2000,

    .min_flight_throttle  = 1150,
  },
  .crc                  = 0,
};

////////////////////////////////////////////////////////////////////////////////
//
// checksum utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline uint16_t
crc16_ccitt(uint16_t crc, unsigned char a)
{
  crc ^= (uint16_t)a << 8;

  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

static inline uint16_t
calcCRC(uint16_t crc, const void *data, uint32_t length)
{
  const uint8_t *p = (const uint8_t *)data;
  const uint8_t *pend = p + length;

  for (; p != pend; p++) {
    crc = crc16_ccitt(crc, *p);
  }
  return crc;
}

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static bool
is_flash_config_valid(void)
{
  uint16_t    crc;

  config_internal_t* flash_cfg = (config_internal_t*)CONFIG_START_ADDR;

  if(flash_cfg->cfg.magic != CONFIG_MAGIC)
  {
    return false;
  }

  crc = calcCRC(0, (const void*)flash_cfg, sizeof(config_t));

  if(flash_cfg->crc != crc)
    return false;

  return true;
}

static void
erase_program_config_to_flash(void)
{
  FLASH_EraseInitTypeDef    eraseStruct;
  uint32_t                  pageErr;
  uint32_t                  *data_ptr;
  uint32_t                  addr;

  HAL_FLASH_Unlock();

  eraseStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
  eraseStruct.Banks         = FLASH_BANK_1;
  eraseStruct.Sector        = FLASH_SECTOR_11;
  eraseStruct.NbSectors     = 1;
  eraseStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
  if (HAL_FLASHEx_Erase(&eraseStruct, &pageErr) != HAL_OK)
  {
    while(1); // infinite loop to indicate something went wrong
  }

  data_ptr  = (uint32_t*)&_config;
  addr      = CONFIG_START_ADDR;

  while(addr < (CONFIG_START_ADDR + sizeof(config_internal_t)))
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *data_ptr) != HAL_OK)
    {
      while(1); // infinite loop to indicate something went wrong
    }
    addr += 4;
    data_ptr++;
  }
  HAL_FLASH_Lock();
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
config_init(void)
{
  config_internal_t*    flash_cfg = (config_internal_t*)CONFIG_START_ADDR;

  if(is_flash_config_valid() == false) {
    // configuration in flash is not valid
    return;
  }

  //
  // just copy
  //
  memcpy(&_config, flash_cfg, sizeof(config_internal_t));
}

void
config_save(void)
{
  _config.crc = calcCRC(0, (const void*)&(_config.cfg), sizeof(config_t));

  __disable_irq();
  erase_program_config_to_flash();
  __enable_irq();
}
