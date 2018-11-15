#ifndef __MS5611_DEF_H__
#define __MS5611_DEF_H__

#include "app_common.h"

#define MS5611_I2C_ADDR         0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

typedef struct
{
  uint32_t    ut;               // temperature
  uint32_t    up;               // pressure
  uint16_t    coef[PROM_NB];    // coefficients

  uint8_t     dev_addr;
} ms5611_t;

extern void ms5611_init(ms5611_t* dev);
extern void ms5611_start_read_pressure(ms5611_t* dev);
extern void ms5611_read_pressure(ms5611_t* dev);
extern void ms5611_start_read_temp(ms5611_t* dev);
extern void ms5611_read_temp(ms5611_t* dev);
extern void ms5611_calc(ms5611_t* dev, int32_t* pressure, int32_t* temperature);

#endif /* !__MS5611_DEF_H__ */
