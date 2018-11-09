#ifndef __HMC5883_DEF_H__
#define __HMC5883_DEF_H__

#define HMC5883_ADDRESS_MAG            (0x1e)
//#define HMC5883_ADDRESS_MAG            (0x1c)
//#define HMC5883_ADDRESS_MAG            (30)

typedef enum
{
  HMC5883_REGISTER_MAG_CRA_REG_M             = 0x00,
  HMC5883_REGISTER_MAG_CRB_REG_M             = 0x01,
  HMC5883_REGISTER_MAG_MR_REG_M              = 0x02,
  HMC5883_REGISTER_MAG_OUT_X_H_M             = 0x03,
  HMC5883_REGISTER_MAG_OUT_X_L_M             = 0x04,
  HMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
  HMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x06,
  HMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x07,
  HMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x08,
  HMC5883_REGISTER_MAG_SR_REG_Mg             = 0x09,
  HMC5883_REGISTER_MAG_IRA_REG_M             = 0x0A,
  HMC5883_REGISTER_MAG_IRB_REG_M             = 0x0B,
  HMC5883_REGISTER_MAG_IRC_REG_M             = 0x0C,
  HMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
  HMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} hmc5883MagRegisters_t;

typedef enum
{
  HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
  HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
  HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
  HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
  HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
  HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
  HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} hmc5883MagGain;  

typedef struct hmc5883Mag_s
{
  uint8_t             address;
} hmc5883Mag;

extern void hmc5883_init(hmc5883Mag* mag, uint8_t address, hmc5883MagGain gain);
extern void hmc5883_set_mag_gain(hmc5883Mag* mag, hmc5883MagGain gain);
extern void hmc5883_read(hmc5883Mag* mag, int16_t m[3]);

#endif //!__HMC5883_DEF_H__
