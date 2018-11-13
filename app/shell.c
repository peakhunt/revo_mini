#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

#include "app_common.h"
#include "shell.h"
#include "shell_if_usb.h"
#include "pwm.h"
#include "mpu6000.h"
#include "accelgyro.h"
#include "magneto.h"
#include "micros.h"
#include "imu.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define SHELL_MAX_COLUMNS_PER_LINE      128
#define SHELL_COMMAND_MAX_ARGS          4

#define VERSION       "STM32F4 Shell V0.3a"

typedef void (*shell_command_handler)(ShellIntf* intf, int argc, const char** argv);

typedef struct
{
  const char*           command;
  const char*           description;
  shell_command_handler handler;
} ShellCommand;

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void shell_command_help(ShellIntf* intf, int argc, const char** argv);
static void shell_command_version(ShellIntf* intf, int argc, const char** argv);
static void shell_command_uptime(ShellIntf* intf, int argc, const char** argv);
static void shell_command_pwm(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mpu(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mpu_raw(ShellIntf* intf, int argc, const char** argv);
static void shell_command_micros(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_raw(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag_cal(ShellIntf* intf, int argc, const char** argv);
static void shell_command_gyro_cal(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel_cal(ShellIntf* intf, int argc, const char** argv);
static void shell_command_mag(ShellIntf* intf, int argc, const char** argv);
static void shell_command_gyro(ShellIntf* intf, int argc, const char** argv);
static void shell_command_accel(ShellIntf* intf, int argc, const char** argv);
static void shell_command_attitude(ShellIntf* intf, int argc, const char** argv);
static void shell_command_cal_show(ShellIntf* intf, int argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
const uint8_t                 _welcome[] = "\r\n**** Welcome ****\r\n";
const uint8_t                 _prompt[]  = "\r\nSTM32F4> ";

static char                   _print_buffer[SHELL_MAX_COLUMNS_PER_LINE + 1];

static LIST_HEAD_DECL(_shell_intf_list);

static ShellCommand     _commands[] = 
{
  {
    "help",
    "show this command",
    shell_command_help,
  },
  {
    "version",
    "show version",
    shell_command_version,
  },
  {
    "uptime",
    "show system uptime",
    shell_command_uptime,
  },
  {
    "pwm",
    "set pwm out duty cycle",
    shell_command_pwm,
  },
  {
    "mpu",
    "mpu6000 related commands",
    shell_command_mpu,
  },
  {
    "mpu_raw",
    "read raw MPU values",
    shell_command_mpu_raw,
  },
  {
    "micros",
    "read current micros",
    shell_command_micros,
  },
  {
    "mag_raw",
    "read raw MAG values",
    shell_command_mag_raw,
  },
  {
    "mag_cal",
    "calibrate magnetometer",
    shell_command_mag_cal,
  },
  {
    "gyro_cal",
    "calibrate gyro",
    shell_command_gyro_cal,
  },
  {
    "accel_cal",
    "calibrate accelerometer",
    shell_command_accel_cal,
  },
  {
    "gyro",
    "show gyro value",
    shell_command_gyro,
  },
  {
    "accel",
    "show accelerometer value",
    shell_command_accel,
  },
  {
    "mag",
    "show magnetometer value",
    shell_command_mag,
  },
  {
    "attitude",
    "show attitude",
    shell_command_attitude,
  },
  {
    "cal_show",
    "show calibration values",
    shell_command_cal_show,
  }
};

////////////////////////////////////////////////////////////////////////////////
//
// shell utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
shell_prompt(ShellIntf* intf)
{
  intf->put_tx_data(intf, (uint8_t*)_prompt, sizeof(_prompt) -1);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell command handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_help(ShellIntf* intf, int argc, const char** argv)
{
  size_t i;

  shell_printf(intf, "\r\n");

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    shell_printf(intf, "%-20s: ", _commands[i].command);
    shell_printf(intf, "%s\r\n", _commands[i].description);
  }
}

static void
shell_command_version(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "%s\r\n", VERSION);
}

static void
shell_command_uptime(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "System Uptime: %lu\r\n", __uptime);
}

static void
shell_command_pwm(ShellIntf* intf, int argc, const char** argv)
{
  static const pwm_channel_enum_t   chnl_map[] = 
  {
    pwm_channel_0,
    pwm_channel_1,
    pwm_channel_2,
    pwm_channel_3,
    pwm_channel_4,
    pwm_channel_5,
  };
  uint16_t    duty;
  uint8_t     i;

  if(argc != 3)
  {
    shell_printf(intf, "Syntax error %s chnl duty\r\n", argv[0]);
    return;
  }

  i = atoi(argv[1]);
  duty = atoi(argv[2]);

  if(i >= sizeof(chnl_map)/sizeof(pwm_channel_enum_t))
  {
    shell_printf(intf, "invalid channel number %s\r\n", argv[1]);
    return;
  }

  if(duty < PWM_OUT_MIN_DUTY_CYCLE || duty > PWM_OUT_MAX_DUTY_CYCLE)
  {
    shell_printf(intf, "invalid duty cycle %s\r\n", argv[2]);
    return;
  }

  pwm_set_duty(chnl_map[i], duty);
  shell_printf(intf, "set duty cycle to %u for channel %u\r\n", duty, i);
}

static void
shell_command_mpu(ShellIntf* intf, int argc, const char** argv)
{
  uint8_t reg,
          ret;

  if(argc != 2)
  {
    shell_printf(intf, "Syntax error %s <reg>\r\n", argv[0]);
    return;
  }

  reg = atoi(argv[1]);

  ret = mpu6000_test(NULL, reg);

  shell_printf(intf, "\r\n");
  shell_printf(intf, "reg 0x%x: 0x%x\r\n", reg, ret);
}

static void
shell_command_mpu_raw(ShellIntf* intf, int argc, const char** argv)
{
  uint16_t  sample_rate;

  sample_rate = accelgyro_sample_rate();

  shell_printf(intf, "\r\n");
  shell_printf(intf, "AX : %d\r\n", accel_raw[0]);
  shell_printf(intf, "AY : %d\r\n", accel_raw[1]);
  shell_printf(intf, "AZ : %d\r\n", accel_raw[2]);
  shell_printf(intf, "\r\n");
  shell_printf(intf, "GX : %d\r\n", gyro_raw[0]);
  shell_printf(intf, "GY : %d\r\n", gyro_raw[1]);
  shell_printf(intf, "GZ : %d\r\n", gyro_raw[2]);
  shell_printf(intf, "\r\n");
  shell_printf(intf, "Sample Rate : %u\r\n", sample_rate);
}

static void
shell_command_micros(ShellIntf* intf, int argc, const char** argv)
{
  uint32_t begin, end;

  shell_printf(intf, "\r\n");

  begin = micros_get();
  HAL_Delay(1);
  end = micros_get();

  shell_printf(intf, "begin 0x%lx, end 0x%lx, delta: %lu\r\n", begin, end, end - begin);

  begin = micros_get();
  end = micros_get();
  shell_printf(intf, "begin 0x%lx, end 0x%lx, delta: %lu\r\n", begin, end, end - begin);
}

static void
shell_command_mag_raw(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "MX : %d\r\n", mag_raw[0]);
  shell_printf(intf, "MY : %d\r\n", mag_raw[1]);
  shell_printf(intf, "MZ : %d\r\n", mag_raw[2]);
}

static void
shell_command_mag(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "MX : %d\r\n", mag_value[0]);
  shell_printf(intf, "MY : %d\r\n", mag_value[1]);
  shell_printf(intf, "MZ : %d\r\n", mag_value[2]);
  shell_printf(intf, "MXN: %d\r\n", mag_ned[0]);
  shell_printf(intf, "MYN: %d\r\n", mag_ned[1]);
  shell_printf(intf, "MZN: %d\r\n", mag_ned[2]);
}

static void
shell_command_gyro(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "GX : %d\r\n", gyro_value[0]);
  shell_printf(intf, "GY : %d\r\n", gyro_value[1]);
  shell_printf(intf, "GZ : %d\r\n", gyro_value[2]);
  shell_printf(intf, "GXN: %.2f\r\n", gyro_ned[0]);
  shell_printf(intf, "GYN: %.2f\r\n", gyro_ned[1]);
  shell_printf(intf, "GZN: %.2f\r\n", gyro_ned[2]);
}

static void
shell_command_accel(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "AX : %d\r\n", accel_value[0]);
  shell_printf(intf, "AY : %d\r\n", accel_value[1]);
  shell_printf(intf, "AZ : %d\r\n", accel_value[2]);
  shell_printf(intf, "AXN: %d\r\n", accel_ned[0]);
  shell_printf(intf, "AYN: %d\r\n", accel_ned[1]);
  shell_printf(intf, "AZN: %d\r\n", accel_ned[2]);
}

static void
shell_command_attitude(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
#if 0
  shell_printf(intf, "Roll  : %d\r\n", attitude[0]);
  shell_printf(intf, "Pitch : %d\r\n", attitude[1]);
  shell_printf(intf, "Yaw   : %d\r\n", attitude[2]);
#else
  shell_printf(intf, "Roll  : %.1f\r\n", attitude[0] / 10.f);
  shell_printf(intf, "Pitch : %.1f\r\n", attitude[1] / 10.f);
  shell_printf(intf, "Yaw   : %.1f\r\n", attitude[2] / 10.f);
#endif
}

static void
shell_command_cal_show(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");

  shell_printf(intf, "AX Offset  : %d\r\n", accel_offset[0]);
  shell_printf(intf, "AY Offset  : %d\r\n", accel_offset[1]);
  shell_printf(intf, "AZ Offset  : %d\r\n", accel_offset[2]);
  shell_printf(intf, "AX Gain    : %d\r\n", accel_gain[0]);
  shell_printf(intf, "AY Gain    : %d\r\n", accel_gain[1]);
  shell_printf(intf, "AZ Gain    : %d\r\n", accel_gain[2]);

  shell_printf(intf, "GX Offset  : %d\r\n", gyro_offset[0]);
  shell_printf(intf, "GY Offset  : %d\r\n", gyro_offset[1]);
  shell_printf(intf, "GZ Offset  : %d\r\n", gyro_offset[2]);

  shell_printf(intf, "MX Offset  : %d\r\n", mag_offset[0]);
  shell_printf(intf, "MY Offset  : %d\r\n", mag_offset[1]);
  shell_printf(intf, "MZ Offset  : %d\r\n", mag_offset[2]);

#ifdef MAGNETO_CAL_SCALE
  shell_printf(intf, "MX Scale   : %.2f\r\n", mag_scale[0]);
  shell_printf(intf, "MY Scale   : %.2f\r\n", mag_scale[1]);
  shell_printf(intf, "MZ Scale   : %.2f\r\n", mag_scale[2]);
#endif
}

////////////////////////////////////////////////////////////////////////////////
//
// magnetometer calibration
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_mag_cal_callback(int16_t offsets[3], void* cb_arg)
{
  ShellIntf* intf = (ShellIntf*)cb_arg;

  shell_printf(intf, "\r\nMagnetometer Calibration Complete\r\n");

  shell_printf(intf, "Offset X : %d\r\n", offsets[0]);
  shell_printf(intf, "Offset Y : %d\r\n", offsets[1]);
  shell_printf(intf, "Offset Z : %d\r\n", offsets[2]);
}

static void
shell_command_mag_cal(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\nStarting Magnetometer Calibration\r\n");

  if(magneto_calibrate(shell_command_mag_cal_callback, (void*)intf) == false)
  {
    shell_printf(intf, "Magnetometer Calibration Already In Progress\r\n");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// gyro calibration
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_gyro_cal_callback(int16_t offsets[3], void* cb_arg)
{
  ShellIntf* intf = (ShellIntf*)cb_arg;

  shell_printf(intf, "\r\nGyro Calibration Complete\r\n");

  shell_printf(intf, "Offset X : %d\r\n", offsets[0]);
  shell_printf(intf, "Offset Y : %d\r\n", offsets[1]);
  shell_printf(intf, "Offset Z : %d\r\n", offsets[2]);
}

static void
shell_command_gyro_cal(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\nStarting Gyro Calibration\r\n");

  if(accelgyro_gyro_calibrate(shell_command_gyro_cal_callback, (void*)intf) == false)
  {
    shell_printf(intf, "Gyro/Accel Calibration Already In Progress\r\n");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// accel calibration
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_accel_cal_step_callback(int ndx, void* cb_arg)
{
  ShellIntf* intf = (ShellIntf*)cb_arg;

  shell_printf(intf, "\r\nAccelerometer Meter Calibration Sampling Done for %d\r\n", ndx);
}

static void
shell_command_accel_cal_done_callback(int16_t offsets[3], int16_t gains[3], void* cb_arg)
{
  ShellIntf* intf = (ShellIntf*)cb_arg;

  shell_printf(intf, "\r\nAccelerometer Meter Calibration Complete\r\n");

  shell_printf(intf, "Offset X : %d\r\n", offsets[0]);
  shell_printf(intf, "Offset Y : %d\r\n", offsets[1]);
  shell_printf(intf, "Offset Z : %d\r\n", offsets[2]);

  shell_printf(intf, "Gain   X : %d\r\n", gains[0]);
  shell_printf(intf, "Gain   Y : %d\r\n", gains[1]);
  shell_printf(intf, "Gain   Z : %d\r\n", gains[2]);
}

static void
shell_command_accel_cal(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\nStarting Accelerometer Calibration\r\n");

  if(accelgyro_accel_calibrate(
        shell_command_accel_cal_step_callback,
        shell_command_accel_cal_done_callback,
        (void*)intf) == false)
  {
    shell_printf(intf, "Gyro/Accel Calibration Already In Progress\r\n");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// shell core
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_execute_command(ShellIntf* intf, char* cmd)
{
  static const char*    argv[SHELL_COMMAND_MAX_ARGS];
  int                   argc = 0;
  size_t                i;
  char                  *s, *t;

  while((s = strtok_r(argc  == 0 ? cmd : NULL, " \t", &t)) != NULL)
  {
    if(argc >= SHELL_COMMAND_MAX_ARGS)
    {
      shell_printf(intf, "\r\nError: too many arguments\r\n");
      return;
    }
    argv[argc++] = s;
  }

  if(argc == 0)
  {
    return;
  }

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    if(strcmp(_commands[i].command, argv[0]) == 0)
    {
      shell_printf(intf, "\r\nExecuting %s\r\n", argv[0]);
      _commands[i].handler(intf, argc, argv);
      return;
    }
  }
  shell_printf(intf, "%s", "\r\nUnknown Command: ");
  shell_printf(intf, "%s", argv[0]);
  shell_printf(intf, "%s", "\r\n");
}


void
shell_printf(ShellIntf* intf, const char* fmt, ...)
{
  va_list   args;
  int       len;

  va_start(args, fmt);
  len = vsnprintf(_print_buffer, SHELL_MAX_COLUMNS_PER_LINE, fmt, args);
  va_end(args);

  intf->put_tx_data(intf, (uint8_t*)_print_buffer, len);
}


////////////////////////////////////////////////////////////////////////////////
//
// public interface
//
////////////////////////////////////////////////////////////////////////////////
void
shell_init(void)
{
  shell_if_usb_init();
}

void
shell_start(void)
{
  ShellIntf* intf;

  list_for_each_entry(intf, &_shell_intf_list, lh)
  {
    intf->put_tx_data(intf, (uint8_t*)_welcome, sizeof(_welcome) -1);
    shell_prompt(intf);
  }
}


void
shell_if_register(ShellIntf* intf)
{
  list_add_tail(&intf->lh, &_shell_intf_list);
}

void
shell_handle_rx(ShellIntf* intf)
{
  uint8_t   b;

  while(1)
  {
    if(intf->get_rx_data(intf, &b) == false)
    {
      return;
    }

    if(b != '\r' && intf->cmd_buffer_ndx < SHELL_MAX_COMMAND_LEN)
    {
      if(b == '\b' || b == 0x7f)
      {
        if(intf->cmd_buffer_ndx > 0)
        {
          shell_printf(intf, "%c%c%c", b, 0x20, b);
          intf->cmd_buffer_ndx--;
        }
      }
      else
      {
        shell_printf(intf, "%c", b);
        intf->cmd_buffer[intf->cmd_buffer_ndx++] = b;
      }
    }
    else if(b == '\r')
    {
      intf->cmd_buffer[intf->cmd_buffer_ndx++] = '\0';

      shell_execute_command(intf, (char*)intf->cmd_buffer);

      intf->cmd_buffer_ndx = 0;
      shell_prompt(intf);
    }
  }
}

struct list_head*
shell_get_intf_list(void)
{
  return &_shell_intf_list;
}
