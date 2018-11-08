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
#include "micros.h"

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
  int16_t accel[3],
          gyro[3];

  uint16_t  sample_rate;

  accelgyro_get(accel, gyro);
  sample_rate = accelgyro_sample_rate();

  shell_printf(intf, "\r\n");
  shell_printf(intf, "AX : %d\r\n", accel[0]);
  shell_printf(intf, "AY : %d\r\n", accel[1]);
  shell_printf(intf, "AZ : %d\r\n", accel[2]);
  shell_printf(intf, "\r\n");
  shell_printf(intf, "GX : %d\r\n", gyro[0]);
  shell_printf(intf, "GY : %d\r\n", gyro[1]);
  shell_printf(intf, "GZ : %d\r\n", gyro[2]);
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

  shell_printf(intf, "begin %lu, end %lu, delta: %lu\r\n", begin, end, end - begin);

  begin = micros_get();
  end = micros_get();
  shell_printf(intf, "begin %lu, end %lu, delta: %lu\r\n", begin, end, end - begin);
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
