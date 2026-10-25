/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       PWM_cmd_pump.c/h
  * @brief      PWM发送函数，通过PWM信号控制气泵.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     2025/2/11       YZX             1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */
#ifndef PWM_CMD_PUMP_H
#define PWM_CMD_PUMP_H
#include "struct_typedef.h"

extern void PwmCmdPump(uint8_t pump_id, uint16_t pwm);

#endif
