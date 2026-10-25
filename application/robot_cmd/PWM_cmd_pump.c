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

#include "PWM_cmd_pump.h"

#include "bsp_pwm.h"
#include "main.h"

#define PUMP_MIN_PWM 0
#define PUMP_MAX_PWM 30000
/*-------------------- Public functions --------------------*/

/**
 * @brief          通过PWM发送PWM信号控制气泵
 * @param[in]      pump_id 气泵ID，从PWM口开始为1
 * @param[in]      pwm pwm信号占空比
 * @retval         none
 */
void PwmCmdPump(uint8_t pump_id, uint16_t pwm)
{
    if (pwm > PUMP_MAX_PWM) {
        pwm = PUMP_MAX_PWM;
    }
    pump_pwm_set(pwm, pump_id);
}
