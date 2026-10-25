/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef SHOOT_H
#define SHOOT_H

#include "robot_param.h"

#if SHOOT_TYPE != SHOOT_NONE

#include "shoot_fric_trigger.h"
#include "struct_typedef.h"

// 任务相关宏定义
#define SHOOT_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define SHOOT_CONTROL_TIME 1      // 任务控制间隔 1ms

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1  // 射击发射开关通道数据

// clang-format off
typedef struct
{
    void     (*GetStatus)(void);
    uint32_t (*GetDuration)(void);
} ShootApi_t;
// clang-format on

extern ShootApi_t shoot;

#endif  // SHOOT_TYPE
#endif
/*------------------------------ End of File ------------------------------*/
