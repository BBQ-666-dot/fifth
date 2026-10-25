/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_engineer.c/h
  * @brief      工程机械臂配套自定义控制器功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-22-2024     Penguin         1. done
  *  V1.0.1     Jan-14-2025     Penguin         1. 能获取关节的位置
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CUSTOM_CONTROLLER_ENGINEER_H
#define CUSTOM_CONTROLLER_ENGINEER_H
#include "robot_param.h"

#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_ENGINEER)

#include "motor.h"
#include "pid.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define JOINT_NUM 6  // 关节数量

typedef enum {
    CUSTOM_CONTROLLER_SAFE,       // 安全模式
    CUSTOM_CONTROLLER_CALIBRATE,  // 校准模式
    CUSTOM_CONTROLLER_DRAGGING,   // 拖拽模式
    CUSTOM_CONTROLLER_DEBUG,      // 调试模式
} CustomControllerMode_e;

/**
 * @brief  机械臂数据结构体
 */
typedef struct
{
    CustomControllerMode_e mode;  // 机械臂模式
    uint8_t error_code;           // 机械臂错误代码

    /*-------------------- Motors --------------------*/
    Motor_s joint_motor[JOINT_NUM];
    /*-------------------- Values --------------------*/

    struct
    {
        struct
        {
            float vel;  // (rad/s)速度
        } joint[JOINT_NUM];
    } ref;

    struct
    {
        struct
        {
            float pos;   // (rad)位置
            float dpos;  // (rad)位置差
            float vel;   // (rad/s)速度
            int16_t round;   // 圈数(当前位于第几圈中，初始为0)
        } joint[JOINT_NUM];
    } fdb;

    struct
    {
        float pos[JOINT_NUM];
    } transform;

    struct
    {
        struct
        {
            float pos[6];
            float vj4_pos;
            float vj5_pos;
        } max;
        struct
        {
            float pos[6];
            float vj4_pos;
            float vj5_pos;
        } min;
    } limit;

    struct
    {
        pid_type_def joint[JOINT_NUM];
    } pid;

    struct
    {
        LowPassFilter_t joint[JOINT_NUM];
    } lpf;

} CustomController_s;

extern void CustomControllerPublish(void);
extern void CustomControllerInit(void);
extern void CustomControllerHandleException(void);
extern void CustomControllerSetMode(void);
extern void CustomControllerObserver(void);
extern void CustomControllerReference(void);
extern void CustomControllerConsole(void);
extern void CustomControllerSendCmd(void);

#endif  // CUSTOM_CONTROLLER_TYPE
#endif  // CUSTOM_CONTROLLER_ENGINEER_H
/*------------------------------ End of File ------------------------------*/
