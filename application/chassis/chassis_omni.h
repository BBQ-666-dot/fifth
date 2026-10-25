/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.03.03      Harry_Wong        1.重新构建全向轮底盘代码，完成基础控制
  *  V1.1.0   2025.11.05       CJH              1. 完成底盘与云台协同控制
  *                                             2. 完成底盘键鼠控制
  *                                             3. 修正全向轮解算的错误
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H
#include "motor.h"
#include "IMU_task.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include  "user_lib.h"
#include "CAN_cmd_dji.h"



/*-------------------- Structural definition --------------------*/
typedef enum {
    CHASSIS_LOCK,      //底盘锁定，所有轮子速度设定为0
    CHASSIS_SINGLE,    //只有底盘的模式
    CHASSIS_FOLLOW,    //云台跟随模式
    CHASSIS_SPIN,
} ChassisMode_e;

/**
 * @brief  底盘轮子PID
 */
 typedef struct
{
    pid_type_def wheel_velocity[4];//麦轮速度解算PID

    pid_type_def follow; //云台跟随PID
} PID_t;   

/**
 * @brief  底盘期望
 */
typedef struct
{
    float vx;
    float vy;
    float wz;
} Reference_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式

    /*-------------------- Motors --------------------*/
    Motor_s wheel[4];  //底盘电机

    /*-------------------- Values --------------------*/
    Reference_t reference; 
    Reference_t reference_rc;

    fp32 feedback[4];
    fp32 set[4];

    fp32 yaw_delta;

    uint16_t x_time;
    uint16_t y_time;
    uint8_t spin_flag;
    uint8_t shift_flag;

    uint8_t sc_flag;
    uint32_t f_flag;
} Chassis_s;


extern void ChassisInit(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif 
#endif 
