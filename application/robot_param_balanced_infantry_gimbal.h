/**
  * @file       robot_param_balanced_infantry_gimbal.h
  * @brief      这里是平衡步兵机器人云台部分参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT  // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER        // 选择发射机构类型

// clang-format off
#define __SELF_BOARD_ID C_BOARD_BALANCE_GIMBAL  // 本板ID
#define __GYRO_BIAS_YAW  0.003096855f           // 陀螺仪零飘，单位rad/s

#define __CONTROL_LINK_RC  CL_RC_UART2   // 控制链路选择：RC遥控器
#define __CONTROL_LINK_KM  CL_KM_RC      // 控制链路选择：键鼠数据

/*******************************************************************************/
/* Gimbal                                                                      */
/*******************************************************************************/
//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

// gimbal can
#define GIMBAL_CAN 1
#define GIMBAL_STDID 0x1FF

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (0.5f)

//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY  ( 100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE ( 20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID   ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN   ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)1)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE   ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION   ( 1)
#define GIMBAL_DIRECT_PITCH_DIRECTION ( 1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO   (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE   (DJI_VOLTAGE_MODE)
#define GIMBAL_DIRECT_PITCH_MODE (DJI_VOLTAGE_MODE)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH ( 0.41f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.70f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (-2.08f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID   ( 1.30f)     //云台初始化正对齐的时候使用的yaw轴正中心量

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE       (28.0f)
#define KI_GIMBAL_YAW_ANGLE       (0.0f)
#define KD_GIMBAL_YAW_ANGLE       (10.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_ANGLE  (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY       (5000.0f)
#define KI_GIMBAL_YAW_VELOCITY       (10.0f)
#define KD_GIMBAL_YAW_VELOCITY       (0.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (2000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY  (25000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE       (30.0f)
#define KI_GIMBAL_PITCH_ANGLE       (0.0f)
#define KD_GIMBAL_PITCH_ANGLE       (10.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE  (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY       (5000.0f)
#define KI_GIMBAL_PITCH_VELOCITY       (15.0f)
#define KD_GIMBAL_PITCH_VELOCITY       (0.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (3000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY  (25000.0f)


/*******************************************************************************/
/* Shoot                                                                       */
/*******************************************************************************/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 1.0f   // 定义英雄电机到拨弹盘的齿轮减速比
  
/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 4
#define FRIC_MOTOR_R_ID 2
#define FRIC_MOTOR_L_ID 1

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机std_id
#define STD_ID 0x200

//单环拨弹速度
#define TRIGGER_SPEED               (350.0f)
//摩擦轮速度
#define FRIC_R_SPEED                  (666.0f) 
#define FRIC_L_SPEED                  (-666.0f) 
#define FRIC_SPEED_LIMIT            (600.0f) 

//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED         5.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1250
#define REVERSE_SPEED               (-20.0f)  

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f)  //  (none)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度单环
#define TRIGGER_SPEED_PID_KP (100.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)  
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

// 单发模式 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (20.0f)
#define TRIGGER_ANGEL_PID_KI (0.5f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (10000.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (1000.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (200.0f)
#define FIRC_SPEED_PID_KI (1.0f)
#define FRIC_SPEED_PID_KD (1.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE     80//89

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
/*------------------------------ End of File ------------------------------*/
