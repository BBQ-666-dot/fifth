/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响 
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/

#include "gimbal_yaw_pitch_direct.h"
#include "usb_debug.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;


/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */

void Angle_solution(void)
{
  /* motor_feedback 使用连续角 = 圈数*2π + 单圈角，避免编码器过零(8191->0)时
     fdb.pos 跳变 2π 导致 angle_zero_for_imu 突跳、限位窗口平移、云台反向/乱摆。
     (fdb.round 在 CAN_receive.c 的 GetDjiFdbData() 中累加) */
  float motor_feedback=gimbal_direct.pitch.fdb.round*2.0f*(float)M_PI+gimbal_direct.pitch.fdb.pos,imu_feedback=gimbal_direct.feedback_pos.pitch,motor_mid=GIMBAL_DIRECT_PITCH_MID,imu_mid=0.0;
  float motor_delta=GIMBAL_DIRECT_PITCH_DIRECTION*(motor_feedback-motor_mid),imu_delta=imu_feedback-imu_mid;
  gimbal_direct.angle_zero_for_imu=imu_delta-motor_delta;
}

/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */

bool Gimbal_direct_init_judge (void)
{
  // 初始化目标由编码器中值换算得到，完成条件比较 IMU 闭环误差。
  float yaw_err = loop_fp32_constrain(
      gimbal_direct.reference.yaw - gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
  float pitch_err = gimbal_direct.reference.pitch - gimbal_direct.feedback_pos.pitch;
  if ((fabsf(yaw_err) < 0.003f && fabsf(pitch_err) < 0.003f) ||
      gimbal_direct.init_timer >= GIMBAL_INIT_TIME)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*----------------Gimbal_direct_ecd_to_imu--------------------*/
/**
 * @brief          ecd角度值转换成imu角度值
 * @param[in]      axis 用于知道读取哪一个轴的角度转换
 * @param[in]      value 用于准换的值
 * @retval         float imu映射角度值
 */

 float Gimbal_direct_ecd_to_imu(uint8_t axis,float value)
 {
  if (axis == AX_PITCH)
  {
    return value - CmdGimbalJointState(AX_PITCH) + gimbal_direct.feedback_pos.pitch;
  }

  else if (axis == AX_YAW)
  {
    return value - CmdGimbalJointState(AX_YAW) + gimbal_direct.feedback_pos.yaw;
  }

  else 
  {
    return 0.0f;
  }
 }


/*-------------------------The end of internal functions--------------------------------------*/

/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
  return loop_fp32_constrain(
      gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID), -M_PI, M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
  return gimbal_direct.init_continue;
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
  if ( axis == AX_PITCH )
  {
    return loop_fp32_constrain(gimbal_direct.pitch.direction * (gimbal_direct.pitch.fdb.pos - GIMBAL_DIRECT_PITCH_MID),-M_PI,M_PI);
  }
  else if ( axis == AX_YAW )
  {
    //yaw 输出连续多圈角，便于上位机跨圈
    return gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.round * 2.0f * (float)M_PI + gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID);
  }
  else 
  {
    return 0.0;
  }
}


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void) 
{
  //step1 获取所有所需变量指针
   gimbal_direct.rc = get_remote_control_point(); 
   //step2 置零所有值
   gimbal_direct.reference.pitch=0;
   gimbal_direct.reference.yaw=0;

   gimbal_direct.feedback_pos.pitch=0;
   gimbal_direct.feedback_pos.yaw=0;

   gimbal_direct.feedback_vel.pitch=0;
   gimbal_direct.feedback_vel.yaw=0;

   gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;

   gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
   //step3 PID数据清零，设置PID参数
   const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
   const static fp32 gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
   
	 const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};

   PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);

   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);

   //step5 初始化云台初始化校准相关变量
   gimbal_direct.init_start_time=0;
   gimbal_direct.init_timer=0;
   gimbal_direct.init_continue=false;

   //step6 模式设置初始化
   gimbal_direct.mode=GIMBAL_ZERO_FORCE;
   gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
   gimbal_direct.mode_before_rc_err = GIMBAL_ZERO_FORCE;
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
  if ( toe_is_error(DBUS_TOE) )
  {
    gimbal_direct.mode=GIMBAL_DBUS_ERR;
  }

  else if (gimbal_direct.last_mode == GIMBAL_DBUS_ERR)
  {
    gimbal_direct.mode = gimbal_direct.mode_before_rc_err;//遥控器(DBUS)断联之前，云台当时所处的模式"，用来在断联恢复后回到原来的模式
  }

  //下档无力
  else if ((switch_is_down(gimbal_direct.rc->rc.s[0]))) //安全档优先级最高
  {
    gimbal_direct.mode=GIMBAL_ZERO_FORCE;
    gimbal_direct.init_continue=false;
  }
  //初始校准模式
  else if (gimbal_direct.mode==GIMBAL_ZERO_FORCE || gimbal_direct.mode==GIMBAL_INIT)  
  {

    gimbal_direct.mode=GIMBAL_INIT;
 
    gimbal_direct.init_continue=Gimbal_direct_init_judge();
    if (gimbal_direct.init_continue==true)//判断是否需要跳出循环
    {
      gimbal_direct.mode=GIMBAL_GAP;
    }
  }
  //上，中档陀螺仪控制
  else if (switch_is_mid(gimbal_direct.rc->rc.s[0]))
  {
    gimbal_direct.mode=GIMBAL_IMU;
  }

  else if (switch_is_up(gimbal_direct.rc->rc.s[0]))
  {
    // 上档小陀螺：云台保持 IMU 世界稳向
    gimbal_direct.mode=GIMBAL_IMU;
  }
}
/*-------------------- Observe --------------------*/
 
/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
  //���������ݸ���
  GetMotorMeasure(&gimbal_direct.yaw);
  GetMotorMeasure(&gimbal_direct.pitch);

  //IMU相关数据更新
  gimbal_direct.feedback_pos.pitch=GetImuAngle(AX_PITCH);
  gimbal_direct.feedback_pos.yaw=GetImuAngle(AX_YAW);

  gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_PITCH);
  gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);

  /*
   * 原 yaw 编码器闭环反馈，现改为 IMU 对地角度闭环，保留代码供回退/对照。
   * 编码器相对角度仍由 GetGimbalDeltaYawMid() 提供给底盘跟随。
   *
   * float yaw_cont=gimbal_direct.yaw.fdb.round*2.0f*(float)M_PI+
   *                gimbal_direct.yaw.fdb.pos;
   * gimbal_direct.feedback_pos.yaw=
   *     gimbal_direct.yaw.direction*(yaw_cont-GIMBAL_DIRECT_YAW_MID);
   * gimbal_direct.feedback_vel.yaw=
   *     gimbal_direct.yaw.direction*gimbal_direct.yaw.fdb.vel;
   */

  Angle_solution();


  if (gimbal_direct.mode == GIMBAL_INIT) //初始化校准模式时钟更新
  {
    if (gimbal_direct.last_mode != GIMBAL_INIT)
    {
      gimbal_direct.init_start_time=xTaskGetTickCount();
    }

    gimbal_direct.init_timer=xTaskGetTickCount()-gimbal_direct.init_start_time;
  }
  else
  {
      gimbal_direct.init_timer=0;
  }

  if (gimbal_direct.mode == GIMBAL_DBUS_ERR && gimbal_direct.last_mode != GIMBAL_DBUS_ERR )
  {
    gimbal_direct.mode_before_rc_err=gimbal_direct.last_mode;
  }

  gimbal_direct.last_mode=gimbal_direct.mode; //上一运行模式更新


}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
  if (gimbal_direct.mode == GIMBAL_INIT)
  {
    gimbal_direct.reference.pitch= 0.0f;// loop_fp32_constrain(//gimbal_direct.pitch.direction * (GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos) + gimbal_direct.feedback_pos.pitch , -M_PI , M_PI);
    // 校准时固定对准编码器零位，但 yaw 闭环反馈使用 IMU。
    gimbal_direct.reference.yaw = loop_fp32_constrain(
        gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos) +
            gimbal_direct.feedback_pos.yaw,
        -M_PI, M_PI);
  }

  else if (gimbal_direct.mode == GIMBAL_GAP)
  {
    gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
    gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
  }

  else if (gimbal_direct.mode==GIMBAL_IMU)
  {
    if (gimbal_direct.last_mode != GIMBAL_IMU)
    {
      gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
      gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
    }

    else 
    {
      // warning :不建议键鼠跟遥控器同时使用！
      //读取鼠标的移动（还未测试过鼠标）
      //暂时先屏蔽一下鼠标功能
      gimbal_direct.reference.pitch=fp32_constrain( gimbal_direct.reference.pitch - gimbal_direct.rc->mouse.y/MOUSE_SENSITIVITY , GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu,GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu);      //GetDt7MouseSpeed(AX_YAW)
      gimbal_direct.reference.yaw = loop_fp32_constrain(
          gimbal_direct.reference.yaw - gimbal_direct.rc->mouse.x / MOUSE_SENSITIVITY,
          -M_PI, M_PI);
      //读取摇杆的数据
      gimbal_direct.reference.pitch= fp32_constrain(gimbal_direct.reference.pitch-fp32_deadline(gimbal_direct.rc->rc.ch[1], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY,GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu,GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu);
      gimbal_direct.reference.yaw = loop_fp32_constrain(
          gimbal_direct.reference.yaw -
              fp32_deadline(gimbal_direct.rc->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE,
                            REMOTE_CONTROLLER_MAX_DEADLINE) /
                  REMOTE_CONTROLLER_SENSITIVITY,
          -M_PI, M_PI);
    }
  }

  else if (gimbal_direct.mode == GIMBAL_AUTO_AIM)
  {
    gimbal_direct.reference.pitch = fp32_constrain(Gimbal_direct_ecd_to_imu(AX_PITCH,GetScCmdGimbalAngle(AX_PITCH)), GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu  , GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu );
    gimbal_direct.reference.yaw = loop_fp32_constrain(
        Gimbal_direct_ecd_to_imu(AX_YAW, GetScCmdGimbalAngle(AX_YAW)), -M_PI, M_PI);
  }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR)
  {
    gimbal_direct.pitch.set.curr=0;
    gimbal_direct.yaw.set.curr=0;
  }
  else if (gimbal_direct.mode == GIMBAL_IMU || gimbal_direct.mode== GIMBAL_GAP || gimbal_direct.mode == GIMBAL_AUTO_AIM || GIMBAL_INIT)
  {
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.curr=gimbal_direct.pitch.direction * PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);

    fp32 delta_yaw = loop_fp32_constrain(
        gimbal_direct.reference.yaw - gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle,0,delta_yaw);
    gimbal_direct.yaw.set.curr=gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);

    if (gimbal_direct.yaw.offline)//编码器反馈丢失时断开输出，防止开环跑飞
    {
      gimbal_direct.yaw.set.curr=0.0f;
    }
  }

  ModifyDebugDataPackage(0, gimbal_direct.reference.pitch, "pitch_ref");
  ModifyDebugDataPackage(1, gimbal_direct.feedback_pos.pitch, "pitch_fdb");
}
  

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void) 
{
    CanCmdDjiMotor(GIMBAL_CAN,GIMBAL_STDID,gimbal_direct.yaw.set.curr,gimbal_direct.pitch.set.curr,0,0);
}



#endif  // GIMBAL_YAW_PITCH
