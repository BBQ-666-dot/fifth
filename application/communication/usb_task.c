/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V1.1.0     2026-4-8        CJH             1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "gimbal.h"
#include "IMU.h"


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   5  // ms
#define SEND_DURATION_GameStatus  100  // ms
// #define SEND_DURATION_GameResult  100  // ms
#define SEND_DURATION_AllRobotHp  50  // ms
#define SEND_DURATION_Event       100  // ms
#define SEND_DURATION_RefereeWarning 100  // ms
// #define SEND_DURATION_DartInfo     5  // ms
#define SEND_DURATION_RobotStatus  20// ms
// #define SEND_DURATION_PowerHeat 20// ms
// #define SEND_DURATION_RobotPos 100// ms
#define SEND_DURATION_Buff   50// ms
// #define SEND_DURATION_Hurt 100// ms
#define SEND_DURATION_Shoot 10// ms
// #define SEND_DURATION_ProjectileAllowance 10// ms
#define SEND_DURATION_RfidStatus 10// ms
// #define SEND_DURATION_DartClientCmd 5// ms
#define SEND_DURATION_GroundRobotPosition 50// ms
// #define SEND_DURATION_RadarMark 5// ms
// #define SEND_DURATION_SentryInfo 5// ms
// #define SEND_DURATION_RadarInfo 5// ms
// #define SEND_DURATION_RobotInteraction 5// ms
// #define SEND_DURATION_CustomController 5// ms
// #define SEND_DURATION_MapCommand 5// ms
// #define SEND_DURATION_RobotCustom 5// ms
// #define SEND_DURATION_RobotCustom3 5// ms
// #define SEND_DURATION_PidDebugData 5// ms
// #define SEND_DURATION_Pid 5// ms
#define SEND_DURATION_JointState   5// ms
#define SEND_DURATION_RobotMotion 5  // ms
// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.send_name) >= SEND_DURATION_##send_name) {   \
            LAST_SEND_TIME.send_name = HAL_GetTick();                                    \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

static const game_status_t * GAME_STATUS;
//static const game_result_t * GAME_RESULT;
static const game_robot_HP_t * GAME_ROBOT_HP;
static const event_data_t * EVENT_DATA;
static const referee_warning_t * REFEREE_WARNING;
//static const dart_info_t * DART_INFO;
static const robot_status_t * ROBOT_STATUS;
// static const power_heat_data_t * POWER_HEAT_DATA;
static const robot_pos_t * ROBOT_POS;
static const buff_t * BUFF_DATA;
static const hurt_data_t * HURT_DATA;
static const shoot_data_t * SHOOT_DATA;
static const projectile_allowance_t * PROJECTILE_ALLOWANCE;
static const rfid_status_t * RFID_STATUS;
//static const dart_client_cmd_t * DART_CLIENT_CMD;
static const ground_robot_position_t * GROUND_ROBOT_POSITION;
//static const radar_mark_data_t * RADAR_MARK_DATA;
//static const sentry_info_t * SENTRY_INFO;
//static const radar_info_t * RADAR_INFO;
//static const robot_interaction_data_t * ROBOT_INTERACTION_DATA;
//static const CustomControllerData_t * CUSTOM_CONTROLLER_DATA;
//static const map_command_t * MAP_COMMAND;
//static const robot_custom_data_t * ROBOT_CUSTOM_DATA;
//static const robot_custom_data_3_t * ROBOT_CUSTOM_DATA3;
// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;

static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
//static SendDataGameResult_s  SEND_DATA_GAME_RESULT;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataRefereeWarning_s SEND_DATA_REFEREE_WARNING;
// static SendDataDartInfo_s      SEND_DATA_DART_INFO;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
// static SendDataPowerHeat_s   SEND_POWER_HEAT_DATA;
//static SendDataRobotPos_s    SEND_ROBOT_POS_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;
//static SendDataHurt_s    SEND_HURT_DATA;
static SendDataShootData_s   SEND_SHOOT_DATA;
//static SendDataProjectileAllowance_s SEND_PROJECTILE_ALLOWANCE_DATA;
static SendDataRfidStatus_s   SEND_RFID_STATUS_DATA;
// static SendDataDartClientCmd_s SEND_DART_CLIENT_CMD_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
// static SendDataRadarMarkData_s SEND_RADAR_MARK_DATA;
// static SendDataSentryInfo_s SEND_SENTRY_INFO_DATA;
// static SendDataRadarInfo_s SEND_RADAR_INFO_DATA;
// static SendDataRobotInteraction_s SEND_ROBOT_INTERACTION_DATA;
// static SendDataCustomController_s SEND_CUSTOM_CONTROLLER_DATA;
// static SendDataMapCommand_s SEND_MAP_COMMAND_DATA;
// static SendDataRobotCustomData_s SEND_ROBOT_CUSTOM_DATA;
// static SendDataRobotCustomData3_s SEND_ROBOT_CUSTOM_DATA3;
// static SendDataPidDebug_s SEND_DATA_PID;
static SendDataJointState_s SEND_JOINT_STATE_DATA;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
// clang-format on

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t GameStatus;
    // uint32_t GameResult;
    uint32_t AllRobotHp;
    uint32_t Event;
    uint32_t RefereeWarning;
    // uint32_t DartInfo;
    uint32_t RobotStatus;
    // uint32_t PowerHeat;
    // uint32_t RobotPos;
    uint32_t Buff;
    // uint32_t Hurt;
    uint32_t Shoot;
    uint32_t ProjectileAllowance;
    uint32_t RfidStatus;
    // uint32_t DartClientCmd;
    uint32_t GroundRobotPosition;
    // uint32_t RadarMark;
    // uint32_t SentryInfo;
    // uint32_t RadarInfo;
    // uint32_t RobotInteraction;
    // uint32_t CustomController;
    // uint32_t MapCommand;
    // uint32_t RobotCustom;
    // uint32_t RobotCustom3;
    // uint32_t Pid;
    uint32_t JointState;
    uint32_t RobotMotion;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void);
static void UsbSendImuData(void);
static void UsbSendRobotStateInfoData(void);
static void UsbSendGameStatusData(void);
// static void UsbSendGameResultData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendEventData(void);
static void UsbSendRefereeWarningData(void);
//static void UsbSendDartInfoData(void);
static void UsbSendRobotStatusData(void);
// static void UsbSendPowerHeatData(void);
//static void UsbSendRobotPosData(void);
static void UsbSendBuffData(void);
//static void UsbSendHurtData(void);
static void UsbSendShootData(void);
//static void UsbSendProjectileAllowanceData(void);
static void UsbSendRfidStatusData(void);
//static void UsbSendDartClientCmdData(void);
static void UsbSendGroundRobotPositionData(void);
//static void UsbSendRadarMarkData(void);
//static void UsbSendSentryInfoData(void);
//static void UsbSendRadarInfoData(void);
//static void UsbSendRobotInteractionData(void);
//static void UsbSendCustomControllerData(void);
//static void UsbSendMapCommandData(void);
//static void UsbSendRobotCustomData(void);
//static void UsbSendRobotCustom3Data(void);
//static void UsbSendPidData(void);
static void UsbSendJointStateData(void);
static void UsbSendRobotMotionData(void);
/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = Subscribe(IMU_NAME);                             // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针
    GAME_STATUS = &game_status;
//   GAME_RESULT = &game_result;
    GAME_ROBOT_HP = &game_robot_HP;
    EVENT_DATA = &field_event;
    REFEREE_WARNING = &referee_warning;
//    DART_INFO = &dart_info;
    ROBOT_STATUS = &robot_status;
//    POWER_HEAT_DATA = &power_heat_data;
    ROBOT_POS = &robot_pos;
    BUFF_DATA = &buff;
    HURT_DATA = &hurt_data;
    SHOOT_DATA = &shoot_data;
    PROJECTILE_ALLOWANCE = &projectile_allowance;
    RFID_STATUS = &rfid_status;
//    DART_CLIENT_CMD = &dart_client_cmd;
    GROUND_ROBOT_POSITION = &ground_robot_position;
//    RADAR_MARK_DATA = &radar_mark_data;
//    SENTRY_INFO = &sentry_info;
//    RADAR_INFO = &radar_info;
//    ROBOT_INTERACTION_DATA = &robot_interaction_data;
//    CUSTOM_CONTROLLER_DATA = &custom_controller_data;
//    MAP_COMMAND = &map_command;
//    ROBOT_CUSTOM_DATA = &robot_custom_data;
//    ROBOT_CUSTOM_DATA3 = &robot_custom_data_3;
    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),sizeof(SEND_DATA_GAME_STATUS.frame_header));
    
    // 5.初始化比赛结果数据
    // SEND_DATA_GAME_RESULT.frame_header.sof = SEND_SOF;
    // SEND_DATA_GAME_RESULT.frame_header.len = (uint8_t)(sizeof(SendDataGameResult_s) - 6);
    // SEND_DATA_GAME_RESULT.frame_header.id = GAME_RESULT_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_DATA_GAME_RESULT.frame_header), sizeof(SEND_DATA_GAME_RESULT.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));
    
    // 7.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum
        ((uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 8.初始化裁判警告数据包
    SEND_DATA_REFEREE_WARNING.frame_header.sof = SEND_SOF;
    SEND_DATA_REFEREE_WARNING.frame_header.len = (uint8_t)(sizeof(SendDataRefereeWarning_s) - 6);
    SEND_DATA_REFEREE_WARNING.frame_header.id = REFEREE_WARNING_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_REFEREE_WARNING.frame_header), sizeof(SEND_DATA_REFEREE_WARNING.frame_header));

    // 9.初始化飞镖信息数据包
    // SEND_DATA_DART_INFO.frame_header.sof = SEND_SOF;
    // SEND_DATA_DART_INFO.frame_header.len = (uint8_t)(sizeof(SendDataDartInfo_s) - 6);
    // SEND_DATA_DART_INFO.frame_header.id = DART_INFO_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_DATA_DART_INFO.frame_header), sizeof(SEND_DATA_DART_INFO.frame_header));

    // 10.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),sizeof(SEND_ROBOT_STATUS_DATA.frame_header));

    // 11.初始化电源热量数据
    // SEND_POWER_HEAT_DATA.frame_header.sof = SEND_SOF;
    // SEND_POWER_HEAT_DATA.frame_header.len = (uint8_t)(sizeof(SendDataPowerHeat_s) - 6);
    // SEND_POWER_HEAT_DATA.frame_header.id = POWER_HEAT_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_POWER_HEAT_DATA.frame_header), sizeof(SEND_POWER_HEAT_DATA.frame_header));
    
    // 12.初始化机器人位置数据
    // SEND_ROBOT_POS_DATA.frame_header.sof = SEND_SOF;
    // SEND_ROBOT_POS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotPos_s) - 6);
    // SEND_ROBOT_POS_DATA.frame_header.id = ROBOT_POS_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_ROBOT_POS_DATA.frame_header), sizeof(SEND_ROBOT_POS_DATA.frame_header));
    
    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_BUFF_DATA.frame_header), sizeof(SEND_BUFF_DATA.frame_header));
    
    // 14.初始化机器人受伤数据
    // SEND_HURT_DATA.frame_header.sof = SEND_SOF;
    // SEND_HURT_DATA.frame_header.len = (uint8_t)(sizeof(SendDataHurt_s) - 6);
    // SEND_HURT_DATA.frame_header.id = HURT_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_HURT_DATA.frame_header), sizeof(SEND_HURT_DATA.frame_header));

    // 15.初始化机器人射击数据
    SEND_SHOOT_DATA.frame_header.sof = SEND_SOF;
    SEND_SHOOT_DATA.frame_header.len = (uint8_t)(sizeof(SendDataShootData_s) - 6);
    SEND_SHOOT_DATA.frame_header.id = SHOOT_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_SHOOT_DATA.frame_header), sizeof(SEND_SHOOT_DATA.frame_header));

    // 16.初始化机器人弹药余量数据
    // SEND_PROJECTILE_ALLOWANCE_DATA.frame_header.sof = SEND_SOF;
    // SEND_PROJECTILE_ALLOWANCE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataProjectileAllowance_s) - 6);
    // SEND_PROJECTILE_ALLOWANCE_DATA.frame_header.id = PROJECTILE_ALLOWANCE_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_PROJECTILE_ALLOWANCE_DATA.frame_header), sizeof(SEND_PROJECTILE_ALLOWANCE_DATA.frame_header));
    
    // 17.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header), sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 18.初始化飞镖客户端命令数据
    // SEND_DART_CLIENT_CMD_DATA.frame_header.sof = SEND_SOF;
    // SEND_DART_CLIENT_CMD_DATA.frame_header.len = (uint8_t)(sizeof(SendDataDartClientCmd_s) - 6);
    // SEND_DART_CLIENT_CMD_DATA.frame_header.id = DART_CLIENT_CMD_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_DART_CLIENT_CMD_DATA.frame_header), sizeof(SEND_DART_CLIENT_CMD_DATA.frame_header));

    // 19.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len =(uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 20.初始化雷达标志数据
    // SEND_RADAR_MARK_DATA.frame_header.sof = SEND_SOF;
    // SEND_RADAR_MARK_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRadarMarkData_s) - 6);
    // SEND_RADAR_MARK_DATA.frame_header.id = RADAR_MARK_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_RADAR_MARK_DATA.frame_header), sizeof(SEND_RADAR_MARK_DATA.frame_header));

    // 21.初始化哨兵信息数据
    // SEND_SENTRY_INFO_DATA.frame_header.sof = SEND_SOF;
    // SEND_SENTRY_INFO_DATA.frame_header.len = (uint8_t)(sizeof(SendDataSentryInfo_s) - 6);
    // SEND_SENTRY_INFO_DATA.frame_header.id = SENTRY_INFO_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_SENTRY_INFO_DATA.frame_header), sizeof(SEND_SENTRY_INFO_DATA.frame_header));

    // 22.初始化雷达信息数据
    // SEND_RADAR_INFO_DATA.frame_header.sof = SEND_SOF;
    // SEND_RADAR_INFO_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRadarInfo_s) - 6);
    // SEND_RADAR_INFO_DATA.frame_header.id = RADAR_INFO_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_RADAR_INFO_DATA.frame_header), sizeof(SEND_RADAR_INFO_DATA.frame_header));

    // // 23.初始化机器人交互数据
    // SEND_ROBOT_INTERACTION_DATA.frame_header.sof = SEND_SOF;
    // SEND_ROBOT_INTERACTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotInteraction_s) - 6);
    // SEND_ROBOT_INTERACTION_DATA.frame_header.id = ROBOT_INTERACTION_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_ROBOT_INTERACTION_DATA.frame_header), sizeof(SEND_ROBOT_INTERACTION_DATA.frame_header));

    // 24.初始化自定义控制器数据
    // SEND_CUSTOM_CONTROLLER_DATA.frame_header.sof = SEND_SOF;
    // SEND_CUSTOM_CONTROLLER_DATA.frame_header.len = (uint8_t)(sizeof(SendDataCustomController_s) - 6);
    // SEND_CUSTOM_CONTROLLER_DATA.frame_header.id = CUSTOM_CONTROLLER_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_CUSTOM_CONTROLLER_DATA.frame_header), sizeof(SEND_CUSTOM_CONTROLLER_DATA.frame_header));
    
    // 25.初始化地图命令数据
    // SEND_MAP_COMMAND_DATA.frame_header.sof = SEND_SOF;
    // SEND_MAP_COMMAND_DATA.frame_header.len = (uint8_t)(sizeof(SendDataMapCommand_s) - 6);
    // SEND_MAP_COMMAND_DATA.frame_header.id = MAP_COMMAND_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_MAP_COMMAND_DATA.frame_header), sizeof(SEND_MAP_COMMAND_DATA.frame_header));

    // 26.初始化机器人自定义数据
    // SEND_ROBOT_CUSTOM_DATA.frame_header.sof = SEND_SOF;
    // SEND_ROBOT_CUSTOM_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotCustomData_s) - 6);
    // SEND_ROBOT_CUSTOM_DATA.frame_header.id = ROBOT_CUSTOM_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_ROBOT_CUSTOM_DATA.frame_header), sizeof(SEND_ROBOT_CUSTOM_DATA.frame_header));

    // // 27.初始化机器人自定义数据3
    // SEND_ROBOT_CUSTOM_DATA3.frame_header.sof = SEND_SOF;
    // SEND_ROBOT_CUSTOM_DATA3.frame_header.len = (uint8_t)(sizeof(SendDataRobotCustomData3_s) - 6);
    // SEND_ROBOT_CUSTOM_DATA3.frame_header.id = ROBOT_CUSTOM_DATA_3_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_ROBOT_CUSTOM_DATA3.frame_header), sizeof(SEND_ROBOT_CUSTOM_DATA3.frame_header));

    // 28.初始化pid调参数据
    // SEND_DATA_PID.frame_header.sof = SEND_SOF;
    // SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    // SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    // append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
    //     (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 29.初始化云台状态数据
    SEND_JOINT_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_JOINT_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    SEND_JOINT_STATE_DATA.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_JOINT_STATE_DATA.frame_header),
        sizeof(SEND_JOINT_STATE_DATA.frame_header));

    // 30.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
}   

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送Debug数据
    CheckDurationAndSend(Debug);
    // 发送Imu数据
    CheckDurationAndSend(Imu);
    // 发送RobotStateInfo数据
    CheckDurationAndSend(RobotStateInfo);
    // 发送GameStatus数据
    CheckDurationAndSend(GameStatus);
    // 发送GameResult数据
    // CheckDurationAndSend(GameResult);
    // 发送AllRobotHp数据
    CheckDurationAndSend(AllRobotHp); 
    // 发送Event数据
    CheckDurationAndSend(Event);
    // 发送RefereeWarning数据
    CheckDurationAndSend(RefereeWarning);
    // 发送DartInfo数据
    // CheckDurationAndSend(DartInfo);
    // 发送RobotStatus数据
    CheckDurationAndSend(RobotStatus);
    // 发送PowerHeatData数据
    // CheckDurationAndSend(PowerHeat);
    // 发送RobotPos数据
    // CheckDurationAndSend(RobotPos);
    // 发送Buff数据
    CheckDurationAndSend(Buff);
    // 发送HurtData数据
    // CheckDurationAndSend(Hurt);
    // 发送ShootData数据
    CheckDurationAndSend(Shoot);
    // 发送ProjectileAllowance数据
    // CheckDurationAndSend(ProjectileAllowance);
    // 发送RfidStatus数据
    CheckDurationAndSend(RfidStatus);
    // 发送DartClientCmd数据
    // CheckDurationAndSend(DartClientCmd);
    // 发送GroundRobotPosition数据
    CheckDurationAndSend(GroundRobotPosition);
    // 发送RadarMarkData数据
    // CheckDurationAndSend(RadarMark);
    // 发送SentryInfo数据
    // CheckDurationAndSend(SentryInfo);
    // 发送RadarInfo数据
    // CheckDurationAndSend(RadarInfo);
    // 发送RobotInteractionData数据
    // CheckDurationAndSend(RobotInteraction);
    // 发送CustomController数据
    // CheckDurationAndSend(CustomController);
    // 发送MapCommand数据
    // CheckDurationAndSend(MapCommand);
    // 发送RobotCustomData数据
    // CheckDurationAndSend(RobotCustom);
    // 发送RobotCustomData3数据
    // CheckDurationAndSend(RobotCustom3);
    // // 发送PidDebug数据
    // CheckDurationAndSend(Pid);
    // 发送JointState数据
    CheckDurationAndSend(JointState);
    // 发送RobotMotion数据
    CheckDurationAndSend(RobotMotion);
    
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/


/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = GetImuAngle(AX_YAW);
    SEND_DATA_IMU.data.pitch = GetImuAngle(AX_PITCH);
    SEND_DATA_IMU.data.roll = GetImuAngle(AX_ROLL);

    SEND_DATA_IMU.data.yaw_vel = GetImuVelocity(AX_YAW);
    SEND_DATA_IMU.data.pitch_vel = GetImuVelocity(AX_PITCH);
    SEND_DATA_IMU.data.roll_vel = GetImuVelocity(AX_ROLL);

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();


    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}



/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_type = GAME_STATUS->game_type;
    SEND_DATA_GAME_STATUS.data.game_progress = GAME_STATUS->game_progress;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = GAME_STATUS->stage_remain_time;
    SEND_DATA_GAME_STATUS.data.SyncTimeStamp = GAME_STATUS->SyncTimeStamp;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送比赛结果数据
 * @param duration 发送周期
 */
//static void UsbSendGameResultData(void)
//{
//   SEND_DATA_GAME_RESULT.time_stamp = HAL_GetTick();

//   SEND_DATA_GAME_RESULT.data.winner = GAME_RESULT->winner;

//   append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_RESULT, sizeof(SendDataGameResult_s));
//   USB_Transmit((uint8_t *)&SEND_DATA_GAME_RESULT, sizeof(SendDataGameResult_s));
//}

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    // SEND_DATA_ALL_ROBOT_HP.data.red_1_robot_hp = 1;
    // SEND_DATA_ALL_ROBOT_HP.data.red_2_robot_hp = 2;
    // SEND_DATA_ALL_ROBOT_HP.data.red_3_robot_hp = 3;
    // SEND_DATA_ALL_ROBOT_HP.data.red_4_robot_hp = 4;
    // SEND_DATA_ALL_ROBOT_HP.data.red_7_robot_hp = 7;
    // SEND_DATA_ALL_ROBOT_HP.data.red_outpost_hp = 8;
    // SEND_DATA_ALL_ROBOT_HP.data.red_base_hp = 9;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_1_robot_hp = 1;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_2_robot_hp = 2;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_3_robot_hp = 3;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_4_robot_hp = 4;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 7;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_outpost_hp = 8;
    // SEND_DATA_ALL_ROBOT_HP.data.blue_base_hp = 9;
    SEND_DATA_ALL_ROBOT_HP.data.ally_1_robot_HP = GAME_ROBOT_HP->ally_1_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_2_robot_HP = GAME_ROBOT_HP->ally_2_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_3_robot_HP = GAME_ROBOT_HP->ally_3_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_4_robot_HP = GAME_ROBOT_HP->ally_4_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_7_robot_HP = GAME_ROBOT_HP->ally_7_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_outpost_HP = GAME_ROBOT_HP->ally_outpost_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_base_HP = GAME_ROBOT_HP->ally_base_HP;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    SEND_DATA_EVENT.data.event_data = EVENT_DATA->event_data;
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USB_Transmit((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送裁判警告数据
 * @param duration 发送周期
 */
static void UsbSendRefereeWarningData(void)
{
    SEND_DATA_REFEREE_WARNING.time_stamp = HAL_GetTick();
    SEND_DATA_REFEREE_WARNING.data.level = REFEREE_WARNING->level;
    SEND_DATA_REFEREE_WARNING.data.offending_robot_id = REFEREE_WARNING->offending_robot_id;
    SEND_DATA_REFEREE_WARNING.data.count = REFEREE_WARNING->count;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_REFEREE_WARNING, sizeof(SendDataRefereeWarning_s));
    USB_Transmit((uint8_t *)&SEND_DATA_REFEREE_WARNING, sizeof(SendDataRefereeWarning_s));
}

/**
 * @brief 发送飞镖信息数据
 * @param duration 发送周期
 */
// static void UsbSendDartInfoData(void)
// {
//     SEND_DATA_DART_INFO.time_stamp = HAL_GetTick();
//     SEND_DATA_DART_INFO.data.dart_remaining_time = DART_INFO->dart_remaining_time;
//     SEND_DATA_DART_INFO.data.dart_info = DART_INFO->dart_info;
//     append_CRC16_check_sum((uint8_t *)&SEND_DATA_DART_INFO, sizeof(SendDataDartInfo_s));
//     USB_Transmit((uint8_t *)&SEND_DATA_DART_INFO, sizeof(SendDataDartInfo_s));
// }

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();
    SEND_ROBOT_STATUS_DATA.data.robot_id = ROBOT_STATUS->robot_id;
    SEND_ROBOT_STATUS_DATA.data.robot_level = ROBOT_STATUS->robot_level;
    SEND_ROBOT_STATUS_DATA.data.current_HP = ROBOT_STATUS->current_HP;
    SEND_ROBOT_STATUS_DATA.data.maximum_HP = ROBOT_STATUS->maximum_HP;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_cooling_value = ROBOT_STATUS->shooter_barrel_cooling_value;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_heat_limit = ROBOT_STATUS->shooter_barrel_heat_limit;
    SEND_ROBOT_STATUS_DATA.data.chassis_power_limit = ROBOT_STATUS->chassis_power_limit;
    
    SEND_ROBOT_STATUS_DATA.data.x = ROBOT_POS->x;
    SEND_ROBOT_STATUS_DATA.data.y = ROBOT_POS->y;
    SEND_ROBOT_STATUS_DATA.data.angle = ROBOT_POS->angle;

    SEND_ROBOT_STATUS_DATA.data.armor_id = HURT_DATA->armor_id;
    SEND_ROBOT_STATUS_DATA.data.HP_deduction_reason = HURT_DATA->HP_deduction_reason;

    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_17mm = PROJECTILE_ALLOWANCE->projectile_allowance_17mm;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_42mm = PROJECTILE_ALLOWANCE->projectile_allowance_42mm;
    SEND_ROBOT_STATUS_DATA.data.remaining_gold_coin = PROJECTILE_ALLOWANCE->remaining_gold_coin;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_fortress = PROJECTILE_ALLOWANCE->projectile_allowance_fortress;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

/**
 * @brief 发送电源热量数据
 * @param duration 发送周期
 */
// static void UsbSendPowerHeatData(void)
// {
//     SEND_POWER_HEAT_DATA.time_stamp = HAL_GetTick();
//     SEND_POWER_HEAT_DATA.data.reserved1 = POWER_HEAT_DATA->reserved1;
//     SEND_POWER_HEAT_DATA.data.reserved2 = POWER_HEAT_DATA->reserved2;
//     SEND_POWER_HEAT_DATA.data.reserved = POWER_HEAT_DATA->reserved;
//     SEND_POWER_HEAT_DATA.data.buffer_energy = POWER_HEAT_DATA->buffer_energy;
//     SEND_POWER_HEAT_DATA.data.shooter_17mm_barrel_heat = POWER_HEAT_DATA->shooter_17mm_barrel_heat;
//     SEND_POWER_HEAT_DATA.data.shooter_42mm_barrel_heat = POWER_HEAT_DATA->shooter_42mm_barrel_heat;
//     append_CRC16_check_sum((uint8_t *)&SEND_POWER_HEAT_DATA, sizeof(SendDataPowerHeat_s));
//     USB_Transmit((uint8_t *)&SEND_POWER_HEAT_DATA, sizeof(SendDataPowerHeat_s));
// }

/**
 * @brief 发送机器人位置数据
 * @param duration 发送周期
 */
//static void UsbSendRobotPosData(void)
//{
//   SEND_ROBOT_POS_DATA.time_stamp = HAL_GetTick();
//   SEND_ROBOT_POS_DATA.data.x = ROBOT_POS->x;
//   SEND_ROBOT_POS_DATA.data.y = ROBOT_POS->y;
//   SEND_ROBOT_POS_DATA.data.angle = ROBOT_POS->angle;
//   append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_POS_DATA, sizeof(SendDataRobotPos_s));
//   USB_Transmit((uint8_t *)&SEND_ROBOT_POS_DATA, sizeof(SendDataRobotPos_s));
//}

/*
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    SEND_BUFF_DATA.data.recovery_buff = BUFF_DATA->recovery_buff;
    SEND_BUFF_DATA.data.defence_buff = BUFF_DATA->defence_buff;
    SEND_BUFF_DATA.data.vulnerability_buff = BUFF_DATA->vulnerability_buff;
    SEND_BUFF_DATA.data.attack_buff = BUFF_DATA->attack_buff;
    SEND_BUFF_DATA.data.remaining_energy = BUFF_DATA->remaining_energy;
    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USB_Transmit((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
}

/**
 * @brief 发送机器人受伤数据
 * @param duration 发送周期
 */
//static void UsbSendHurtData(void)
//{
//   SEND_HURT_DATA.time_stamp = HAL_GetTick();
//   SEND_HURT_DATA.data.armor_id = HURT_DATA->armor_id;
//   SEND_HURT_DATA.data.HP_deduction_reason = HURT_DATA->HP_deduction_reason;
//   append_CRC16_check_sum((uint8_t *)&SEND_HURT_DATA, sizeof(SendDataHurt_s));
//   USB_Transmit((uint8_t *)&SEND_HURT_DATA, sizeof(SendDataHurt_s));
//}

/**
 * @brief 发送机器人射击数据
 * @param duration 发送周期
 */
static void UsbSendShootData(void)
{
    SEND_SHOOT_DATA.time_stamp = HAL_GetTick();
    SEND_SHOOT_DATA.data.bullet_type = SHOOT_DATA->bullet_type;
    SEND_SHOOT_DATA.data.shooter_number = SHOOT_DATA->shooter_number;
    SEND_SHOOT_DATA.data.launching_frequency = SHOOT_DATA->launching_frequency;
    SEND_SHOOT_DATA.data.initial_speed = SHOOT_DATA->initial_speed;
    append_CRC16_check_sum((uint8_t *)&SEND_SHOOT_DATA, sizeof(SendDataShootData_s));
    USB_Transmit((uint8_t *)&SEND_SHOOT_DATA, sizeof(SendDataShootData_s));
}

/**
 * @brief 发送机器人弹药余量数据
 * @param duration 发送周期
 */
//static void UsbSendProjectileAllowanceData(void)
//{
//   SEND_PROJECTILE_ALLOWANCE_DATA.time_stamp = HAL_GetTick();
//   SEND_PROJECTILE_ALLOWANCE_DATA.data.projectile_allowance_17mm = PROJECTILE_ALLOWANCE->projectile_allowance_17mm;
//   SEND_PROJECTILE_ALLOWANCE_DATA.data.projectile_allowance_42mm = PROJECTILE_ALLOWANCE->projectile_allowance_42mm;
//   SEND_PROJECTILE_ALLOWANCE_DATA.data.remaining_gold_coin = PROJECTILE_ALLOWANCE->remaining_gold_coin;
//   SEND_PROJECTILE_ALLOWANCE_DATA.data.projectile_allowance_fortress = PROJECTILE_ALLOWANCE->projectile_allowance_fortress;
//   append_CRC16_check_sum((uint8_t *)&SEND_PROJECTILE_ALLOWANCE_DATA, sizeof(SendDataProjectileAllowance_s));
//   USB_Transmit((uint8_t *)&SEND_PROJECTILE_ALLOWANCE_DATA, sizeof(SendDataProjectileAllowance_s));
//}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    SEND_RFID_STATUS_DATA.data.rfid_status = RFID_STATUS->rfid_status;
    SEND_RFID_STATUS_DATA.data.rfid_status_2 = RFID_STATUS->rfid_status_2;
    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USB_Transmit((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送飞镖客户端命令数据
 * @param duration 发送周期
 */
// static void UsbSendDartClientCmdData(void)
// {
//     SEND_DART_CLIENT_CMD_DATA.time_stamp = HAL_GetTick();
//     SEND_DART_CLIENT_CMD_DATA.data.dart_launch_opening_status = DART_CLIENT_CMD->dart_launch_opening_status;
//     SEND_DART_CLIENT_CMD_DATA.data.reserved = DART_CLIENT_CMD->reserved;
//     SEND_DART_CLIENT_CMD_DATA.data.target_change_time = DART_CLIENT_CMD->target_change_time;
//     SEND_DART_CLIENT_CMD_DATA.data.latest_launch_cmd_time = DART_CLIENT_CMD->latest_launch_cmd_time;
//     append_CRC16_check_sum((uint8_t *)&SEND_DART_CLIENT_CMD_DATA, sizeof(SendDataDartClientCmd_s));
//     USB_Transmit((uint8_t *)&SEND_DART_CLIENT_CMD_DATA, sizeof(SendDataDartClientCmd_s));
// }

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_x = GROUND_ROBOT_POSITION->hero_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_y = GROUND_ROBOT_POSITION->hero_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_x = GROUND_ROBOT_POSITION->engineer_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_y = GROUND_ROBOT_POSITION->engineer_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_x = GROUND_ROBOT_POSITION->standard_3_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_y = GROUND_ROBOT_POSITION->standard_3_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_x = GROUND_ROBOT_POSITION->standard_4_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_y = GROUND_ROBOT_POSITION->standard_4_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved1 = GROUND_ROBOT_POSITION->reserved1;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved2 = GROUND_ROBOT_POSITION->reserved2;
    append_CRC16_check_sum((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送雷达标志数据
 * @param duration 发送周期
 */
// static void UsbSendRadarMarkData(void)
// {
//     SEND_RADAR_MARK_DATA.time_stamp = HAL_GetTick();
//     SEND_RADAR_MARK_DATA.data.mark_progress = RADAR_MARK_DATA->mark_progress;
//     append_CRC16_check_sum((uint8_t *)&SEND_RADAR_MARK_DATA, sizeof(SendDataRadarMarkData_s));
//     USB_Transmit((uint8_t *)&SEND_RADAR_MARK_DATA, sizeof(SendDataRadarMarkData_s));
// }

/**
 * @brief 发送哨兵信息数据
 * @param duration 发送周期
 */
// static void UsbSendSentryInfoData(void)
// {
//     SEND_SENTRY_INFO_DATA.time_stamp = HAL_GetTick();
//     SEND_SENTRY_INFO_DATA.data.sentry_info = SENTRY_INFO->sentry_info;
//     SEND_SENTRY_INFO_DATA.data.sentry_info_2 = SENTRY_INFO->sentry_info_2;
//     append_CRC16_check_sum((uint8_t *)&SEND_SENTRY_INFO_DATA, sizeof(SendDataSentryInfo_s));
//     USB_Transmit((uint8_t *)&SEND_SENTRY_INFO_DATA, sizeof(SendDataSentryInfo_s));
// }

/**
 * @brief 发送雷达信息数据
 * @param duration 发送周期
 */
// static void UsbSendRadarInfoData(void)
// {
//     SEND_RADAR_INFO_DATA.time_stamp = HAL_GetTick();
//     SEND_RADAR_INFO_DATA.data.radar_info = RADAR_INFO->radar_info;
//     append_CRC16_check_sum((uint8_t *)&SEND_RADAR_INFO_DATA, sizeof(SendDataRadarInfo_s));
//     USB_Transmit((uint8_t *)&SEND_RADAR_INFO_DATA, sizeof(SendDataRadarInfo_s));
// }

/**
 * @brief 发送机器人交互数据
 * @param duration 发送周期
 */
// static void UsbSendRobotInteractionData(void)
// {
//     SEND_ROBOT_INTERACTION_DATA.time_stamp = HAL_GetTick();
//     SEND_ROBOT_INTERACTION_DATA.data.data_cmd_id = ROBOT_INTERACTION_DATA->data_cmd_id;
//     SEND_ROBOT_INTERACTION_DATA.data.sender_id = ROBOT_INTERACTION_DATA->sender_id;
//     SEND_ROBOT_INTERACTION_DATA.data.receiver_id = ROBOT_INTERACTION_DATA->receiver_id;
//     memcpy(SEND_ROBOT_INTERACTION_DATA.data.user_data, ROBOT_INTERACTION_DATA->user_data, sizeof(SEND_ROBOT_INTERACTION_DATA.data.user_data));
//     append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_INTERACTION_DATA, sizeof(SendDataRobotInteraction_s));
//     USB_Transmit((uint8_t *)&SEND_ROBOT_INTERACTION_DATA, sizeof(SendDataRobotInteraction_s));
// }

/**
 * @brief 发送自定义控制器数据
 * @param duration 发送周期
 */
// static void UsbSendCustomControllerData(void)
// {
//     SEND_CUSTOM_CONTROLLER_DATA.time_stamp = HAL_GetTick();
//     memcpy(SEND_CUSTOM_CONTROLLER_DATA.data.data, CUSTOM_CONTROLLER_DATA->data, sizeof(SEND_CUSTOM_CONTROLLER_DATA.data.data));
//     append_CRC16_check_sum((uint8_t *)&SEND_CUSTOM_CONTROLLER_DATA, sizeof(SendDataCustomController_s));
//     USB_Transmit((uint8_t *)&SEND_CUSTOM_CONTROLLER_DATA, sizeof(SendDataCustomController_s));
// }

/**
 * @brief 发送地图命令数据
 * @param duration 发送周期
 */
// static void UsbSendMapCommandData(void)
// {
//     SEND_MAP_COMMAND_DATA.time_stamp = HAL_GetTick();
//     SEND_MAP_COMMAND_DATA.data.target_position_x = MAP_COMMAND->target_position_x;
//     SEND_MAP_COMMAND_DATA.data.target_position_y = MAP_COMMAND->target_position_y;
//     SEND_MAP_COMMAND_DATA.data.cmd_keyboard = MAP_COMMAND->cmd_keyboard;
//     SEND_MAP_COMMAND_DATA.data.target_robot_id = MAP_COMMAND->target_robot_id;
//     SEND_MAP_COMMAND_DATA.data.cmd_source = MAP_COMMAND->cmd_source;
//     append_CRC16_check_sum((uint8_t *)&SEND_MAP_COMMAND_DATA, sizeof(SendDataMapCommand_s));
//     USB_Transmit((uint8_t *)&SEND_MAP_COMMAND_DATA, sizeof(SendDataMapCommand_s));
// }

// /**
//  * @brief 发送机器人自定义数据
//  * @param duration 发送周期
//  */
// static void UsbSendRobotCustomData(void)
// {
//     SEND_ROBOT_CUSTOM_DATA.time_stamp = HAL_GetTick();
//     memcpy(SEND_ROBOT_CUSTOM_DATA.data.data, ROBOT_CUSTOM_DATA->data, sizeof(SEND_ROBOT_CUSTOM_DATA.data.data));
//     append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_CUSTOM_DATA, sizeof(SendDataRobotCustomData_s));
//     USB_Transmit((uint8_t *)&SEND_ROBOT_CUSTOM_DATA, sizeof(SendDataRobotCustomData_s));
// }

// /**
//  * @brief 发送机器人自定义数据3
//  * @param duration 发送周期
//  */
// static void UsbSendRobotCustom3Data(void)
// {
//     SEND_ROBOT_CUSTOM_DATA3.time_stamp = HAL_GetTick();
//     memcpy(SEND_ROBOT_CUSTOM_DATA3.data.data, ROBOT_CUSTOM_DATA3->data, sizeof(SEND_ROBOT_CUSTOM_DATA3.data.data));
//     append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_CUSTOM_DATA3, sizeof(SendDataRobotCustomData3_s));
//     USB_Transmit((uint8_t *)&SEND_ROBOT_CUSTOM_DATA3, sizeof(SendDataRobotCustomData3_s));
// }

// /**
//  * @brief 发送PidD数据
//  * @param duration 发送周期
//  */
// static void UsbSendPidData(void)
// {
//     SEND_DATA_PID.time_stamp = HAL_GetTick();
//     append_CRC16_check_sum((uint8_t *)&SEND_DATA_PID, sizeof(SendDataPidDebug_s));
//     USB_Transmit((uint8_t *)&SEND_DATA_PID, sizeof(SendDataPidDebug_s));
// }

/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = HAL_GetTick();
    SEND_JOINT_STATE_DATA.data.pitch = CmdGimbalJointState(AX_PITCH);
    SEND_JOINT_STATE_DATA.data.yaw = CmdGimbalJointState(AX_YAW);
    append_CRC16_check_sum((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
    USB_Transmit((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}
/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;

    ROBOT_CMD_DATA.tracking.tracking = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    SEND_DATA_DEBUG.packages[index].data = data;

    if (SEND_DATA_DEBUG.packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline uint8_t GetScCmdFire(void)
{
    return ROBOT_CMD_DATA.shoot.fire;
}


/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline uint8_t GetScCmdFricOn(void)
{
    return ROBOT_CMD_DATA.shoot.fric_on;
}

/**
 * @brief 获取上位机状态：发现敌方装甲板
 * @param void
 * @return bool 是否发现
 */
inline bool GetSCcmdtracking(void)
{
    return ROBOT_CMD_DATA.tracking.tracking;
}
/*------------------------------ End of File ------------------------------*/
