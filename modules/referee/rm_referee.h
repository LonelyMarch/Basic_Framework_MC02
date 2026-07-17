#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

typedef struct
{
    uint8_t Robot_Color; // 机器人颜色
    uint16_t Robot_ID; // 本机器人ID
    uint16_t Cilent_ID; // 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState; // 0x0001
    ext_game_result_t GameResult; // 0x0002
    ext_game_robot_HP_t GameRobotHP; // 0x0003
    ext_event_data_t EventData; // 0x0101
    ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
    ext_game_robot_state_t GameRobotState; // 0x0201
    ext_power_heat_data_t PowerHeatData; // 0x0202
    ext_game_robot_pos_t GameRobotPos; // 0x0203
    ext_buff_musk_t BuffMusk; // 0x0204
    aerial_robot_energy_t AerialRobotEnergy; // 0x0205
    ext_robot_hurt_t RobotHurt; // 0x0206
    ext_shoot_data_t ShootData; // 0x0207

    // 自定义交互数据的接收
    Communicate_ReceiveData_t ReceiveData;

    uint8_t init_flag;
} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t lid_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    chassis_mode_e chassis_mode; // 底盘模式
    gimbal_mode_e gimbal_mode; // 云台模式
    shoot_mode_e shoot_mode; // 发射模式设置
    friction_mode_e friction_mode; // 摩擦轮关闭
    lid_mode_e lid_mode; // 弹舱盖打开
    Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

    // 上一次的模式，用于flag判断
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    friction_mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;
} Referee_Interactive_info_t;

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t* RefereeInit(UART_HandleTypeDef * referee_usart_handle);

/**
 * @brief 获取裁判系统数据快照
 *
 * @note USART解析任务会更新内部数据,其他任务读取时建议使用该接口复制快照,
 *       避免直接读取内部结构体时遇到任务切换造成的半更新数据。
 *
 * @param snapshot 快照输出地址
 * @return uint8_t 1表示复制成功,0表示参数非法
 */
uint8_t RefereeGet(referee_info_t* snapshot);


/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 该接口只把交互帧放入裁判系统内部发送队列,不会阻塞等待发送节流。
 *       真正发送由RefereeTxProcess()在UI任务中按裁判系统频率限制逐帧处理。
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 * @return HAL_StatusTypeDef HAL_OK表示成功入队,HAL_BUSY表示发送队列已满
 */
HAL_StatusTypeDef RefereeSend(uint8_t* send, uint16_t tx_len);


/**
 * @brief 处理裁判系统发送队列
 *
 * @note 该函数需要在任务上下文周期调用。它会在USART空闲且满足裁判系统
 *       交互数据上行频率限制时,启动下一帧DMA发送。
 */
void RefereeTxProcess(void);

#endif // !REFEREE_H
