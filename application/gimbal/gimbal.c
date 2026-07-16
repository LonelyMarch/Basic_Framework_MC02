/**
 * @file gimbal.c
 * @brief 云台 APP 的无业务迁移骨架。
 */

#include "gimbal.h"

/**
 * @brief 初始化云台 APP。
 */
void GimbalInit(void)
{
    /*
     * 迁移入口：注册云台命令订阅者、反馈发布者、IMU 数据源和云台电机实例。
     * 电机控制模式及 PID 参数应在电机实例注册时固定配置。
     */
}

/**
 * @brief 执行一次云台控制更新。
 */
void GimbalTask(void)
{
    /*
     * 迁移入口：读取最新命令和姿态反馈，更新云台状态机，并设置电机目标值。
     * 实际通信发送由统一电机管理任务完成，本函数不直接发送 CAN/RS485 报文。
     */
}
