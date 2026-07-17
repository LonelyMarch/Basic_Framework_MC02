/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "daemon.h"

typedef struct
{
    uint16_t vol; // 超级电容电压,原始单位由电容固件协议决定
    uint16_t current; // 超级电容电流,原始单位由电容固件协议决定
    uint16_t power; // 超级电容功率,原始单位由电容固件协议决定
} SuperCap_Msg_s;

/* 超级电容实例 */
typedef struct
{
    CANInstance* can_ins; // CAN实例
    DaemonInstance* daemon; // 在线检测daemon,收到合法反馈帧后会被重载
    SuperCap_Msg_s cap_msg; // 最近一次解析出的超级电容反馈信息
    uint8_t update_flag; // 新数据标志,SuperCapGet()成功取走后清零
    uint8_t lost_logged; // 离线日志标志,避免daemon离线期间持续刷屏
    uint32_t rx_error_count; // 接收长度或参数异常计数
    uint32_t tx_error_count; // 发送失败计数
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
    uint16_t daemon_count; // 在线检测重载计数,为0时使用daemon默认值
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance* SuperCapInit(SuperCap_Init_Config_s* supercap_config);


/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 * @param len 发送长度,最大8字节
 * @return uint8_t 1表示发送成功提交到CAN发送FIFO,0表示参数错误或发送失败
 */
uint8_t SuperCapSend(SuperCapInstance* instance, const uint8_t* data, uint8_t len);


/**
 * @brief 获取最近一次超级电容反馈信息
 *
 * @param instance 超级电容实例
 * @param msg 输出缓存
 * @return uint8_t 1表示本次取到新数据,0表示无新数据或参数错误
 */
uint8_t SuperCapGet(SuperCapInstance* instance, SuperCap_Msg_s* msg);


/**
 * @brief 判断超级电容是否在线
 *
 * @param instance 超级电容实例
 * @return uint8_t 1表示在线,0表示离线或实例无效
 */
uint8_t SuperCapIsOnline(SuperCapInstance* instance);

#endif // !SUPER_CAP_H
