/**
 * @file can_comm.h
 * @author Neo neozng1@hnu.edu.cn
 * @brief  用于多机CAN通信的收发模块
 * @version 0.1
 * @date 2022-11-27
 *
 * @copyright Copyright (c) 2022 HNUYueLu EC all rights reserved
 *
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"

#define MX_CAN_COMM_COUNT 4 // 注意均衡负载,一条总线上不要挂载过多的外设

#define CAN_COMM_MAX_BUFFSIZE 60 // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'      // 帧头
#define CAN_COMM_TAIL 'e'        // 帧尾
#define CAN_COMM_HEADER_INDEX 0U // 帧头所在位置
#define CAN_COMM_SEQ_INDEX 1U    // 包序号所在位置,用于检测整包跳变和丢包
#define CAN_COMM_LEN_INDEX 2U    // 数据长度所在位置
#define CAN_COMM_DATA_OFFSET 3U  // 有效数据起始位置
#define CAN_COMM_OFFSET_BYTES 5U // 's' + seq + datalen + crc8 + 'e'
#define CAN_COMM_RX_TIMEOUT_MS 5.0f // 半包接收超时时间,超过该时间仍未收齐则丢弃当前半包

/* CAN comm 运行期结构体, 只在MCU内部保存状态,不直接作为CAN协议帧发送,因此保持编译器默认对齐。 */
typedef struct
{
    CANInstance* can_ins;
    /* 发送部分 */
    uint8_t send_data_len; // 发送数据长度
    uint8_t send_buf_len; // 发送缓冲区长度,为发送数据长度+帧头、序号、长度、校验和、帧尾
    uint8_t tx_seq; // 发送包序号,每成功组织一个CANComm整包后自增
    uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 保存帧头、序号、长度、有效数据、校验和、帧尾
    /* 接收部分 */
    uint8_t recv_data_len; // 接收数据长度
    uint8_t recv_buf_len; // 接收缓冲区长度,为接收数据长度+帧头、序号、长度、校验和、帧尾
    uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 保存帧头、序号、长度、有效数据、校验和、帧尾
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE]; // 解包后的数据,CANCommGet()会把这里的数据复制到调用者提供的缓存
    /* 接收和更新标志位*/
    uint8_t recv_state; // 接收状态,
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    uint8_t update_flag; // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0
    uint8_t rx_seq; // 当前正在接收的包序号
    uint8_t last_rx_seq; // 上一次成功接收的包序号
    uint8_t has_rx_seq; // 是否已经成功接收过至少一个包
    uint8_t lost_logged; // 离线日志是否已经打印过,避免Daemon超时后持续刷日志
    float rx_start_time; // 当前半包开始接收的时间,单位ms

    DaemonInstance* comm_daemon;

    uint32_t rx_len_error_count; // 长度错误计数
    uint32_t rx_crc_error_count; // CRC错误计数
    uint32_t rx_tail_error_count; // 帧尾错误计数
    uint32_t rx_seq_jump_count; // 包序号跳变事件计数,用于粗略观察整包丢失
    uint32_t rx_timeout_count; // 半包超时计数
} CANCommInstance;

/* CAN comm 初始化结构体 */
typedef struct
{
    CAN_Init_Config_s can_config; // CAN初始化结构体
    uint8_t send_data_len; // 发送数据长度
    uint8_t recv_data_len; // 接收数据长度

    uint16_t daemon_count; // 守护进程计数,用于初始化守护进程
} CANComm_Init_Config_s;

/**
 * @brief 初始化CANComm
 *
 * @param config CANComm初始化结构体
 * @return CANCommInstance*
 */
CANCommInstance* CANCommInit(CANComm_Init_Config_s* comm_config);


/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 * @return uint8_t 1表示整包所有CAN子帧均已成功提交发送,0表示参数错误或任意子帧发送失败
 */
uint8_t CANCommSend(CANCommInstance* instance, const uint8_t* data);


/**
 * @brief 获取CANComm接收的新数据
 *
 * @param instance cancomm实例
 * @param data 接收缓存地址,有效空间必须不小于初始化时设置的recv_data_len
 * @return uint8_t 1表示本次读取到了新数据,0表示没有新数据或参数错误
 * @attention CANComm内部会按字节复制数据,强烈建议通过CANComm传输的结构体使用pack(1),
 *            否则结构体填充字节可能导致两端解释出的字段不一致。
 */
uint8_t CANCommGet(CANCommInstance* instance, void* data);


/**
 * @brief 检查CANComm是否在线
 *
 * @param instance
 * @return uint8_t
 */
uint8_t CANCommIsOnline(CANCommInstance* instance);

#endif // !CAN_COMM_H
