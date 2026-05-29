#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "main.h"

#define HC05_DATASIZE 4U // HC05单帧最大有效载荷长度,根据上层协议需要修改

/**
 * @brief HC05通信数据结构体
 *
 * @note 该模块用于外接HC-05/HC-06一类串口蓝牙模块,不是板载蓝牙驱动。
 *       当前协议帧格式为:
 *       0xAA + len + payload[len] + checksum + 0x55
 *       len有效范围为1~HC05_DATASIZE。
 *       checksum为len和payload逐字节异或的结果。
 */
typedef struct
{
    uint8_t send_data[HC05_DATASIZE + 4U]; // 最近一次发送的完整协议帧
    uint8_t recv_data[HC05_DATASIZE];      // 最近一次接收到的有效载荷
    volatile uint8_t recv_len;             // 最近一次有效载荷长度
} HC05;

// HC05串口接收初始化
HC05 *HC05Init(UART_HandleTypeDef *hc05_usart_handle);

/**
 * @brief 拷贝最近一次收到的HC05有效载荷
 *
 * @param data 上层提供的目标缓冲区
 * @param max_len 目标缓冲区最大长度
 * @return uint8_t 实际拷贝的数据长度,返回0表示暂无有效数据或参数错误
 */
uint8_t HC05_GetData(uint8_t *data, uint8_t max_len);

/**
 * @brief HC05串口发送函数,一次发送1~HC05_DATASIZE个字节
 *
 * @return HAL_OK表示发送已启动,HAL_BUSY表示串口仍在发送上一帧,HAL_ERROR表示参数或状态错误
 */
HAL_StatusTypeDef HC05_SendData(const uint8_t *data, uint8_t data_num);

#endif
