/**
 * @file bsp_usb.h
 * @brief USB CDC虚拟串口BSP封装,用于调试输出、VOFA数据和可选日志后端。
 */
#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdint.h>
#include "main.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#define USB_TX_BUFFER_SIZE 2048U         // 单次USB CDC发送最大缓存长度
#define USB_RX_BUFFER_SIZE 2048U         // 单次USB CDC接收最大缓存长度
#define USB_PRINTF_BUFFER_SIZE 512U      // USBPrintf格式化临时缓存长度
#define USB_RX_FRAME_BUFFER_CNT 2U       // 双接收缓存,避免任务解析时被下一帧覆盖

typedef void (*USBRxCallback)(const uint8_t *buf, uint16_t len);
typedef void (*USBTxCallback)(void);

typedef struct
{
    USBRxCallback rx_callback; // 接收回调,由USBProcess()在任务上下文调用
    USBTxCallback tx_callback; // 发送完成回调,由BSP服务任务在任务上下文调用
} USB_Init_Config_s;

/**
 * @brief 初始化USB CDC BSP层。
 * @note 该函数只注册BSP回调和清理状态,USB设备栈仍由MX_USB_DEVICE_Init()启动。
 *
 * @param usb_conf 回调配置,不需要接收/发送完成回调时可传NULL
 * @return USBD_StatusTypeDef USBD_OK表示BSP层初始化成功
 */
USBD_StatusTypeDef USBInit(const USB_Init_Config_s *usb_conf);

/**
 * @brief 判断USB CDC是否已枚举且当前没有发送任务。
 *
 * @return uint8_t ready 1, busy/not configured 0
 */
uint8_t USBIsReady(void);

/**
 * @brief 通过USB CDC发送一帧数据。
 * @note 内部会复制到BSP发送缓存,调用者可以传局部buffer。
 *
 * @param buffer 待发送数据
 * @param len 数据长度
 * @return USBD_StatusTypeDef USBD_OK表示已启动发送,USBD_BUSY表示USB忙或未枚举
 */
USBD_StatusTypeDef USBTransmit(const uint8_t *buffer, uint16_t len);

/**
 * @brief 格式化并通过USB CDC发送字符串,适合调试信息和VOFA文本协议。
 *
 * @param fmt printf风格格式字符串
 * @return USBD_StatusTypeDef USBD_OK表示已启动发送
 */
USBD_StatusTypeDef USBPrintf(const char *fmt, ...);

/**
 * @brief 在任务上下文处理USB接收数据。
 * @note 不能在中断中调用。当前由BSPServiceTask在被事件唤醒或兜底超时唤醒后调用。
 */
void USBProcess(void);

/**
 * @brief 获取USB接收任务来不及处理时丢弃的帧数。
 *
 * @return uint8_t 丢帧计数
 */
uint8_t USBGetDroppedFrameCount(void);

/**
 * @brief USB CDC底层接收完成桥接函数,由usbd_cdc_if.c调用。
 */
void USB_CDC_RxCpltCallback(uint8_t *buf, uint32_t len);

/**
 * @brief USB CDC底层发送完成桥接函数,由usbd_cdc_if.c调用。
 */
void USB_CDC_TxCpltCallback(void);

#endif
