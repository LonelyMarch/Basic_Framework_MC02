/**
 * @file bsp_usb.c
 * @brief USB CDC BSP封装,主要用于调试信息/VOFA数据输出
 */

#include "bsp_usb.h"
#include "bsp_frame_queue.h"
#include "bsp_log.h"
#include "bsp_service.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    USBRxCallback rx_callback; // 接收解析回调,在USBProcess()任务上下文执行
    USBTxCallback tx_callback; // 发送完成回调,由BSP服务任务在任务上下文执行
    volatile uint8_t initialized; // BSP注册完成标志
    volatile uint8_t tx_busy; // 发送忙标志,防止上一次USB包未发完时覆盖发送缓冲
    BSPFrameQueue rx_queue; // USB接收帧缓存队列,中断写入,任务读取
} USBInstance;

static USBInstance usb_instance;

/*
 * USB OTG HS当前配置为Full Speed Device Only,PCD DMA关闭,收发数据由CPU搬运到USB FIFO。
 * 因此这里的缓冲区不强制放到.dma_buffer段。若后续开启USB DMA,需要再按H7内存域重审。
 */
static uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
static uint8_t usb_printf_buffer[USB_PRINTF_BUFFER_SIZE];
static uint8_t usb_rx_buffer[USB_RX_FRAME_BUFFER_CNT][USB_RX_BUFFER_SIZE];
static uint16_t usb_rx_len[USB_RX_FRAME_BUFFER_CNT];

static uint8_t USBTryAcquireTx(void)
{
    uint8_t acquired = 0;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (usb_instance.tx_busy == 0)
    {
        usb_instance.tx_busy = 1;
        acquired = 1;
    }
    __set_PRIMASK(primask);

    return acquired;
}

static void USBDispatchTxCallback(void* arg)
{
    (void)arg;

    if (usb_instance.tx_callback != NULL)
    {
        usb_instance.tx_callback();
    }
}

static void USBReleaseTx(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    usb_instance.tx_busy = 0;
    __set_PRIMASK(primask);
}

USBD_StatusTypeDef USBInit(const USB_Init_Config_s* usb_conf)
{
    memset(&usb_instance, 0, sizeof(usb_instance));
    memset(usb_tx_buffer, 0, sizeof(usb_tx_buffer));
    memset(usb_printf_buffer, 0, sizeof(usb_printf_buffer));
    BSPFrameQueueInit(&usb_instance.rx_queue,
                      &usb_rx_buffer[0][0],
                      usb_rx_len,
                      USB_RX_FRAME_BUFFER_CNT,
                      USB_RX_BUFFER_SIZE);

    if (usb_conf != NULL)
    {
        usb_instance.rx_callback = usb_conf->rx_callback;
        usb_instance.tx_callback = usb_conf->tx_callback;
    }

    /*
     * MX_USB_DEVICE_Init()由CubeMX默认任务在FreeRTOS启动后调用。
     * 这里不重复初始化USB外设,只完成BSP层回调注册和状态清理。
     */
    usb_instance.initialized = 1;
    LOGINFO("[bsp_usb] USB CDC BSP init done");
    return USBD_OK;
}

uint8_t USBIsReady(void)
{
    extern USBD_HandleTypeDef hUsbDeviceHS;

    return (usb_instance.initialized != 0) &&
        (usb_instance.tx_busy == 0) &&
        (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) &&
        (hUsbDeviceHS.pClassData != NULL);
}

USBD_StatusTypeDef USBTransmit(const uint8_t* buffer, uint16_t len)
{
    uint8_t status;

    if (buffer == NULL || len == 0)
        return USBD_FAIL;

    if (len > USB_TX_BUFFER_SIZE)
    {
        LOGERROR("[bsp_usb] tx len [%d] exceed limit [%d]", len, USB_TX_BUFFER_SIZE);
        return USBD_FAIL;
    }

    if (usb_instance.initialized == 0)
        return USBD_BUSY;

    if (USBTryAcquireTx() == 0)
        return USBD_BUSY;

    /*
     * USB CDC发送是异步完成的,上层传入的buffer可能是局部变量。
     * 先复制到BSP内部发送缓冲,确保函数返回后数据仍然有效。
     */
    memcpy(usb_tx_buffer, buffer, len);
    status = CDC_Transmit_HS(usb_tx_buffer, len);
    if (status != USBD_OK)
    {
        USBReleaseTx();
        if (status != USBD_BUSY)
            LOGWARNING("[bsp_usb] CDC transmit failed, status [%d]", status);
    }

    return (USBD_StatusTypeDef)status;
}

USBD_StatusTypeDef USBPrintf(const char* fmt, ...)
{
    int len;
    va_list args;

    if (fmt == NULL)
        return USBD_FAIL;

    va_start(args, fmt);
    len = vsnprintf((char*)usb_printf_buffer, sizeof(usb_printf_buffer), fmt, args);
    va_end(args);

    if (len <= 0)
        return USBD_FAIL;

    if ((uint32_t)len >= sizeof(usb_printf_buffer))
        len = sizeof(usb_printf_buffer) - 1U;

    return USBTransmit(usb_printf_buffer, (uint16_t)len);
}

void USBProcess(void)
{
    uint8_t* recv_buff;
    uint16_t recv_len;

    while (BSPFrameQueuePeek(&usb_instance.rx_queue, &recv_buff, &recv_len) != 0U)
    {
        if (usb_instance.rx_callback != NULL && recv_len > 0)
        {
            // 接收回调在任务上下文执行,可以进行协议解析和普通模块状态更新。
            usb_instance.rx_callback(recv_buff, recv_len);
        }

        memset(recv_buff, 0, recv_len);
        BSPFrameQueuePop(&usb_instance.rx_queue);
    }
}

uint8_t USBGetDroppedFrameCount(void)
{
    return (uint8_t)BSPFrameQueueDroppedCount(&usb_instance.rx_queue);
}

void USB_CDC_RxCpltCallback(uint8_t* buf, uint32_t len)
{
    uint16_t copy_len;

    if (usb_instance.initialized == 0 || buf == NULL || len == 0)
        return;

    copy_len = len > USB_RX_BUFFER_SIZE ? USB_RX_BUFFER_SIZE : (uint16_t)len;
    if (BSPFrameQueuePush(&usb_instance.rx_queue, buf, copy_len) == 0U)
        return;

    // USB底层只保存帧数据,随后唤醒BSP服务任务在任务上下文执行协议解析。
    BSPServiceNotify();
}

void USB_CDC_TxCpltCallback(void)
{
    USBReleaseTx();
    if (usb_instance.tx_callback != NULL)
    {
        (void)BSPServicePostFromISR(USBDispatchTxCallback, NULL);
    }
}
