#ifndef BSP_SERVICE_H
#define BSP_SERVICE_H

#include "cmsis_os2.h"
#include <stdint.h>

#define BSP_SERVICE_EVENT_CNT 64U


typedef void (*BSPServiceEventCallback)(void* arg);


/**
 * @brief 初始化BSP服务事件队列
 *
 * @note 当前使用FreeRTOS静态Queue保存延后事件,由BSPInit()在系统启动早期调用。
 */
void BSPServiceInit(void);


/**
 * @brief BSP服务任务入口
 *
 * @note 该任务统一处理BSP层轻量延后事件,例如USART/USB接收解析和GPIO EXTI回调。
 *       CAN接收由独立的CANProcessTask处理,避免高速CAN回调挤占通用BSP服务任务。
 */
void BSPServiceTask(void* argument);


/**
 * @brief 从任务上下文投递一个BSP延后事件
 *
 * @param callback 任务上下文中执行的回调
 * @param arg      回调参数
 * @return uint8_t 1表示投递成功,0表示队列满或参数非法
 */
uint8_t BSPServicePost(BSPServiceEventCallback callback, void* arg);


/**
 * @brief 从中断上下文投递一个BSP延后事件
 *
 * @param callback 任务上下文中执行的回调
 * @param arg      回调参数
 * @return uint8_t 1表示投递成功,0表示队列满或参数非法
 */
uint8_t BSPServicePostFromISR(BSPServiceEventCallback callback, void* arg);


/**
 * @brief 唤醒BSP服务任务处理已有待处理数据
 *
 * @note USART/USB/log等模块已经有自己的缓冲区时,可以只调用本函数唤醒服务任务,
 *       不需要额外投递一个空事件。该接口可在任务或ISR上下文调用。
 */
void BSPServiceNotify(void);


/**
 * @brief 主动处理一次BSP服务事件
 *
 * @note 正常情况下由BSPServiceTask在被事件唤醒后调用。保留该接口便于后续单元化调度
 *       或在特定任务中手动泵一次BSP事件。
 */
void BSPServiceProcess(void);


/**
 * @brief 获取BSP服务事件队列溢出次数
 */
uint32_t BSPServiceGetDroppedEventCount(void);

#endif
