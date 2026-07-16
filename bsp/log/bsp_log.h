/**
 * @file bsp_log.h
 * @brief BSP日志接口,默认输出到SEGGER RTT,可选转发到USB CDC。
 */
#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include <stdint.h>

#define BUFFER_INDEX 0

#ifndef BSP_LOG_QUEUE_SIZE
#define BSP_LOG_QUEUE_SIZE 64U // 日志队列深度,任务来不及输出时新日志会被丢弃并计数
#endif

#ifndef BSP_LOG_LINE_SIZE
#define BSP_LOG_LINE_SIZE 160U // 单条日志最大长度,过长内容会被截断
#endif

#ifndef BSP_LOG_USE_USB
#define BSP_LOG_USE_USB 0U // 默认只走RTT,避免和视觉/VOFA等USB业务通道抢占
#endif

/**
 * @brief 日志系统初始化
 *
 */
void BSPLogInit(void);


/**
 * @brief 在任务上下文中输出队列中的日志
 *
 * @note 当前由BSPServiceTask在被事件唤醒或兜底超时唤醒后调用,一般不需要上层手动调用
 */
void BSPLogProcess(void);


/**
 * @brief 获取日志队列满导致的丢弃次数
 *
 * @return uint32_t 丢弃计数
 */
uint32_t BSPLogGetDroppedCount(void);


/**
 * @brief 日志功能原型,供下面的LOGI,LOGW,LOGE等使用
 *
 * @return int 格式化后的日志长度,失败返回负数
 */
int BSPLogPrintf(const char* type, const char* color, const char* format, ...);


#define LOG_PROTO(type, color, format, ...) BSPLogPrintf(type, color, format, ##__VA_ARGS__)

/*----------------------------------------下面是日志输出的接口-------------------------------------------------*/

/* 清屏 */
#define LOG_CLEAR() SEGGER_RTT_WriteString(BUFFER_INDEX, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/**
 *  有颜色格式日志输出,建议使用这些宏来输出日志
 *  @attention 注意这些接口不支持浮点格式化输出,建议换算成整数单位后打印
 *  @note Release配置默认定义DISABLE_LOG_SYSTEM后会关闭LOGINFO/LOGWARNING/LOGERROR。
 */
#if defined(DISABLE_LOG_SYSTEM) && (DISABLE_LOG_SYSTEM)
#define LOGINFO(format, ...)
#define LOGWARNING(format, ...)
#define LOGERROR(format, ...)
#else
// information level
#define LOGINFO(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
// warning level
#define LOGWARNING(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
// error level
#define LOGERROR(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif //  DISABLE_LOG_SYSTEM

/**
 * @brief 通过segger RTT打印日志,支持格式化输出,格式化输出的实现参考printf.
 * @attention !! 此函数不支持浮点格式化,建议换算成整数单位后打印 !!
 *
 * @param fmt 格式字符串
 * @param ... 参数列表
 * @return int 打印的log字符数
 */
int PrintLog(const char* fmt, ...);

#endif
