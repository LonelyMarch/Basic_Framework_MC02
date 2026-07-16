#ifndef MONITOR_H
#define MONITOR_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MX_CNT 64
#define DAEMON_TASK_PERIOD_MS 10U      // DaemonTask调度周期,application层需按该周期调用DaemonTask()
#define DAEMON_DEFAULT_TIMEOUT_MS 1000U // 未配置reload_count时的默认超时时间
#define DAEMON_DEFAULT_COUNT \
    ((DAEMON_DEFAULT_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1U) / DAEMON_TASK_PERIOD_MS)


/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void*);


/* daemon结构体定义 */
typedef struct daemon_ins
{
    uint16_t reload_count; // 重载值
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    uint16_t temp_count; // 当前值,减为零说明模块离线或异常
    void* owner_id; // daemon实例的地址,初始化的时候填入
} DaemonInstance;

/* daemon初始化配置 */
typedef struct
{
    uint16_t reload_count; // 实际上这是app唯一需要设置的值?
    uint16_t init_count; // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    void* owner_id; // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;

/**
 * @brief 注册一个daemon实例
 * @note 离线callback由DaemonTask()在daemon任务上下文执行,不是中断上下文。
 *       callback应保持短小,只做状态标记、一次性日志或轻量恢复;
 *       不建议在callback中长时间阻塞、忙等或循环重试。
 *       需要周期性重连的模块应在自身任务中根据DaemonIsOnline()做节流恢复。
 *
 * @param config 初始化配置
 * @return DaemonInstance* 返回实例指针
 */
DaemonInstance* DaemonRegister(Daemon_Init_Config_s* config);


/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance daemon实例指针
 */
void DaemonReload(DaemonInstance* instance);


/**
 * @brief 确认模块是否离线
 *
 * @param instance daemon实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t DaemonIsOnline(DaemonInstance* instance);


/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask();

#endif // !MONITOR_H
