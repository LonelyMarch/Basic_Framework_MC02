#include "daemon.h"
#include "bsp_log.h"
#include "main.h"
#include "memory.h"

// 用于保存所有的daemon instance
static DaemonInstance daemon_instance_pool[DAEMON_MX_CNT]; // daemon实例静态池,避免初始化阶段依赖heap分配
static DaemonInstance* daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

DaemonInstance* DaemonRegister(Daemon_Init_Config_s* config)
{
    DaemonInstance* instance;

    if (config == NULL)
    {
        LOGERROR("[daemon] register with null config");
        return NULL;
    }

    if (idx >= DAEMON_MX_CNT)
    {
        LOGERROR("[daemon] instance count exceeded, used:%u, limit:%u",
                 (unsigned int)idx,
                 (unsigned int)DAEMON_MX_CNT);
        return NULL;
    }

    instance = &daemon_instance_pool[idx];
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0U ? DAEMON_DEFAULT_COUNT : config->reload_count;
    instance->callback = config->callback;
    /*
     * init_count用于给刚启动的模块预留上线等待时间。
     * 未显式设置时,默认等于reload_count,避免注册后还没来得及收到第一帧就被判离线。
     */
    instance->temp_count = config->init_count == 0U ? instance->reload_count : config->init_count;

    daemon_instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance* instance)
{
    if (instance == NULL)
        return;

    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance* instance)
{
    if (instance == NULL)
        return 0U;

    return instance->temp_count > 0;
}

void DaemonTask()
{
    DaemonInstance* dins; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {
        uint8_t should_callback;
        uint32_t primask;

        dins = daemon_instances[i];

        /*
         * temp_count--是读-改-写复合操作,可能和其他任务中的DaemonReload()交错。
         * 临界区只覆盖计数判断和递减,离线回调必须放在临界区外执行。
         */
        primask = __get_PRIMASK();
        __disable_irq();
        if (dins->temp_count > 0U) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
        {
            dins->temp_count--;
            should_callback = 0U;
        }
        else
        {
            should_callback = 1U;
        }
        __set_PRIMASK(primask);

        if (should_callback != 0U && dins->callback != NULL) // 等于零说明超时了,调用回调函数(如果有的话)
        {
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}

// (需要id的原因是什么?) 下面是copilot的回答!
// 需要id的原因是因为有些module可能有多个实例,而我们需要知道具体是哪个实例offline
// 如果只有一个实例,则可以不用id,直接调用callback即可
// 比如: 有一个module叫做"电机",它有两个实例,分别是"电机1"和"电机2",那么我们调用电机的离线处理函数时就需要知道是哪个电机offline
