#include "super_cap.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include "memory.h"

#define SUPER_CAP_INSTANCE_COUNT 1U // 当前工程只会挂载一台超级电容,因此保持单实例静态池
#define SUPER_CAP_RX_LEN 6U         // 反馈帧: 电压2字节 + 电流2字节 + 功率2字节
#define SUPER_CAP_TX_LEN_MAX 8U     // 经典CAN单帧最大8字节
#define SUPER_CAP_DAEMON_COUNT 100U // 默认在线检测计数,与daemon模块默认值保持一致

static SuperCapInstance super_cap_pool[SUPER_CAP_INSTANCE_COUNT]; // 静态实例池,避免初始化阶段依赖堆分配
static uint8_t super_cap_idx;                                     // 当前已经分配的超级电容实例数量

static uint16_t SuperCapReadBe16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void SuperCapLostCallback(void *owner)
{
    SuperCapInstance *instance = (SuperCapInstance *)owner;

    if (instance == NULL)
    {
        return;
    }

    /*
     * Daemon离线后会周期性调用本回调,这里用lost_logged限流,
     * 避免超级电容长时间离线时持续输出同一条告警日志。
     */
    if (instance->lost_logged == 0U)
    {
        instance->lost_logged = 1U;
        LOGWARNING("[super_cap] offline, rx_id:0x%x",
                   instance->can_ins == NULL ? 0U : (unsigned int)instance->can_ins->rx_id);
    }
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    SuperCapInstance *instance;
    const uint8_t *rxbuff;

    if (_instance == NULL || _instance->id == NULL)
    {
        return;
    }

    instance = (SuperCapInstance *)_instance->id;
    if (_instance->rx_len < SUPER_CAP_RX_LEN)
    {
        instance->rx_error_count++;
        return;
    }

    rxbuff = _instance->rx_buff;
    instance->cap_msg.vol = SuperCapReadBe16(&rxbuff[0]);
    instance->cap_msg.current = SuperCapReadBe16(&rxbuff[2]);
    instance->cap_msg.power = SuperCapReadBe16(&rxbuff[4]);
    instance->update_flag = 1U;
    instance->lost_logged = 0U;
    if (instance->daemon != NULL)
    {
        DaemonReload(instance->daemon);
    }
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    SuperCapInstance *instance;
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;

    if (supercap_config == NULL || supercap_config->can_config.can_handle == NULL)
    {
        LOGERROR("[super_cap] init with invalid config");
        return NULL;
    }

    if (super_cap_idx >= SUPER_CAP_INSTANCE_COUNT)
    {
        LOGWARNING("[super_cap] only one super capacitor is supported");
        return &super_cap_pool[0];
    }

    instance = &super_cap_pool[super_cap_idx++];
    memset(instance, 0, sizeof(SuperCapInstance));

    /*
     * CANInstance的id字段保存上层模块实例指针。
     * 这样CAN回调不用依赖全局变量,即使当前只使用一台超级电容,归属关系也更清晰。
     */
    can_config = supercap_config->can_config;
    can_config.id = instance;
    can_config.can_module_callback = SuperCapRxCallback;
    instance->can_ins = CANRegister(&can_config);
    if (instance->can_ins == NULL)
    {
        LOGERROR("[super_cap] CAN register failed");
        super_cap_idx--;
        return NULL;
    }

    daemon_config.callback = SuperCapLostCallback;
    daemon_config.owner_id = instance;
    daemon_config.reload_count = supercap_config->daemon_count == 0U
                                     ? SUPER_CAP_DAEMON_COUNT
                                     : supercap_config->daemon_count;
    daemon_config.init_count = daemon_config.reload_count;
    instance->daemon = DaemonRegister(&daemon_config);
    if (instance->daemon == NULL)
    {
        LOGERROR("[super_cap] daemon register failed");
        super_cap_idx--;
        return NULL;
    }

    return instance;
}

uint8_t SuperCapSend(SuperCapInstance *instance, const uint8_t *data, uint8_t len)
{
    if (instance == NULL || instance->can_ins == NULL || data == NULL || len == 0U || len > SUPER_CAP_TX_LEN_MAX)
    {
        LOGERROR("[super_cap] send with invalid param, len:%u", (unsigned int)len);
        if (instance != NULL)
        {
            instance->tx_error_count++;
        }
        return 0U;
    }

    CANSetDLC(instance->can_ins, len);
    memcpy(instance->can_ins->tx_buff, data, len);
    if (CANTransmit(instance->can_ins, 1.0f) == 0U)
    {
        instance->tx_error_count++;
        LOGWARNING("[super_cap] CAN transmit failed, tx_id:0x%x", (unsigned int)instance->can_ins->tx_id);
        return 0U;
    }

    return 1U;
}

uint8_t SuperCapGet(SuperCapInstance *instance, SuperCap_Msg_s *msg)
{
    uint8_t has_update;
    int32_t kernel_lock = -1;

    if (instance == NULL || msg == NULL)
    {
        LOGERROR("[super_cap] get with invalid param");
        return 0U;
    }

    if (__get_IPSR() != 0U)
    {
        return 0U;
    }

    /*
     * SuperCapRxCallback()由CANProcessTask在任务上下文写入cap_msg。
     * 读取侧短暂锁住调度器,避免结构体复制过程中被CANProcessTask切换并更新数据。
     */
    if (osKernelGetState() == osKernelRunning)
    {
        kernel_lock = osKernelLock();
        if (kernel_lock < 0)
        {
            LOGERROR("[super_cap] kernel lock failed when get data");
            return 0U;
        }
    }

    has_update = instance->update_flag;
    if (has_update != 0U)
    {
        *msg = instance->cap_msg;
        instance->update_flag = 0U;
    }

    if (kernel_lock >= 0)
    {
        (void)osKernelRestoreLock(kernel_lock);
    }

    return has_update;
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    if (instance == NULL || instance->daemon == NULL)
    {
        return 0U;
    }

    return DaemonIsOnline(instance->daemon);
}
