#include "can_comm.h"
#include "memory.h"
#include "crc8.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os2.h"

static CANCommInstance can_comm_pool[MX_CAN_COMM_COUNT]; // CANComm实例静态池,避免初始化阶段依赖堆分配
static uint8_t can_comm_idx; // 当前已经分配的CANComm实例数量

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance* ins)
{
    if (ins == NULL)
    {
        return;
    }

    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state = 0; // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
    ins->rx_start_time = 0.0f;
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 */
static void CANCommRxCallback(CANInstance* _instance)
{
    if (_instance == NULL)
    {
        return;
    }

    CANCommInstance* comm = (CANCommInstance*)_instance->id; // 注意写法,将can instance的id强制转换为CANCommInstance*类型
    uint8_t is_new_packet;
    uint8_t crc8;
    uint8_t expected_seq;

    if (comm == NULL)
    {
        return;
    }

    /*
     * 收到帧头后,同一个CANComm整包的后续子帧应该在很短时间内连续到达。
     * 若半包停留过久,说明中间子帧大概率丢失,这里主动丢弃半包并等待下一包帧头重新同步。
     */
    if (comm->recv_state != 0U &&
        (DWT_GetTimeline_ms() - comm->rx_start_time) > CAN_COMM_RX_TIMEOUT_MS)
    {
        comm->rx_timeout_count++;
        CANCommResetRx(comm);
    }

    /* 当前接收状态判断 */
    is_new_packet = (comm->recv_state == 0U &&
        _instance->rx_len >= CAN_COMM_DATA_OFFSET &&
        _instance->rx_buff[CAN_COMM_HEADER_INDEX] == CAN_COMM_HEADER);
    if (is_new_packet != 0U)
    {
        if (_instance->rx_buff[CAN_COMM_LEN_INDEX] == comm->recv_data_len) // 当前暂不支持动态包长,长度必须和初始化配置一致
        {
            CANCommResetRx(comm); // 新帧头到来时重新同步,避免上一包丢帧后继续错拼
            comm->recv_state = 1; // 设置接收状态为1,说明已经开始接收
            comm->rx_seq = _instance->rx_buff[CAN_COMM_SEQ_INDEX];
            comm->rx_start_time = DWT_GetTimeline_ms();
        }
        else
        {
            comm->rx_len_error_count++;
            return; // 直接跳过即可
        }
    }

    if (comm->recv_state) // 已经收到过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            comm->rx_len_error_count++;
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
            {
                // 通过校验,复制数据到unpack_data中
                crc8 = crc_8(comm->raw_recvbuf + CAN_COMM_SEQ_INDEX, (uint16_t)comm->recv_data_len + 2U);
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc8)
                {
                    // 数据量大的话考虑使用DMA
                    if (comm->has_rx_seq != 0U)
                    {
                        expected_seq = (uint8_t)(comm->last_rx_seq + 1U);
                        if (comm->rx_seq != expected_seq)
                        {
                            comm->rx_seq_jump_count++;
                        }
                    }
                    comm->last_rx_seq = comm->rx_seq;
                    comm->has_rx_seq = 1U;

                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + CAN_COMM_DATA_OFFSET, comm->recv_data_len);
                    comm->update_flag = 1; // 数据更新flag置为1
                    comm->lost_logged = 0U; // 收到合法整包后,允许下一次离线时重新打印告警
                    DaemonReload(comm->comm_daemon); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
                }
                else
                {
                    comm->rx_crc_error_count++;
                }
            }
            else
            {
                comm->rx_tail_error_count++;
            }
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }
    }
}

static void CANCommLostCallback(void* cancomm)
{
    CANCommInstance* comm = (CANCommInstance*)cancomm;

    if (comm == NULL)
    {
        return;
    }

    CANCommResetRx(comm);
    if (comm->lost_logged == 0U)
    {
        comm->lost_logged = 1U;
        LOGWARNING("[can_comm] can comm rx[0x%x] lost, reset rx state.",
                   comm->can_ins == NULL ? 0U : (unsigned int)comm->can_ins->rx_id);
    }
}

CANCommInstance* CANCommInit(CANComm_Init_Config_s* comm_config)
{
    CAN_Init_Config_s can_config;
    CANCommInstance* ins;

    if (comm_config == NULL || comm_config->can_config.can_handle == NULL)
    {
        LOGERROR("[can_comm] init with invalid config");
        return NULL;
    }

    if (comm_config->send_data_len == 0U || comm_config->send_data_len > CAN_COMM_MAX_BUFFSIZE ||
        comm_config->recv_data_len == 0U || comm_config->recv_data_len > CAN_COMM_MAX_BUFFSIZE)
    {
        LOGERROR("[can_comm] invalid data length, send:%u, recv:%u, limit:%u",
                 (unsigned int)comm_config->send_data_len,
                 (unsigned int)comm_config->recv_data_len,
                 (unsigned int)CAN_COMM_MAX_BUFFSIZE);
        return NULL;
    }

    if (can_comm_idx >= MX_CAN_COMM_COUNT)
    {
        LOGERROR("[can_comm] instance exceeded, used:%u, limit:%u",
                 (unsigned int)can_comm_idx,
                 (unsigned int)MX_CAN_COMM_COUNT);
        return NULL;
    }

    ins = &can_comm_pool[can_comm_idx++];
    memset(ins, 0, sizeof(CANCommInstance));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES; // head + seq + datalen + crc8 + tail
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[CAN_COMM_HEADER_INDEX] = CAN_COMM_HEADER; // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ins->raw_sendbuf[CAN_COMM_LEN_INDEX] = comm_config->send_data_len; // datalen
    ins->raw_sendbuf[comm_config->send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;
    // can instance的设置
    can_config = comm_config->can_config;
    can_config.id = ins; // CANComm的实例指针作为CANInstance的id,回调函数中会用到
    can_config.can_module_callback = CANCommRxCallback;
    ins->can_ins = CANRegister(&can_config);
    if (ins->can_ins == NULL)
    {
        LOGERROR("[can_comm] CAN register failed");
        return NULL;
    }

    Daemon_Init_Config_s daemon_config = {
        .callback = CANCommLostCallback,
        .owner_id = (void*)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    if (ins->comm_daemon == NULL)
    {
        LOGERROR("[can_comm] daemon register failed");
        return NULL;
    }
    return ins;
}

uint8_t CANCommSend(CANCommInstance* instance, const uint8_t* data)
{
    uint8_t crc8;
    uint8_t send_len;

    if (instance == NULL || data == NULL || instance->can_ins == NULL)
    {
        LOGERROR("[can_comm] send with invalid param");
        return 0;
    }

    /*
     * CANComm整包格式:
     * 's' + seq + len + data + crc8(seq/len/data) + 'e'
     * seq用于观察整包跳变,CRC覆盖seq/len/data,避免长度字节被破坏后仍只按data校验。
     */
    instance->raw_sendbuf[CAN_COMM_SEQ_INDEX] = instance->tx_seq++;
    instance->raw_sendbuf[CAN_COMM_LEN_INDEX] = instance->send_data_len;
    memcpy(instance->raw_sendbuf + CAN_COMM_DATA_OFFSET, data, instance->send_data_len);
    crc8 = crc_8(instance->raw_sendbuf + CAN_COMM_SEQ_INDEX, (uint16_t)instance->send_data_len + 2U);
    instance->raw_sendbuf[CAN_COMM_DATA_OFFSET + instance->send_data_len] = crc8;
    instance->raw_sendbuf[CAN_COMM_DATA_OFFSET + instance->send_data_len + 1U] = CAN_COMM_TAIL;

    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    {
        // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        if (CANTransmit(instance->can_ins, 1) == 0U)
        {
            return 0;
        }
    }

    return 1;
}

uint8_t CANCommGet(CANCommInstance* instance, void* data)
{
    uint8_t has_update;
    int32_t kernel_lock = -1;

    if (instance == NULL || data == NULL)
    {
        LOGERROR("[can_comm] get with invalid param");
        return 0;
    }

    if (__get_IPSR() != 0U)
    {
        return 0;
    }

    /*
     * CANCommRxCallback()由CANProcessTask在任务上下文写入unpacked_recv_data。
     * 读取侧短暂锁住调度器,避免拷贝结构体时被CANProcessTask切入并写入新数据,
     * 从而出现前半段旧数据、后半段新数据的撕裂读取。
     */
    if (osKernelGetState() == osKernelRunning)
    {
        kernel_lock = osKernelLock();
        if (kernel_lock < 0)
        {
            LOGERROR("[can_comm] kernel lock failed when get data");
            return 0;
        }
    }

    has_update = instance->update_flag;
    if (has_update != 0U)
    {
        memcpy(data, instance->unpacked_recv_data, instance->recv_data_len);
        instance->update_flag = 0; // 成功读取后清除更新标志,下一次无新帧时返回0
    }

    if (kernel_lock >= 0)
    {
        (void)osKernelRestoreLock(kernel_lock);
    }

    return has_update;
}

uint8_t CANCommIsOnline(CANCommInstance* instance)
{
    if (instance == NULL || instance->comm_daemon == NULL)
    {
        return 0U;
    }

    return DaemonIsOnline(instance->comm_daemon);
}
