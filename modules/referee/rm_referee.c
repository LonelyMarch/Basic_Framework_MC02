/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define REFEREE_FRAME_MAX_SIZE RE_RX_BUFFER_SIZE
#define REFEREE_SEND_INTERVAL_MS 115U // 裁判系统交互数据上行频率最大约10Hz,发送后按协议节流
#define REFEREE_TX_QUEUE_DEPTH 32U    // UI初始化一次最多会连续投递约15帧,这里预留足够余量
#define REFEREE_TX_FRAME_MAX_SIZE 128U // 7图形刷新约120字节,字符刷新和删除帧更短

static USARTInstance* referee_usart_instance; // 裁判系统串口实例
static DaemonInstance* referee_daemon; // 裁判系统守护进程
static referee_info_t referee_info; // 裁判系统数据
static uint8_t referee_lost_reported; // 离线期间只打印一次日志,避免daemon周期回调刷屏

typedef struct
{
    uint16_t len; // 当前槽位中待发送帧的实际长度
    uint8_t data[REFEREE_TX_FRAME_MAX_SIZE]; // 裁判系统交互帧完整缓存
} RefereeTxFrame_t;

static RefereeTxFrame_t referee_tx_queue[REFEREE_TX_QUEUE_DEPTH];
static volatile uint8_t referee_tx_head; // 下一帧发送位置
static volatile uint8_t referee_tx_tail; // 下一帧写入位置
static volatile uint8_t referee_tx_count; // 队列中等待发送的帧数
static uint32_t referee_last_tx_tick; // 上一次真正启动发送的系统tick
static uint8_t referee_tx_started; // 是否已经发送过第一帧,第一帧允许立即发送

static void RefereeTxQueueReset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    referee_tx_head = 0U;
    referee_tx_tail = 0U;
    referee_tx_count = 0U;
    referee_last_tx_tick = 0U;
    referee_tx_started = 0U;
    __set_PRIMASK(primask);
}

static HAL_StatusTypeDef RefereeTxQueuePush(const uint8_t* send, uint16_t tx_len)
{
    uint32_t primask;

    if (send == NULL || tx_len == 0U || tx_len > REFEREE_TX_FRAME_MAX_SIZE)
        return HAL_ERROR;

    /*
	 * 队列会被UI任务和其他可能的模块任务投递,用极短临界区保护索引和槽位写入。
	 * 单帧最大128字节,复制时间很短,不会形成明显的实时性负担。
	 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (referee_tx_count >= REFEREE_TX_QUEUE_DEPTH)
    {
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }

    referee_tx_queue[referee_tx_tail].len = tx_len;
    memcpy(referee_tx_queue[referee_tx_tail].data, send, tx_len);
    referee_tx_tail = (uint8_t)((referee_tx_tail + 1U) % REFEREE_TX_QUEUE_DEPTH);
    referee_tx_count++;
    __set_PRIMASK(primask);

    return HAL_OK;
}

static RefereeTxFrame_t* RefereeTxQueuePeek(void)
{
    if (referee_tx_count == 0U)
        return NULL;

    return &referee_tx_queue[referee_tx_head];
}

static void RefereeTxQueuePop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (referee_tx_count > 0U)
    {
        referee_tx_head = (uint8_t)((referee_tx_head + 1U) % REFEREE_TX_QUEUE_DEPTH);
        referee_tx_count--;
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 将一帧已通过CRC校验的裁判系统数据写入内部状态
 *
 * @note 该函数在USARTProcess任务上下文中调用。写入内部状态时用极短临界区保护,
 *       使其他任务通过RefereeGet()复制快照时不会读到半更新数据。
 */
static void RefereeStoreFrameData(const uint8_t* frame, uint16_t data_length)
{
    uint16_t cmd_id;
    uint32_t primask;

    if (frame == NULL)
        return;

    cmd_id = (uint16_t)(((uint16_t)frame[CMD_ID_Offset + 1U] << 8) | frame[CMD_ID_Offset]);

    primask = __get_PRIMASK();
    __disable_irq();

    memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);
    referee_info.CmdID = cmd_id;

    // 第8个字节开始才是数据区。每个分支都按协议长度拷贝,避免短帧覆盖结构体。
    switch (cmd_id)
    {
    case ID_game_state: // 0x0001
        if (data_length >= LEN_game_state)
            memcpy(&referee_info.GameState, (frame + DATA_Offset), LEN_game_state);
        break;
    case ID_game_result: // 0x0002
        if (data_length >= LEN_game_result)
            memcpy(&referee_info.GameResult, (frame + DATA_Offset), LEN_game_result);
        break;
    case ID_game_robot_survivors: // 0x0003
        if (data_length >= LEN_game_robot_HP)
            memcpy(&referee_info.GameRobotHP, (frame + DATA_Offset), LEN_game_robot_HP);
        break;
    case ID_event_data: // 0x0101
        if (data_length >= LEN_event_data)
            memcpy(&referee_info.EventData, (frame + DATA_Offset), LEN_event_data);
        break;
    case ID_supply_projectile_action: // 0x0102
        if (data_length >= LEN_supply_projectile_action)
            memcpy(&referee_info.SupplyProjectileAction, (frame + DATA_Offset), LEN_supply_projectile_action);
        break;
    case ID_game_robot_state: // 0x0201
        if (data_length >= LEN_game_robot_state)
            memcpy(&referee_info.GameRobotState, (frame + DATA_Offset), LEN_game_robot_state);
        break;
    case ID_power_heat_data: // 0x0202
        if (data_length >= LEN_power_heat_data)
            memcpy(&referee_info.PowerHeatData, (frame + DATA_Offset), LEN_power_heat_data);
        break;
    case ID_game_robot_pos: // 0x0203
        if (data_length >= LEN_game_robot_pos)
            memcpy(&referee_info.GameRobotPos, (frame + DATA_Offset), LEN_game_robot_pos);
        break;
    case ID_buff_musk: // 0x0204
        if (data_length >= LEN_buff_musk)
            memcpy(&referee_info.BuffMusk, (frame + DATA_Offset), LEN_buff_musk);
        break;
    case ID_aerial_robot_energy: // 0x0205
        if (data_length >= LEN_aerial_robot_energy)
            memcpy(&referee_info.AerialRobotEnergy, (frame + DATA_Offset), LEN_aerial_robot_energy);
        break;
    case ID_robot_hurt: // 0x0206
        if (data_length >= LEN_robot_hurt)
            memcpy(&referee_info.RobotHurt, (frame + DATA_Offset), LEN_robot_hurt);
        break;
    case ID_shoot_data: // 0x0207
        if (data_length >= LEN_shoot_data)
            memcpy(&referee_info.ShootData, (frame + DATA_Offset), LEN_shoot_data);
        break;
    case ID_student_interactive: // 0x0301
        if (data_length >= LEN_receive_data)
        {
            uint16_t data_cmd_id = (uint16_t)(frame[DATA_Offset] | ((uint16_t)frame[DATA_Offset + 1U] << 8));
            uint16_t receiver_id = (uint16_t)(frame[DATA_Offset + 4U] | ((uint16_t)frame[DATA_Offset + 5U] << 8));

            /*
			 * 0x0301同时承载UI绘图和机器人间自定义通信。
			 * 这里只保存自定义通信数据,避免把自己发出的UI绘图包误解析到ReceiveData。
			 */
            if (data_cmd_id == Communicate_Data_ID &&
                (referee_info.referee_id.Robot_ID == 0U || receiver_id == referee_info.referee_id.Robot_ID))
            {
                memcpy(&referee_info.ReceiveData, (frame + DATA_Offset), LEN_receive_data);
            }
        }
        break;
    default:
        break;
    }

    __set_PRIMASK(primask);
}

/**
 * @brief  读取裁判数据,在USARTProcess任务上下文中解析
 * @param  buff: 读取到的裁判系统原始数据
 * @param  buff_len: 本次实际接收到的数据长度
 * @retval 是否解析到有效帧
 * @attention  循环扫描buffer,坏帧只丢弃当前字节,尽量恢复后续合法帧
 */
static uint8_t JudgeReadData(uint8_t* buff, uint16_t buff_len)
{
    uint16_t data_length;
    uint32_t frame_length;
    uint16_t offset = 0;
    uint8_t parsed = 0;

    if (buff == NULL) // 空数据包,不作任何处理
        return 0;

    if (buff_len < LEN_HEADER)
        return 0;

    while ((uint32_t)offset + LEN_HEADER <= buff_len)
    {
        uint8_t* frame = buff + offset;

        if (frame[SOF] != REFEREE_SOF)
        {
            offset++;
            continue;
        }

        if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) != TRUE)
        {
            offset++;
            continue;
        }

        data_length = (uint16_t)(frame[DATA_LENGTH] | ((uint16_t)frame[DATA_LENGTH + 1U] << 8));
        frame_length = (uint32_t)data_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
        if (frame_length > REFEREE_FRAME_MAX_SIZE)
        {
            offset++;
            continue;
        }

        if (frame_length > (uint32_t)buff_len - offset)
        {
            offset++;
            continue;
        }

        if (Verify_CRC16_Check_Sum(frame, frame_length) != TRUE)
        {
            offset++;
            continue;
        }

        RefereeStoreFrameData(frame, data_length);
        parsed = 1;
        offset = (uint16_t)(offset + frame_length);
    }

    return parsed;
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
    if (referee_usart_instance == NULL)
        return;

    if (JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->recv_len))
    {
        if (referee_daemon != NULL)
            DaemonReload(referee_daemon);
        referee_lost_reported = 0;
    }
}

// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void* arg)
{
    UNUSED(arg);

    if (referee_lost_reported == 0U)
    {
        USARTServiceInit(referee_usart_instance);
        LOGWARNING("[rm_ref] lost referee data");
        referee_lost_reported = 1U;
    }
}

/* 裁判系统通信初始化 */
referee_info_t* RefereeInit(UART_HandleTypeDef* referee_usart_handle)
{
    USART_Init_Config_s conf = {0};
    memset(&referee_info, 0, sizeof(referee_info));
    referee_lost_reported = 0U;
    RefereeTxQueueReset();

    conf.module_callback = RefereeRxCallback;
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
    referee_usart_instance = USARTRegister(&conf);
    if (referee_usart_instance == NULL)
    {
        LOGERROR("[rm_ref] USART register failed");
        return NULL;
    }

    Daemon_Init_Config_s daemon_conf = {
        .callback = RefereeLostCallback,
        .owner_id = referee_usart_instance,
        .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    };
    referee_daemon = DaemonRegister(&daemon_conf);

    return &referee_info;
}

uint8_t RefereeGet(referee_info_t* snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
        return 0U;

    // 与RefereeStoreFrameData()使用同一类短临界区,保证快照复制期间不发生任务切换。
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(snapshot, &referee_info, sizeof(referee_info_t));
    __set_PRIMASK(primask);

    return 1U;
}

/**
 * @brief 裁判系统数据发送函数
 *
 * @note 该接口只负责把一帧交互数据放入内部发送队列,真正的DMA发送由RefereeTxProcess()
 *       在UI任务上下文中按裁判系统频率限制逐帧启动。
 */
HAL_StatusTypeDef RefereeSend(uint8_t* send, uint16_t tx_len)
{
    HAL_StatusTypeDef status;

    if (referee_usart_instance == NULL)
    {
        LOGWARNING("[rm_ref] send before referee init");
        return HAL_ERROR;
    }

    status = RefereeTxQueuePush(send, tx_len);
    if (status == HAL_BUSY)
    {
        LOGWARNING("[rm_ref] tx queue full, drop frame len [%d]", tx_len);
    }
    else if (status != HAL_OK)
    {
        LOGWARNING("[rm_ref] tx enqueue failed, len [%d]", tx_len);
    }

    return status;
}

void RefereeTxProcess(void)
{
    RefereeTxFrame_t* frame;
    HAL_StatusTypeDef status;
    uint32_t now;

    if (referee_usart_instance == NULL)
        return;

    // RTOS启动前不主动发送队列,避免异步DMA发送和初始化阶段的中断关闭状态冲突。
    if (osKernelGetState() != osKernelRunning)
        return;

    if (USARTIsReady(referee_usart_instance) == 0U)
        return;

    now = osKernelGetTickCount();
    if (referee_tx_started != 0U && (now - referee_last_tx_tick) < REFEREE_SEND_INTERVAL_MS)
        return;

    frame = RefereeTxQueuePeek();
    if (frame == NULL)
        return;

    status = USARTSend(referee_usart_instance, frame->data, frame->len, USART_TRANSFER_DMA);
    if (status == HAL_OK)
    {
        RefereeTxQueuePop();
        referee_last_tx_tick = now;
        referee_tx_started = 1U;
    }
    else if (status != HAL_BUSY)
    {
        /*
		 * 非BUSY错误通常说明当前帧或串口状态异常。丢弃当前帧可避免坏帧长期卡住队首,
		 * 后续UI刷新还有机会继续发送。
		 */
        LOGWARNING("[rm_ref] tx failed, status [%d], drop frame", status);
        RefereeTxQueuePop();
        referee_last_tx_tick = now;
        referee_tx_started = 1U;
    }
}
