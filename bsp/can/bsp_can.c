#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os2.h"

#define CAN_RX_EVENT_CNT 64U
#define CAN_RX_THREAD_FLAG 0x00000001U
#define FDCAN_RX_ACTIVE_ITS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | \
							 FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | \
							 FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL | \
							 FDCAN_IT_RX_FIFO1_WATERMARK | FDCAN_IT_RX_FIFO1_MESSAGE_LOST)

static const uint32_t DLC_LookUp_Table[9] = {
	FDCAN_DLC_BYTES_0,
	FDCAN_DLC_BYTES_1,
	FDCAN_DLC_BYTES_2,
	FDCAN_DLC_BYTES_3,
	FDCAN_DLC_BYTES_4,
	FDCAN_DLC_BYTES_5,
	FDCAN_DLC_BYTES_6,
	FDCAN_DLC_BYTES_7,
	FDCAN_DLC_BYTES_8,
};

typedef struct
{
	CANInstance *instance;
	uint8_t data[8];
	uint8_t len;
} CANRxEvent;

typedef struct
{
	FDCAN_HandleTypeDef *handle;                      // 当前资源对应的FDCAN硬件句柄
	CANInstance *instances[CAN_MX_REGISTER_CNT];      // 挂在该FDCAN总线下的CAN实例
	uint8_t instance_count;                           // 当前总线已经注册的实例数量
	uint8_t filter_idx;                               // 当前总线已经使用到的标准过滤器索引
	uint8_t started;                                  // 当前FDCAN是否已经启动
	osMutexId_t tx_mutex;                            // 当前FDCAN发送互斥锁,避免多个任务同时操作同一路发送FIFO
	volatile uint32_t fifo0_lost_count;               // FIFO0硬件丢帧计数
	volatile uint32_t fifo1_lost_count;               // FIFO1硬件丢帧计数
} CANBusResource;

static CANBusResource can_bus[DEVICE_CAN_CNT] = {
	{.handle = &hfdcan1},
	{.handle = &hfdcan2},
	{.handle = &hfdcan3},
};
static uint8_t can_service_started;
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增
static CANInstance can_instance_pool[CAN_MX_REGISTER_CNT]; // CAN实例静态池,放在默认.bss/DTCM中
static CANRxEvent can_rx_events[CAN_RX_EVENT_CNT];
static volatile uint8_t can_rx_write_idx;
static volatile uint8_t can_rx_read_idx;
static volatile uint8_t can_rx_pending_cnt;
static volatile uint8_t can_rx_task_pending;
static volatile uint32_t can_rx_dropped_cnt;
static volatile uint32_t can_rx_hal_error_cnt;
static volatile uint32_t can_rx_invalid_dlc_cnt;
static osThreadId_t can_process_task_handle;

static void CANStartFDCANService(CANBusResource *bus);

__attribute__((noreturn)) static void CANFatalError(void)
{
	Error_Handler();
	__builtin_unreachable();
}

static CANBusResource *CANFindBusByHandle(FDCAN_HandleTypeDef *hfdcan)
{
	for (uint8_t i = 0; i < DEVICE_CAN_CNT; i++)
	{
		if (can_bus[i].handle == hfdcan)
		{
			return &can_bus[i];
		}
	}

	return NULL;
}

static uint8_t CANGetBusIndex(CANBusResource const *bus)
{
	if (bus == NULL)
	{
		return 0U;
	}

	for (uint8_t i = 0; i < DEVICE_CAN_CNT; i++)
	{
		if (&can_bus[i] == bus)
		{
			return (uint8_t)(i + 1U);
		}
	}

	return 0U;
}

static void CANTryNotifyProcessTask(void)
{
	if (can_process_task_handle != NULL)
	{
		(void)osThreadFlagsSet(can_process_task_handle, CAN_RX_THREAD_FLAG);
	}
}

static void CANProcessRxEvents(void)
{
	CANRxEvent event;
	uint32_t primask;

	for (;;)
	{
		memset(&event, 0, sizeof(event));

		primask = __get_PRIMASK();
		__disable_irq();
		if (can_rx_pending_cnt == 0U)
		{
			can_rx_task_pending = 0U;
			__set_PRIMASK(primask);
			break;
		}

		event = can_rx_events[can_rx_read_idx];
		can_rx_events[can_rx_read_idx].instance = NULL;
		can_rx_read_idx = (uint8_t)((can_rx_read_idx + 1U) % CAN_RX_EVENT_CNT);
		can_rx_pending_cnt--;
		__set_PRIMASK(primask);

		if (event.instance != NULL && event.instance->can_module_callback != NULL)
		{
			event.instance->rx_len = event.len;
			memcpy(event.instance->rx_buff, event.data, event.len);
			// CAN模块回调统一在CANProcessTask中执行,避免占用FDCAN接收中断和BSPServiceTask。
			event.instance->can_module_callback(event.instance);
		}
	}
}

__attribute__((noreturn)) void CANProcessTask(void *argument)
{
	(void)argument;

	can_process_task_handle = osThreadGetId();
	LOGINFO("[bsp_can] CAN Process Task Start");

	for (;;)
	{
		CANProcessRxEvents();
		(void)osThreadFlagsWait(CAN_RX_THREAD_FLAG, osFlagsWaitAny, osWaitForever);
	}
}

static void CANQueueRxEvent(CANInstance *instance, const uint8_t *data, uint8_t len)
{
	uint8_t should_post = 0U;
	uint32_t primask;

	if (instance == NULL || data == NULL || len > 8U || instance->can_module_callback == NULL)
	{
		return;
	}

	/*
	 * CAN接收中断可能连续取出多帧报文。这里先把帧数据复制到BSP内部队列,
	 * 之后由高优先级CANProcessTask再复制到对应instance->rx_buff并调用模块回调,
	 * 避免只投递instance指针导致后续帧覆盖上一帧数据。
	 */
	primask = __get_PRIMASK();
	__disable_irq();

	if (can_rx_pending_cnt < CAN_RX_EVENT_CNT)
	{
		can_rx_events[can_rx_write_idx].instance = instance;
		can_rx_events[can_rx_write_idx].len = len;
		memcpy(can_rx_events[can_rx_write_idx].data, data, len);
		can_rx_write_idx = (uint8_t)((can_rx_write_idx + 1U) % CAN_RX_EVENT_CNT);
		can_rx_pending_cnt++;

		if (can_rx_task_pending == 0U)
		{
			can_rx_task_pending = 1U;
			should_post = 1U;
		}
	}
	else
	{
		can_rx_dropped_cnt++;
	}

	__set_PRIMASK(primask);

	if (should_post != 0U)
	{
		CANTryNotifyProcessTask();
	}
}

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给FDCAN添加过滤器后,硬件会根据接收ID过滤报文,符合规则的ID会进入指定FIFO并触发中断。
 *
 * @note H723系列FDCAN过滤器数量完全在CubeMX中自定义,因此按当前FDCAN实例检查StdFiltersNbr,
 *       再添加过滤器。当前按接收ID奇偶分配FIFO,奇数rx_id进入FIFO0,偶数rx_id进入FIFO1。
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *_instance)
{
	CANBusResource *bus = CANFindBusByHandle(_instance->can_handle);
	uint8_t can_idx = CANGetBusIndex(bus);

	if (bus == NULL)
	{
		LOGERROR("[bsp_can] invalid FDCAN handle, tx_id:0x%x, rx_id:0x%x",
		         (unsigned int)_instance->tx_id, (unsigned int)_instance->rx_id);
		CANFatalError();
	}

	// 只检查当前FDCAN实例的过滤器数量；StdFiltersNbr为数量，合法索引范围是0到StdFiltersNbr - 1。
	if (bus->filter_idx >= _instance->can_handle->Init.StdFiltersNbr)
	{
		LOGERROR("[bsp_can] FDCAN filter exceeded, used:%u, limit:%u, tx_id:0x%x, rx_id:0x%x",
		         (unsigned int)bus->filter_idx, (unsigned int)_instance->can_handle->Init.StdFiltersNbr,
		         (unsigned int)_instance->tx_id, (unsigned int)_instance->rx_id);
		CANFatalError();
	}

	if (bus->started != 0U)
	{
		/*
		 * FDCAN过滤器应在外设停止状态下配置。若运行期确实注册新实例,
		 * 先短暂停止当前总线,配置完成后再启动回来,避免HAL状态不允许配置过滤器。
		 */
		if (HAL_FDCAN_Stop(bus->handle) != HAL_OK)
		{
			LOGERROR("[bsp_can] FDCAN%u stop before config filter failed", (unsigned int)can_idx);
			CANFatalError();
		}
		bus->started = 0U;
	}

	FDCAN_FilterTypeDef fdcan_filter_conf;
	fdcan_filter_conf.FilterIndex=bus->filter_idx++;
	//使用单个ID模式
	fdcan_filter_conf.FilterType=FDCAN_FILTER_DUAL;
	fdcan_filter_conf.FilterConfig=(_instance->rx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;//按接收ID分配FIFO,奇数rx_id进入FIFO0,偶数rx_id进入FIFO1
	fdcan_filter_conf.FilterID1=_instance->rx_id;
	fdcan_filter_conf.FilterID2=_instance->rx_id;
	fdcan_filter_conf.IdType=FDCAN_STANDARD_ID;
	fdcan_filter_conf.IsCalibrationMsg=0;
	//fdcan_filter_conf.RxBufferIndex=0;

	if (HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u config filter failed, filter:%u, tx_id:0x%x, rx_id:0x%x",
		         (unsigned int)can_idx, (unsigned int)fdcan_filter_conf.FilterIndex,
		         (unsigned int)_instance->tx_id, (unsigned int)_instance->rx_id);
		CANFatalError();
	}
}

/**
 * @brief 启动单路FDCAN服务,并检查每一步HAL配置结果
 *
 * @param bus FDCAN总线资源
 */
static void CANStartFDCANService(CANBusResource *bus)
{
	uint8_t can_idx = CANGetBusIndex(bus);

	if (bus == NULL || bus->handle == NULL)
	{
		LOGERROR("[bsp_can] invalid FDCAN bus resource");
		CANFatalError();
	}

	if (bus->started != 0U)
	{
		return;
	}

	if (HAL_FDCAN_ConfigRxFifoOverwrite(bus->handle, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u config RX FIFO0 overwrite failed", (unsigned int)can_idx);
		CANFatalError();
	}

	if (HAL_FDCAN_ConfigRxFifoOverwrite(bus->handle, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u config RX FIFO1 overwrite failed", (unsigned int)can_idx);
		CANFatalError();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(bus->handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u config global filter failed", (unsigned int)can_idx);
		CANFatalError();
	}

	if (HAL_FDCAN_Start(bus->handle) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u start failed", (unsigned int)can_idx);
		CANFatalError();
	}

	if (HAL_FDCAN_ActivateNotification(bus->handle, FDCAN_RX_ACTIVE_ITS, 0) != HAL_OK)
	{
		LOGERROR("[bsp_can] FDCAN%u activate notification failed", (unsigned int)can_idx);
		CANFatalError();
	}

	bus->started = 1U;
}

/**
 * @brief 将CAN发送超时时间从ms换算成RTOS tick
 *
 * @note 原CANTransmit()的timeout单位是ms,这里保留旧接口语义。
 *       小于1个tick的超时时间会使用0 tick,表示不等待发送互斥锁。
 */
static uint32_t CANTimeoutMsToTicks(float timeout_ms)
{
	uint32_t tick_freq;
	uint32_t ticks;

	if (timeout_ms <= 0.0f)
	{
		return 0U;
	}

	tick_freq = osKernelGetTickFreq();
	ticks = (uint32_t)((timeout_ms * (float)tick_freq) / 1000.0f);
	return ticks;
}

/**
 * @brief 确保当前FDCAN总线发送互斥锁已经创建
 *
 * @note 互斥锁按FDCAN硬件总线共享,不是按CANInstance单独创建。
 *       这样同一路FDCAN上多个模块同时发送时,会按任务调度顺序串行进入HAL发送接口。
 */
static HAL_StatusTypeDef CANEnsureTxMutex(CANBusResource *bus)
{
	const osMutexAttr_t mutex_attr = {
		.name = "bsp_can_tx",
		.attr_bits = osMutexPrioInherit,
	};
	int32_t kernel_lock;

	if (bus == NULL)
	{
		return HAL_ERROR;
	}

	if (osKernelGetState() != osKernelRunning)
	{
		return HAL_OK;
	}

	if (bus->tx_mutex != NULL)
	{
		return HAL_OK;
	}

	kernel_lock = osKernelLock();
	if (kernel_lock < 0)
	{
		LOGERROR("[bsp_can] CAN tx mutex kernel lock failed");
		return HAL_ERROR;
	}

	if (bus->tx_mutex == NULL)
	{
		bus->tx_mutex = osMutexNew(&mutex_attr);
	}

	if (osKernelRestoreLock(kernel_lock) < 0)
	{
		LOGERROR("[bsp_can] CAN tx mutex kernel restore failed");
		return HAL_ERROR;
	}

	if (bus->tx_mutex == NULL)
	{
		LOGERROR("[bsp_can] CAN tx mutex create failed");
		return HAL_ERROR;
	}

	return HAL_OK;
}

/**
 * @brief 获取当前FDCAN总线发送权
 *
 * @return int8_t 1表示拿到锁,0表示RTOS未运行无需锁,-1表示失败
 */
static int8_t CANLockTx(CANInstance *instance, float timeout_ms)
{
	CANBusResource *bus;

	if (instance == NULL || instance->can_handle == NULL)
	{
		return -1;
	}

	if (__get_IPSR() != 0U)
	{
		return -1;
	}

	if (osKernelGetState() != osKernelRunning)
	{
		return 0;
	}

	bus = CANFindBusByHandle(instance->can_handle);
	if (bus == NULL)
	{
		LOGERROR("[bsp_can] CAN tx lock failed, invalid FDCAN handle");
		return -1;
	}

	if (CANEnsureTxMutex(bus) != HAL_OK)
	{
		return -1;
	}

	if (osMutexAcquire(bus->tx_mutex, CANTimeoutMsToTicks(timeout_ms)) != osOK)
	{
		return -1;
	}

	return 1;
}

/**
 * @brief 释放当前FDCAN总线发送权
 */
static void CANUnlockTx(CANInstance *instance, int8_t lock_state)
{
	CANBusResource *bus;

	if (lock_state <= 0 || instance == NULL || osKernelGetState() != osKernelRunning)
	{
		return;
	}

	bus = CANFindBusByHandle(instance->can_handle);
	if (bus != NULL && bus->tx_mutex != NULL)
	{
		(void)osMutexRelease(bus->tx_mutex);
	}
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动三路FDCAN并开启接收中断。
 *       当前采用FIFO接收方式,全局过滤器配置为全部拒绝,只接受已注册过滤器的标准帧ID。
 *       
 */
void CANServiceInit()
{
	//HAL_FDCAN_ConfigClockCalibration()
	for (uint8_t i = 0; i < DEVICE_CAN_CNT; i++)
	{
		CANStartFDCANService(&can_bus[i]);
	}
	can_service_started = 1U;
}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL)
    {
        LOGERROR("[bsp_can] CAN register with invalid config");
        CANFatalError();
        return NULL;
    }

    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
        CANFatalError();
        return NULL;
    }
    CANBusResource *bus = CANFindBusByHandle(config->can_handle);
    if (bus == NULL)
    {
        LOGERROR("[bsp_can] CAN register with invalid FDCAN handle");
        CANFatalError();
        return NULL;
    }
    if (bus->instance_count >= CAN_MX_REGISTER_CNT)
    {
        LOGERROR("[bsp_can] FDCAN%u instance count exceeds limit",
                 (unsigned int)CANGetBusIndex(bus));
        CANFatalError();
        return NULL;
    }
    for (size_t i = 0; i < bus->instance_count; i++)
    { // 同一路FDCAN上不允许重复注册相同接收ID
        if (bus->instances[i]->rx_id == config->rx_id)
        {
            LOGERROR("[bsp_can] CAN id crash, tx [0x%x] or rx [0x%x] already registered", config->tx_id, config->rx_id);
            CANFatalError();
            return NULL;
        }
    }

    CANInstance *instance = &can_instance_pool[idx];
    memset(instance, 0, sizeof(CANInstance));
    // 进行发送报文的配置
    instance->txconf.Identifier = config->tx_id; 				// 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;  				// 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;    		// 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;    		// 数据长度为8字节
	instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;	// 兼容CAN2.0,错误状态指示器设为主动
	instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;         	// 兼容CAN2.0禁用位速率切换
	instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;          	// 使用经典CAN格式
	instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 不需要，禁用事件FIFO
	instance->txconf.MessageMarker = 0;                     	// 不使用消息标记

    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    bus->instances[bus->instance_count++] = instance; // 将实例保存到对应FDCAN总线资源中
    idx++;
    if (can_service_started != 0U && bus->started == 0U)
    {
        CANStartFDCANService(bus); // 运行期补注册实例时,先把实例放入表中再恢复接收中断
    }
    if (can_service_started == 0U)
    {
        CANServiceInit(); // 第一次注册完成后启动硬件,确保首个过滤器先于FDCAN Start配置
        LOGINFO("[bsp_can] CAN Service Init");
    }

    return instance; // 返回can实例指针
}

/*
 * 发送接口沿用“模块先写instance->tx_buff,再调用CANTransmit()”的用法。
 * 这种方式让发送缓存生命周期由CANInstance管理,上层不需要额外保证临时buffer有效。
 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
    int8_t tx_lock_state;

    if (_instance == NULL || _instance->can_handle == NULL)
    {
        LOGERROR("[bsp_can] CAN transmit with invalid instance");
        return 0;
    }

    if (__get_IPSR() != 0U)
    {
        return 0;
    }

    tx_lock_state = CANLockTx(_instance, timeout);
    if (tx_lock_state < 0)
    {
        LOGWARNING("[bsp_can] CAN tx lock failed. Cnt [%d]", busy_count);
        busy_count++;
        return 0;
    }
    if (timeout > 0.0f && (DWT_GetTimeline_ms() - dwt_start > timeout))
    {
        LOGWARNING("[bsp_can] CAN tx lock timeout. Cnt [%d]", busy_count);
        busy_count++;
        CANUnlockTx(_instance, tx_lock_state);
        return 0;
    }

    while(HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle)==0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            CANUnlockTx(_instance, tx_lock_state);
            return 0;
        }

        /*
         * 若调度器已经运行且调用者给了毫秒级超时,等待Tx FIFO空闲时主动让出CPU。
         * 小于1ms的超时仍使用DWT短忙等,避免osDelay(1)把亚毫秒发送等待放大成1个tick。
         */
        if (osKernelGetState() == osKernelRunning && timeout >= 1.0f)
        {
            osDelay(1U);
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;

    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff))
    {
        LOGWARNING("[bsp_can] CAN bus BUSY! cnt:%d", busy_count);
        busy_count++;
        CANUnlockTx(_instance, tx_lock_state);
        return 0;
    }
    CANUnlockTx(_instance, tx_lock_state);
    return 1; // 发送成功
}

uint32_t CANGetDroppedRxEventCount(void)
{
	return can_rx_dropped_cnt;
}

uint32_t CANGetFifoLostCount(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
	CANBusResource *bus = CANFindBusByHandle(hfdcan);

	if (bus == NULL)
	{
		LOGERROR("[bsp_can] CANGetFifoLostCount with invalid FDCAN handle");
		return 0U;
	}

	if (fifo == FDCAN_RX_FIFO0)
	{
		return bus->fifo0_lost_count;
	}
	if (fifo == FDCAN_RX_FIFO1)
	{
		return bus->fifo1_lost_count;
	}

	LOGERROR("[bsp_can] CANGetFifoLostCount with invalid fifo:%u", (unsigned int)fifo);
	return 0U;
}

uint32_t CANGetHardwareLostCount(void)
{
	uint32_t lost_count = 0U;

	for (uint8_t i = 0; i < DEVICE_CAN_CNT; i++)
	{
		lost_count += can_bus[i].fifo0_lost_count;
		lost_count += can_bus[i].fifo1_lost_count;
	}

	return lost_count;
}

uint32_t CANGetRxHalErrorCount(void)
{
	return can_rx_hal_error_cnt;
}

uint32_t CANGetRxInvalidDlcCount(void)
{
	return can_rx_invalid_dlc_cnt;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (_instance == NULL)
    {
        LOGERROR("[bsp_can] CANSetDLC with null instance");
        return;
    }

    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
    {
        LOGERROR("[bsp_can] CAN DLC error, length:%u", (unsigned int)length);
        return;
    }

    _instance->txconf.DataLength = DLC_LookUp_Table[length];
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 将HAL FDCAN的DLC宏转换为实际字节数
 *
 * @param dlc HAL接收头中的DataLength字段
 * @return uint8_t 实际数据长度,当前Classic CAN仅支持0到8字节
 */
static uint8_t FDCANDecodeDLC(uint32_t dlc)
{
	switch (dlc)
	{
	case FDCAN_DLC_BYTES_0:
		return 0;
	case FDCAN_DLC_BYTES_1:
		return 1;
	case FDCAN_DLC_BYTES_2:
		return 2;
	case FDCAN_DLC_BYTES_3:
		return 3;
	case FDCAN_DLC_BYTES_4:
		return 4;
	case FDCAN_DLC_BYTES_5:
		return 5;
	case FDCAN_DLC_BYTES_6:
		return 6;
	case FDCAN_DLC_BYTES_7:
		return 7;
	case FDCAN_DLC_BYTES_8:
		return 8;
	default:
		can_rx_invalid_dlc_cnt++;
		return 0;
	}
}

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _fdhcan
 * @param fifox FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
    static uint8_t fdcan_rx_buff[8];
	CANBusResource *bus = CANFindBusByHandle(_hfdcan);
    uint8_t data_length;

	if (bus == NULL)
	{
		return;
	}

    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff) != HAL_OK) // 从FIFO中获取数据
        {
            can_rx_hal_error_cnt++;
            break;
        }
        data_length = FDCANDecodeDLC(rxconf.DataLength); // 将HAL DLC宏转换为实际接收字节数
        if(rxconf.RxFrameType==FDCAN_DATA_FRAME && rxconf.IdType==FDCAN_STANDARD_ID)
        {
            for (size_t i = 0; i < bus->instance_count; ++i)
            {
                // 两者相等说明这是要找的实例
                if (rxconf.Identifier == bus->instances[i]->rx_id)
                {
                    CANQueueRxEvent(bus->instances[i], fdcan_rx_buff, data_length); // 中断只缓存帧,模块回调由CANProcessTask执行
                    break; // 当前帧已找到归属实例,只退出实例查找循环,继续处理FIFO中的后续帧
                }
            }
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	CANBusResource *bus = CANFindBusByHandle(hfdcan);

	/* 检查Rx FIFO 0中是否有消息丢失 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		if (bus != NULL)
		{
			bus->fifo0_lost_count++;
		}
	}
	/* 检查是否有新消息写入Rx FIFO 0或到达一定阈值 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
	}
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	CANBusResource *bus = CANFindBusByHandle(hfdcan);

	/* 检查Rx FIFO 1中是否有消息丢失 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		if (bus != NULL)
		{
			bus->fifo1_lost_count++;
		}
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}
