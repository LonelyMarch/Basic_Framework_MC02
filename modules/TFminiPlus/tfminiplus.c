#include "tfminiplus.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include <string.h>

#define TFMINIPLUS_CMD_GET_DATA_LEN 5U
#define TFMINIPLUS_FRAME_HEADER 0x59U
#define TFMINIPLUS_STANDARD_CHECKSUM_INDEX 8U
#define TFMINIPLUS_EXTENDED_CHECKSUM_INDEX (TFMINIPLUS_FRAME_LEN - 1U)

/* TFmini Plus获取一次测距数据命令: 最后一字节为前4字节低8位累加和。 */
static uint8_t tfmini_plus_get_data_cmd[TFMINIPLUS_CMD_GET_DATA_LEN] = {0x5AU, 0x05U, 0x00U, 0x06U, 0x65U};

static TFMiniPlusInstance tfmini_plus_pool[TFMINIPLUS_INSTANCE_CNT];
static uint8_t tfmini_plus_idx = 0U;

/**
 * @brief 根据当前RTOS状态选择合适的延时方式。
 *
 * 注册可能发生在RobotInit阶段,也可能发生在任务中。调度器启动前只能使用DWT忙等,
 * 调度器启动后使用osDelay让出CPU,避免低优先级任务被无意义占用。
 */
static void TFMiniPlusDelayMs(uint32_t delay_ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        (void)osDelay(delay_ms);
    }
    else
    {
        DWT_Delay((float)delay_ms / 1000.0f);
    }
}

/**
 * @brief 计算TFmini Plus数据帧校验和。
 *
 * TFmini Plus常用数据帧的最后一字节为前面所有字节的低8位累加和。
 */
static uint8_t TFMiniPlusCalcChecksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0U;

    if (data == NULL || len == 0U)
    {
        return 0U;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        sum = (uint16_t)(sum + data[i]);
    }

    return (uint8_t)sum;
}

/**
 * @brief 校验测距数据帧格式。
 */
static HAL_StatusTypeDef TFMiniPlusCheckFrame(const uint8_t *frame)
{
    uint8_t standard_checksum;
    uint8_t extended_checksum;

    if (frame == NULL)
    {
        return HAL_ERROR;
    }

    if (frame[0] != TFMINIPLUS_FRAME_HEADER || frame[1] != TFMINIPLUS_FRAME_HEADER)
    {
        LOGWARNING("[tfminiplus] frame header invalid: 0x%02X 0x%02X", frame[0], frame[1]);
        return HAL_ERROR;
    }

    /*
     * TFmini Plus常见主动输出帧为9字节,部分I2C读取实现会多读扩展字段。
     * 因此这里同时接受9字节标准校验和11字节扩展校验,避免扩展字段未使用时误判。
     */
    standard_checksum = TFMiniPlusCalcChecksum(frame, TFMINIPLUS_STANDARD_CHECKSUM_INDEX);
    extended_checksum = TFMiniPlusCalcChecksum(frame, TFMINIPLUS_EXTENDED_CHECKSUM_INDEX);
    if (standard_checksum != frame[TFMINIPLUS_STANDARD_CHECKSUM_INDEX] &&
        extended_checksum != frame[TFMINIPLUS_EXTENDED_CHECKSUM_INDEX])
    {
        LOGWARNING("[tfminiplus] frame checksum invalid: std=0x%02X/0x%02X ext=0x%02X/0x%02X",
                   standard_checksum,
                   frame[TFMINIPLUS_STANDARD_CHECKSUM_INDEX],
                   extended_checksum,
                   frame[TFMINIPLUS_EXTENDED_CHECKSUM_INDEX]);
        return HAL_ERROR;
    }

    return HAL_OK;
}

TFMiniPlusInstance *TFMiniPlusRegister(const TFMiniPlus_Init_Config_s *config)
{
    TFMiniPlusInstance *instance;
    IIC_Init_Config_s iic_config;

    if (config == NULL || config->hi2c == NULL)
    {
        LOGERROR("[tfminiplus] register failed: config or hi2c is null");
        return NULL;
    }

    if (config->work_mode != IIC_BLOCK_MODE &&
        config->work_mode != IIC_IT_MODE &&
        config->work_mode != IIC_DMA_MODE)
    {
        LOGERROR("[tfminiplus] register failed: invalid iic work mode");
        return NULL;
    }

    if (tfmini_plus_idx >= TFMINIPLUS_INSTANCE_CNT)
    {
        LOGERROR("[tfminiplus] register failed: instance pool full");
        return NULL;
    }

    instance = &tfmini_plus_pool[tfmini_plus_idx];
    memset(instance, 0, sizeof(TFMiniPlusInstance));

    iic_config.handle = config->hi2c;
    iic_config.dev_address = TFMINIPLUS_I2C_ADDR;
    iic_config.work_mode = config->work_mode;
    iic_config.callback = NULL;
    iic_config.id = instance;

    instance->iic = IICRegister(&iic_config);
    if (instance->iic == NULL)
    {
        LOGERROR("[tfminiplus] register failed: IICRegister returned null");
        return NULL;
    }

    tfmini_plus_idx++;
    TFMiniPlusDelayMs(TFMINIPLUS_POWER_ON_DELAY_MS);
    return instance;
}

HAL_StatusTypeDef TFMiniPlusRead(TFMiniPlusInstance *instance)
{
    HAL_StatusTypeDef status;

    if (instance == NULL || instance->iic == NULL)
    {
        LOGERROR("[tfminiplus] read failed: instance or iic is null");
        return HAL_ERROR;
    }

    status = IICTransmit(instance->iic, tfmini_plus_get_data_cmd, TFMINIPLUS_CMD_GET_DATA_LEN, IIC_SEQ_RELEASE);
    if (status != HAL_OK)
    {
        instance->online = 0U;
        instance->error_count++;
        LOGWARNING("[tfminiplus] command transmit failed: status=%d", (int)status);
        return status;
    }

    status = IICReceive(instance->iic, instance->raw_frame, TFMINIPLUS_FRAME_LEN, IIC_SEQ_RELEASE);
    if (status != HAL_OK)
    {
        instance->online = 0U;
        instance->error_count++;
        LOGWARNING("[tfminiplus] data receive failed: status=%d", (int)status);
        return status;
    }

    status = TFMiniPlusCheckFrame(instance->raw_frame);
    if (status != HAL_OK)
    {
        instance->online = 0U;
        instance->error_count++;
        return status;
    }

    /*
     * 数据帧保持TFmini Plus常用小端格式:
     * raw_frame[2:3]为距离,raw_frame[4:5]为信号强度,raw_frame[6]保留为模式/状态字段。
     */
    instance->distance = (uint16_t)instance->raw_frame[2] | ((uint16_t)instance->raw_frame[3] << 8);
    instance->strength = (uint16_t)instance->raw_frame[4] | ((uint16_t)instance->raw_frame[5] << 8);
    instance->mode = instance->raw_frame[6];
    instance->online = 1U;
    return HAL_OK;
}

uint16_t TFMiniPlusGetDistance(const TFMiniPlusInstance *instance)
{
    if (instance == NULL)
    {
        return 0U;
    }

    return instance->distance;
}

uint16_t TFMiniPlusGetStrength(const TFMiniPlusInstance *instance)
{
    if (instance == NULL)
    {
        return 0U;
    }

    return instance->strength;
}

uint8_t TFMiniPlusIsOnline(const TFMiniPlusInstance *instance)
{
    if (instance == NULL)
    {
        return 0U;
    }

    return instance->online;
}
