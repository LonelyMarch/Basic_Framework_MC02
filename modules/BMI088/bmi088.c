#include "bmi088.h"
#include "bmi088_regNdef.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>

#define BMI088_INIT_RETRY_MAX 3U
#define BMI088_CALIBRATE_SAMPLE_COUNT 6000U
#define BMI088_CALIBRATE_TIMEOUT_S 12.0f
// 标准重力加速度,用于把当前板子的加速度模长修正到m/s^2单位。
#define BMI088_STANDARD_GRAVITY 9.805f
// 在线标定时允许的加速度模长波动,超过该值认为标定期间发生晃动。
#define BMI088_CALIBRATE_G_NORM_DIFF_LIMIT 0.5f
// 在线标定时允许的三轴角速度波动,超过该值认为标定期间发生转动。
#define BMI088_CALIBRATE_GYRO_DIFF_LIMIT 0.15f
// 在线标定最终允许的重力模长误差,用于剔除明显不合理的加速度候选结果。
#define BMI088_CALIBRATE_G_NORM_ERROR_LIMIT 0.5f
// 在线标定最终允许的陀螺仪零偏绝对值,用于剔除明显不合理的零偏候选结果。
#define BMI088_CALIBRATE_GYRO_OFFSET_LIMIT 0.01f
// 加速度模长过小时无法计算比例系数,该阈值用于避免除零或异常放大。
#define BMI088_CALIBRATE_MIN_G_NORM 1e-6f
// 标定过程中两次采样之间的等待时间,给BMI088留出数据更新间隔。
#define BMI088_CALIBRATE_SAMPLE_DELAY_S 0.0005f
// 一轮标定失败后再次尝试前的等待时间,避免异常状态下高频空转。
#define BMI088_CALIBRATE_RETRY_DELAY_S 0.01f
// BMI088温度寄存器约1.28s更新一次,高频重复读取没有意义,按1kHz姿态任务折算约1Hz读取一次。
#define BMI088_TEMP_UPDATE_DIVIDER 1000U
#define BMI088_TRIGGER_SYNC_RETRY_MAX 2U
#define BMI088_COM_WAIT_S ((float)BMI088_COM_WAIT_SENSOR_TIME / 1000000.0f)
#define BMI088_LONG_WAIT_S ((float)BMI088_LONG_DELAY_TIME / 1000.0f)
#define BMI088_ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))

/*
 * 当前工程没有统一定义INFANTRY_ID。若后续在robot_def.h中增加该宏,
 * 这里会自动沿用旧驱动的离线标定参数表;否则默认使用0号参数。
 */
#ifndef INFANTRY_ID
#define BMI088_OFFLINE_PARAM_ID 0
#else
#define BMI088_OFFLINE_PARAM_ID INFANTRY_ID
#endif

#if BMI088_OFFLINE_PARAM_ID == 0
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.00247530174f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.000393082853f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.000393082853f
#define BMI088_PRE_CALI_G_NORM 9.69293118f
#elif BMI088_OFFLINE_PARAM_ID == 1
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.0007222f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET -0.001786f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.0004346f
#define BMI088_PRE_CALI_G_NORM 9.876785f
#elif BMI088_OFFLINE_PARAM_ID == 2
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.0007222f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET -0.001786f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.0004346f
#define BMI088_PRE_CALI_G_NORM 9.876785f
#elif BMI088_OFFLINE_PARAM_ID == 3
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.00270364084f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET -0.000532632112f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.00478090625f
#define BMI088_PRE_CALI_G_NORM 9.73574924f
#else
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.0007222f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET -0.001786f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.0004346f
#define BMI088_PRE_CALI_G_NORM 9.876785f
#endif

typedef struct
{
    uint8_t reg; // 待配置的BMI088寄存器地址
    uint8_t data; // 写入寄存器的配置值
    BMI088_ERORR_CODE_e error; // 该项失败时记录的bit mask错误码
} BMI088_Init_Reg_Config_t;

static BMI088Instance bmi088_instance;
static uint8_t bmi088_resource_ready = 0U;
static uint8_t bmi088_init_ok = 0U;

/*
 * 高频采样命令固定不变,使用静态TX数组避免1kHz路径里反复memset。
 * SPITransRecv()不会修改发送缓冲区,但其接口历史上使用uint8_t*,因此这里保持非const。
 */
static uint8_t BMI088_Accel_Data_Read_Tx[8] = {
    (uint8_t)(BMI088_ACCEL_XOUT_L | 0x80U), 0x55U, 0x55U, 0x55U,
    0x55U, 0x55U, 0x55U, 0x55U,
};
static uint8_t BMI088_Gyro_Data_Read_Tx[7] = {
    (uint8_t)(BMI088_GYRO_X_L | 0x80U), 0x55U, 0x55U, 0x55U,
    0x55U, 0x55U, 0x55U,
};

// 加速度计基础初始化表:周期模式和触发模式都会写入。
static const BMI088_Init_Reg_Config_t BMI088_Accel_Base_Init_Table[] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
};

// 加速度计触发模式配置表:只有需要DRDY外部中断时才写入。
static const BMI088_Init_Reg_Config_t BMI088_Accel_Trigger_Init_Table[] =
{
    {
        BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
        BMI088_INT1_IO_CTRL_ERROR
    },
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

// 加速度计周期模式配置表:只关闭DRDY映射,保留INT1输出配置,避免MCU侧输入悬空。
static const BMI088_Init_Reg_Config_t BMI088_Accel_Periodic_Init_Table[] =
{
    {BMI088_INT_MAP_DATA, 0x00U, BMI088_INT_MAP_DATA_ERROR},
};

// 陀螺仪基础初始化表:周期模式和触发模式都会写入。
static const BMI088_Init_Reg_Config_t BMI088_Gyro_Base_Init_Table[] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
};

// 陀螺仪触发模式配置表:只有需要DRDY外部中断时才写入。
static const BMI088_Init_Reg_Config_t BMI088_Gyro_Trigger_Init_Table[] =
{
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {
        BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR
    },
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

// 陀螺仪周期模式配置表:显式关闭DRDY输出和映射,避免周期读取时产生无意义EXTI。
static const BMI088_Init_Reg_Config_t BMI088_Gyro_Periodic_Init_Table[] =
{
    {BMI088_GYRO_CTRL, BMI088_DRDY_OFF, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_OFF, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};


static HAL_StatusTypeDef BMI088AccelRead(BMI088Instance* bmi088, uint8_t reg, uint8_t* data, uint8_t len);


static HAL_StatusTypeDef BMI088GyroRead(BMI088Instance* bmi088, uint8_t reg, uint8_t* data, uint8_t len);


static HAL_StatusTypeDef BMI088AccelWriteSingleReg(BMI088Instance* bmi088, uint8_t reg, uint8_t data);


static HAL_StatusTypeDef BMI088GyroWriteSingleReg(BMI088Instance* bmi088, uint8_t reg, uint8_t data);


static HAL_StatusTypeDef BMI088ReadTemperatureDecimated(BMI088Instance* bmi088, float* temperature);


static BMI088_ERORR_CODE_e BMI088AccelInit(BMI088Instance * bmi088);
static BMI088_ERORR_CODE_e BMI088GyroInit(BMI088Instance * bmi088);
static void BMI088LoadOfflineParams(BMI088Instance * bmi088);
static void BMI088SnapshotTriggerCounter(BMI088Instance * bmi088,
                                         uint32_t * acc_drdy_count,
                                         uint32_t * gyro_drdy_count,
                                         uint32_t * acc_read_count,
                                         uint32_t * gyro_read_count);

/**
 * @brief 限频记录运行期读取失败
 *
 * INS任务通常1kHz运行,如果SPI持续异常,每次都打印日志会进一步拖慢系统。
 * 因此前10次全部打印,之后每1000次打印一次。
 */
static void BMI088LogReadFailure(const char* op, HAL_StatusTypeDef status)
{
    static uint32_t fail_count = 0U;

    fail_count++;
    if (fail_count <= 10U || (fail_count % 1000U) == 0U)
    {
        LOGWARNING("[bmi088] %s failed: status=%u, count=%lu",
                   op,
                   (unsigned int)status,
                   (unsigned long)fail_count);
    }
}

/**
 * @brief 读取BMI088加速度计寄存器
 *
 * BMI088加速度计SPI读时,地址字节后需要额外1个dummy byte,
 * 因此总传输长度为len + 2,有效数据从rx[2]开始。
 */
static HAL_StatusTypeDef BMI088AccelRead(BMI088Instance* bmi088, uint8_t reg, uint8_t* data, uint8_t len)
{
    uint8_t tx[8];
    uint8_t rx[8] = {0};
    HAL_StatusTypeDef status;

    if (bmi088 == NULL || bmi088->spi_acc == NULL || data == NULL || len == 0U || len > 6U)
    {
        return HAL_ERROR;
    }

    memset(tx, 0x55, sizeof(tx));
    tx[0] = (uint8_t)(reg | 0x80U);

    status = SPITransRecv(bmi088->spi_acc, rx, tx, (uint16_t)(len + 2U));
    if (status == HAL_OK)
    {
        memcpy(data, &rx[2], len);
    }

    return status;
}

/**
 * @brief 读取BMI088陀螺仪寄存器
 *
 * BMI088陀螺仪SPI读时只有地址字节这1个dummy阶段,
 * 因此总传输长度为len + 1,有效数据从rx[1]开始。
 */
static HAL_StatusTypeDef BMI088GyroRead(BMI088Instance* bmi088, uint8_t reg, uint8_t* data, uint8_t len)
{
    uint8_t tx[9];
    uint8_t rx[9] = {0};
    HAL_StatusTypeDef status;

    if (bmi088 == NULL || bmi088->spi_gyro == NULL || data == NULL || len == 0U || len > 8U)
    {
        return HAL_ERROR;
    }

    memset(tx, 0x55, sizeof(tx));
    tx[0] = (uint8_t)(reg | 0x80U);

    status = SPITransRecv(bmi088->spi_gyro, rx, tx, (uint16_t)(len + 1U));
    if (status == HAL_OK)
    {
        memcpy(data, &rx[1], len);
    }

    return status;
}

static HAL_StatusTypeDef BMI088AccelWriteSingleReg(BMI088Instance* bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};

    if (bmi088 == NULL || bmi088->spi_acc == NULL)
    {
        return HAL_ERROR;
    }

    return SPITransmit(bmi088->spi_acc, tx, sizeof(tx));
}

static HAL_StatusTypeDef BMI088GyroWriteSingleReg(BMI088Instance* bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};

    if (bmi088 == NULL || bmi088->spi_gyro == NULL)
    {
        return HAL_ERROR;
    }

    return SPITransmit(bmi088->spi_gyro, tx, sizeof(tx));
}

static int16_t BMI088MakeInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/**
 * @brief 解码BMI088温度
 *
 * BMI088温度是11位有符号数,官方例程在原始值大于1023时减2048完成符号扩展。
 */
static float BMI088DecodeTemperature(const uint8_t temp_buf[2])
{
    int16_t raw_temp = (int16_t)(((uint16_t)temp_buf[0] << 3) | ((uint16_t)temp_buf[1] >> 5));

    if (raw_temp > 1023)
    {
        raw_temp -= 2048;
    }

    return (float)raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

static void BMI088DecodeAccel(BMI088Instance* bmi088, const uint8_t buf[6], float acc[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        const int16_t raw = BMI088MakeInt16(buf[2U * i], buf[2U * i + 1U]);
        acc[i] = (float)raw * bmi088->acc_coef;
    }
}

static void BMI088DecodeGyro(BMI088Instance* bmi088, const uint8_t buf[6], float gyro[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        const int16_t raw = BMI088MakeInt16(buf[2U * i], buf[2U * i + 1U]);
        gyro[i] = (float)raw * bmi088->BMI088_GYRO_SEN - bmi088->gyro_offset[i];
    }
}

static void BMI088StoreData(BMI088Instance* bmi088, const BMI088_Data_t* data)
{
    bmi088->acc[0] = data->acc[0];
    bmi088->acc[1] = data->acc[1];
    bmi088->acc[2] = data->acc[2];
    bmi088->gyro[0] = data->gyro[0];
    bmi088->gyro[1] = data->gyro[1];
    bmi088->gyro[2] = data->gyro[2];
    bmi088->temperature = data->temperature;
}

static void BMI088SetDefaultScale(BMI088Instance* bmi088)
{
    bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
    bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    bmi088->acc_coef = bmi088->BMI088_ACCEL_SEN;
    bmi088->gNorm = BMI088_STANDARD_GRAVITY;
    bmi088->temp_when_cali = 40.0f;
    bmi088->temperature = 40.0f;
    bmi088->temp_read_count = BMI088_TEMP_UPDATE_DIVIDER - 1U;
    memset(bmi088->gyro_offset, 0, sizeof(bmi088->gyro_offset));
}

static void BMI088LoadOfflineParams(BMI088Instance* bmi088)
{
    if (bmi088 == NULL)
    {
        return;
    }

    bmi088->gyro_offset[0] = BMI088_PRE_CALI_GYRO_X_OFFSET;
    bmi088->gyro_offset[1] = BMI088_PRE_CALI_GYRO_Y_OFFSET;
    bmi088->gyro_offset[2] = BMI088_PRE_CALI_GYRO_Z_OFFSET;
    bmi088->gNorm = BMI088_PRE_CALI_G_NORM;
    bmi088->acc_coef = bmi088->BMI088_ACCEL_SEN * BMI088_STANDARD_GRAVITY / bmi088->gNorm;
    bmi088->temp_when_cali = 40.0f;
    bmi088->temperature = 40.0f;
    bmi088->temp_read_count = BMI088_TEMP_UPDATE_DIVIDER - 1U;
}

static void BMI088SnapshotTriggerCounter(BMI088Instance* bmi088,
                                         uint32_t* acc_drdy_count,
                                         uint32_t* gyro_drdy_count,
                                         uint32_t* acc_read_count,
                                         uint32_t* gyro_read_count)
{
    uint32_t primask;

    if (bmi088 == NULL || acc_drdy_count == NULL || gyro_drdy_count == NULL)
    {
        return;
    }

    /*
     * 触发模式下DRDY计数可能由BSP服务任务更新,INS任务同时读取。
     * 这里用极短临界区拿到一组自洽快照,后续用读前/读后快照判断SPI读取期间
     * 是否又发生了新的传感器更新。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    *acc_drdy_count = bmi088->acc_drdy_count;
    *gyro_drdy_count = bmi088->gyro_drdy_count;
    if (acc_read_count != NULL)
    {
        *acc_read_count = bmi088->acc_read_count;
    }
    if (gyro_read_count != NULL)
    {
        *gyro_read_count = bmi088->gyro_read_count;
    }
    __set_PRIMASK(primask);
}

static BMI088_ERORR_CODE_e BMI088AccelInit(BMI088Instance* bmi088)
{
    uint8_t whoami = 0U;
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    HAL_StatusTypeDef status;

    /*
     * BMI088加速度计上电后默认可能处于I2C接口识别状态。
     * 官方例程先读两次CHIP_ID,通过片选上升沿让芯片稳定进入SPI访问路径。
     */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        status = BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami, 1U);
        if (status != HAL_OK)
        {
            LOGERROR("[bmi088] acc chip id pre-read failed, status=%u", (unsigned int)status);
            return BMI088_NO_SENSOR;
        }
        DWT_Delay(BMI088_COM_WAIT_S);
    }

    status = BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] acc soft reset failed, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(BMI088_LONG_WAIT_S);

    for (uint8_t i = 0U; i < 2U; i++)
    {
        status = BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami, 1U);
        if (status != HAL_OK)
        {
            LOGERROR("[bmi088] acc chip id read after reset failed, status=%u", (unsigned int)status);
            return BMI088_NO_SENSOR;
        }
        DWT_Delay(BMI088_COM_WAIT_S);
    }

    if (whoami != BMI088_ACC_CHIP_ID_VALUE)
    {
        LOGERROR("[bmi088] acc chip id mismatch: read=0x%x, expect=0x%x",
                 (unsigned int)whoami,
                 (unsigned int)BMI088_ACC_CHIP_ID_VALUE);
        return BMI088_NO_SENSOR;
    }

    for (size_t i = 0U; i < BMI088_ARRAY_LEN(BMI088_Accel_Base_Init_Table); i++)
    {
        const uint8_t reg = BMI088_Accel_Base_Init_Table[i].reg;
        const uint8_t expected = BMI088_Accel_Base_Init_Table[i].data;
        uint8_t readback = 0U;

        status = BMI088AccelWriteSingleReg(bmi088, reg, expected);
        if (status != HAL_OK)
        {
            error |= BMI088_Accel_Base_Init_Table[i].error;
            LOGERROR("[bmi088] acc write reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        status = BMI088AccelRead(bmi088, reg, &readback, 1U);
        if (status != HAL_OK)
        {
            error |= BMI088_Accel_Base_Init_Table[i].error;
            LOGERROR("[bmi088] acc read reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        if (readback != expected)
        {
            error |= BMI088_Accel_Base_Init_Table[i].error;
            LOGERROR("[bmi088] acc reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)reg,
                     (unsigned int)readback,
                     (unsigned int)expected);
        }
    }

    /*
     * 周期模式不需要BMI088输出DRDY到外部中断脚;触发模式才打开DRDY映射。
     * 这里显式写关闭配置,可以避免上一次固件/运行模式残留导致周期模式仍然产生EXTI。
     */
    const BMI088_Init_Reg_Config_t* mode_table =
        (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
            ? BMI088_Accel_Trigger_Init_Table
            : BMI088_Accel_Periodic_Init_Table;
    const size_t mode_table_len =
        (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
            ? BMI088_ARRAY_LEN(BMI088_Accel_Trigger_Init_Table)
            : BMI088_ARRAY_LEN(BMI088_Accel_Periodic_Init_Table);

    for (size_t i = 0U; i < mode_table_len; i++)
    {
        const uint8_t reg = mode_table[i].reg;
        const uint8_t expected = mode_table[i].data;
        uint8_t readback = 0U;

        status = BMI088AccelWriteSingleReg(bmi088, reg, expected);
        if (status != HAL_OK)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] acc write mode reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        status = BMI088AccelRead(bmi088, reg, &readback, 1U);
        if (status != HAL_OK)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] acc read mode reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        if (readback != expected)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] acc mode reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)reg,
                     (unsigned int)readback,
                     (unsigned int)expected);
        }
    }

    return error;
}

static BMI088_ERORR_CODE_e BMI088GyroInit(BMI088Instance* bmi088)
{
    uint8_t whoami = 0U;
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    HAL_StatusTypeDef status;

    for (uint8_t i = 0U; i < 2U; i++)
    {
        status = BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &whoami, 1U);
        if (status != HAL_OK)
        {
            LOGERROR("[bmi088] gyro chip id pre-read failed, status=%u", (unsigned int)status);
            return BMI088_NO_SENSOR;
        }
        DWT_Delay(BMI088_COM_WAIT_S);
    }

    status = BMI088GyroWriteSingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] gyro soft reset failed, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(BMI088_LONG_WAIT_S);

    for (uint8_t i = 0U; i < 2U; i++)
    {
        status = BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &whoami, 1U);
        if (status != HAL_OK)
        {
            LOGERROR("[bmi088] gyro chip id read after reset failed, status=%u", (unsigned int)status);
            return BMI088_NO_SENSOR;
        }
        DWT_Delay(BMI088_COM_WAIT_S);
    }

    if (whoami != BMI088_GYRO_CHIP_ID_VALUE)
    {
        LOGERROR("[bmi088] gyro chip id mismatch: read=0x%x, expect=0x%x",
                 (unsigned int)whoami,
                 (unsigned int)BMI088_GYRO_CHIP_ID_VALUE);
        return BMI088_NO_SENSOR;
    }

    for (size_t i = 0U; i < BMI088_ARRAY_LEN(BMI088_Gyro_Base_Init_Table); i++)
    {
        const uint8_t reg = BMI088_Gyro_Base_Init_Table[i].reg;
        const uint8_t expected = BMI088_Gyro_Base_Init_Table[i].data;
        uint8_t readback = 0U;

        status = BMI088GyroWriteSingleReg(bmi088, reg, expected);
        if (status != HAL_OK)
        {
            error |= BMI088_Gyro_Base_Init_Table[i].error;
            LOGERROR("[bmi088] gyro write reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        status = BMI088GyroRead(bmi088, reg, &readback, 1U);
        if (status != HAL_OK)
        {
            error |= BMI088_Gyro_Base_Init_Table[i].error;
            LOGERROR("[bmi088] gyro read reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        if (readback != expected)
        {
            error |= BMI088_Gyro_Base_Init_Table[i].error;
            LOGERROR("[bmi088] gyro reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)reg,
                     (unsigned int)readback,
                     (unsigned int)expected);
        }
    }

    /*
     * 陀螺仪DRDY需要先由GYRO_CTRL打开,再映射到INT3/INT4。
     * 周期模式下反向写入关闭配置,让BMI088不再主动打断MCU。
     */
    const BMI088_Init_Reg_Config_t* mode_table =
        (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
            ? BMI088_Gyro_Trigger_Init_Table
            : BMI088_Gyro_Periodic_Init_Table;
    const size_t mode_table_len =
        (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
            ? BMI088_ARRAY_LEN(BMI088_Gyro_Trigger_Init_Table)
            : BMI088_ARRAY_LEN(BMI088_Gyro_Periodic_Init_Table);

    for (size_t i = 0U; i < mode_table_len; i++)
    {
        const uint8_t reg = mode_table[i].reg;
        const uint8_t expected = mode_table[i].data;
        uint8_t readback = 0U;

        status = BMI088GyroWriteSingleReg(bmi088, reg, expected);
        if (status != HAL_OK)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] gyro write mode reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        status = BMI088GyroRead(bmi088, reg, &readback, 1U);
        if (status != HAL_OK)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] gyro read mode reg 0x%x failed, status=%u", (unsigned int)reg, (unsigned int)status);
            continue;
        }
        DWT_Delay(BMI088_COM_WAIT_S);

        if (readback != expected)
        {
            error |= mode_table[i].error;
            LOGERROR("[bmi088] gyro mode reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)reg,
                     (unsigned int)readback,
                     (unsigned int)expected);
        }
    }

    return error;
}

static void BMI088MarkAccReady(BMI088Instance* bmi088)
{
    uint32_t primask;

    if (bmi088 == NULL)
    {
        return;
    }

    /*
     * DRDY事件可能由BSP服务任务写入,而INS任务同时读取计数。
     * 用极短临界区保护计数和标志位,保证触发模式下事件不会被部分更新状态干扰。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (bmi088->acc_drdy_count != bmi088->acc_read_count)
    {
        bmi088->update_flag.acc_overrun = 1U;
    }
    bmi088->acc_drdy_count++;
    bmi088->update_flag.acc = 1U;
    bmi088->update_flag.imu_ready = 1U;
    __set_PRIMASK(primask);
}

static void BMI088MarkGyroReady(BMI088Instance* bmi088)
{
    uint32_t primask;

    if (bmi088 == NULL)
    {
        return;
    }

    // 与加速度计DRDY相同,保持计数和标志位作为一个整体更新。
    primask = __get_PRIMASK();
    __disable_irq();
    if (bmi088->gyro_drdy_count != bmi088->gyro_read_count)
    {
        bmi088->update_flag.gyro_overrun = 1U;
    }
    bmi088->gyro_drdy_count++;
    bmi088->update_flag.gyro = 1U;
    bmi088->update_flag.imu_ready = 1U;
    __set_PRIMASK(primask);
}

static void BMI088FinishAccRead(BMI088Instance* bmi088, uint32_t handled_count)
{
    uint32_t primask;

    if (bmi088 == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    bmi088->acc_read_count = handled_count;
    if (bmi088->acc_drdy_count == bmi088->acc_read_count)
    {
        bmi088->update_flag.acc = 0U;
        bmi088->update_flag.acc_overrun = 0U;
    }
    bmi088->update_flag.imu_ready = (bmi088->update_flag.acc || bmi088->update_flag.gyro || bmi088->update_flag.temp)
                                        ? 1U
                                        : 0U;

    __set_PRIMASK(primask);
}

static void BMI088FinishGyroRead(BMI088Instance* bmi088, uint32_t handled_count)
{
    uint32_t primask;

    if (bmi088 == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    bmi088->gyro_read_count = handled_count;
    if (bmi088->gyro_drdy_count == bmi088->gyro_read_count)
    {
        bmi088->update_flag.gyro = 0U;
        bmi088->update_flag.gyro_overrun = 0U;
    }
    bmi088->update_flag.imu_ready = (bmi088->update_flag.acc || bmi088->update_flag.gyro || bmi088->update_flag.temp)
                                        ? 1U
                                        : 0U;

    __set_PRIMASK(primask);
}

static void BMI088AccINTCallback(GPIOInstance* gpio)
{
    if (gpio != NULL)
    {
        BMI088MarkAccReady((BMI088Instance*)gpio->id);
    }
}

static void BMI088GyroINTCallback(GPIOInstance* gpio)
{
    if (gpio != NULL)
    {
        BMI088MarkGyroReady((BMI088Instance*)gpio->id);
    }
}

static HAL_StatusTypeDef BMI088ReadAccelData(BMI088Instance* bmi088, float acc[3])
{
    uint8_t rx[8] = {0};
    HAL_StatusTypeDef status;

    if (bmi088 == NULL || bmi088->spi_acc == NULL || acc == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * 高频采样路径直接从SPI接收缓冲区解码,避免先复制到6字节中间buffer。
     * 加速度计读6字节数据时总线实际传输:地址 + dummy + 6字节数据。
     */
    status = SPITransRecv(bmi088->spi_acc, rx, BMI088_Accel_Data_Read_Tx, (uint16_t)sizeof(rx));
    if (status == HAL_OK)
    {
        BMI088DecodeAccel(bmi088, &rx[2], acc);
    }

    return status;
}

static HAL_StatusTypeDef BMI088ReadGyroData(BMI088Instance* bmi088, float gyro[3])
{
    uint8_t rx[7] = {0};
    HAL_StatusTypeDef status;

    if (bmi088 == NULL || bmi088->spi_gyro == NULL || gyro == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * 陀螺仪读6字节数据时总线实际传输:地址 + 6字节数据。
     * 直接从rx[1]开始解码,减少高频路径中的一次memcpy。
     */
    status = SPITransRecv(bmi088->spi_gyro, rx, BMI088_Gyro_Data_Read_Tx, (uint16_t)sizeof(rx));
    if (status == HAL_OK)
    {
        BMI088DecodeGyro(bmi088, &rx[1], gyro);
    }

    return status;
}

static HAL_StatusTypeDef BMI088ReadTemperature(BMI088Instance* bmi088, float* temperature)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef status;

    if (temperature == NULL)
    {
        return HAL_ERROR;
    }

    status = BMI088AccelRead(bmi088, BMI088_TEMP_M, buf, sizeof(buf));
    if (status == HAL_OK)
    {
        *temperature = BMI088DecodeTemperature(buf);
    }

    return status;
}

/**
 * @brief 按固定比例降低温度读取频率
 *
 * 姿态解算需要1kHz级别更新加速度和陀螺仪,但BMI088温度变化很慢,
 * 每个姿态周期都读取温度会额外增加一次SPI事务。这里每
 * BMI088_TEMP_UPDATE_DIVIDER 次采样才真正访问一次温度寄存器,
 * 其余周期沿用最近一次有效温度,从而缩短BMI088Acquire()的常规耗时。
 */
static HAL_StatusTypeDef BMI088ReadTemperatureDecimated(BMI088Instance* bmi088, float* temperature)
{
    if (bmi088 == NULL || temperature == NULL)
    {
        return HAL_ERROR;
    }

    bmi088->temp_read_count++;
    if (bmi088->temp_read_count < BMI088_TEMP_UPDATE_DIVIDER)
    {
        return HAL_OK;
    }

    bmi088->temp_read_count = 0U;
    HAL_StatusTypeDef status = BMI088ReadTemperature(bmi088, temperature);
    if (status != HAL_OK)
    {
        // 温度读取失败后下一次采样立即重试,避免温控长期停留在旧温度。
        bmi088->temp_read_count = BMI088_TEMP_UPDATE_DIVIDER - 1U;
    }

    return status;
}

uint8_t BMI088Acquire(BMI088Instance* bmi088, BMI088_Data_t* data_store)
{
    HAL_StatusTypeDef status;

    if (bmi088 == NULL || data_store == NULL)
    {
        return 0U;
    }

    if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE)
    {
        data_store->temperature = bmi088->temperature;

        status = BMI088ReadAccelData(bmi088, data_store->acc);
        if (status != HAL_OK)
        {
            BMI088LogReadFailure("read acc", status);
            return 0U;
        }

        status = BMI088ReadGyroData(bmi088, data_store->gyro);
        if (status != HAL_OK)
        {
            BMI088LogReadFailure("read gyro", status);
            return 0U;
        }

        status = BMI088ReadTemperatureDecimated(bmi088, &data_store->temperature);
        if (status != HAL_OK)
        {
            /*
             * 温度只用于慢速热控,温度读取失败不应阻断1kHz姿态更新。
             * 此时沿用上一帧温度,并通过限频日志提示问题。
             */
            BMI088LogReadFailure("read temp", status);
        }

        BMI088StoreData(bmi088, data_store);
        return 1U;
    }

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        for (uint8_t retry = 0U; retry < BMI088_TRIGGER_SYNC_RETRY_MAX; retry++)
        {
            uint32_t acc_target_count = 0U;
            uint32_t gyro_target_count = 0U;
            uint32_t acc_read_count = 0U;
            uint32_t gyro_read_count = 0U;
            uint32_t acc_after_count = 0U;
            uint32_t gyro_after_count = 0U;
            uint8_t acc_pending;
            uint8_t gyro_pending;
            BMI088_Data_t local_data;

            /*
             * 触发模式以"完整IMU帧"作为上层可用数据的边界。
             * 先带入上一帧数据,随后只有在acc和gyro两路DRDY都到达后才读取并发布新帧。
             */
            memcpy(local_data.acc, bmi088->acc, sizeof(local_data.acc));
            memcpy(local_data.gyro, bmi088->gyro, sizeof(local_data.gyro));
            local_data.temperature = bmi088->temperature;

            BMI088SnapshotTriggerCounter(bmi088,
                                         &acc_target_count,
                                         &gyro_target_count,
                                         &acc_read_count,
                                         &gyro_read_count);
            acc_pending = (acc_target_count != acc_read_count) ? 1U : 0U;
            gyro_pending = (gyro_target_count != gyro_read_count) ? 1U : 0U;

            /*
             * INS融合需要一帧完整的acc+gyro数据。若只到了一路DRDY,先不读取也不清标志,
             * 等另一路DRDY到来后再一次性读取两路,避免上层把半帧误认为完整新数据。
             */
            if (acc_pending == 0U || gyro_pending == 0U)
            {
                return 0U;
            }

            status = BMI088ReadAccelData(bmi088, local_data.acc);
            if (status != HAL_OK)
            {
                BMI088LogReadFailure("trigger read acc", status);
                return 0U;
            }

            status = BMI088ReadGyroData(bmi088, local_data.gyro);
            if (status != HAL_OK)
            {
                BMI088LogReadFailure("trigger read gyro", status);
                return 0U;
            }

            status = BMI088ReadTemperatureDecimated(bmi088, &local_data.temperature);
            if (status != HAL_OK)
            {
                BMI088LogReadFailure("trigger read temp", status);
            }

            BMI088SnapshotTriggerCounter(bmi088,
                                         &acc_after_count,
                                         &gyro_after_count,
                                         NULL,
                                         NULL);

            /*
             * 若SPI读取期间又来了新的DRDY,说明寄存器可能已经被下一次采样刷新。
             * 这种情况下不发布当前数据,而是立即用新的DRDY快照重读一次,避免上层拿到
             * 跨采样窗口的acc/gyro组合。连续重试仍不稳定时,丢弃已观察到的事件,
             * 等下一对DRDY重新形成一帧,避免事件积压越来越多。
             */
            if (acc_after_count != acc_target_count || gyro_after_count != gyro_target_count)
            {
                if ((retry + 1U) >= BMI088_TRIGGER_SYNC_RETRY_MAX)
                {
                    BMI088FinishAccRead(bmi088, acc_after_count);
                    BMI088FinishGyroRead(bmi088, gyro_after_count);
                    BMI088LogReadFailure("trigger sync unstable", HAL_BUSY);
                    return 0U;
                }

                continue;
            }

            BMI088StoreData(bmi088, &local_data);
            BMI088FinishAccRead(bmi088, acc_target_count);
            BMI088FinishGyroRead(bmi088, gyro_target_count);
            *data_store = local_data;
            return 1U;
        }

        return 0U;
    }

    return 0U;
}

void BMI088CalibrateIMU(BMI088Instance* bmi088)
{
    BMI088_Work_Mode_e saved_mode;
    float start_time;

    if (bmi088 == NULL)
    {
        return;
    }

    if (bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE)
    {
        BMI088LoadOfflineParams(bmi088);
        return;
    }

    saved_mode = bmi088->work_mode;
    bmi088->work_mode = BMI088_BLOCK_PERIODIC_MODE;

    start_time = DWT_GetTimeline_s();
    do
    {
        BMI088_Data_t sample = {0};
        float gyro_sum[3] = {0.0f};
        float gyro_max[3] = {0.0f};
        float gyro_min[3] = {0.0f};
        float g_norm_sum = 0.0f;
        float g_norm_max = 0.0f;
        float g_norm_min = 0.0f;
        float gyro_diff[3] = {0.0f};
        float g_norm_diff = 0.0f;
        float candidate_g_norm = 0.0f;
        float candidate_gyro_offset[3] = {0.0f};
        float candidate_acc_coef = 0.0f;
        uint16_t valid_samples = 0U;

        /*
         * 每一轮标定都必须基于未修正的基础数据。
         * 若上一轮候选结果没有通过最终检查,它不能继续参与下一轮采样解码。
         */
        bmi088->acc_coef = bmi088->BMI088_ACCEL_SEN;
        memset(bmi088->gyro_offset, 0, sizeof(bmi088->gyro_offset));

        if ((DWT_GetTimeline_s() - start_time) > BMI088_CALIBRATE_TIMEOUT_S)
        {
            LOGERROR("[bmi088] calibration timeout, load offline params");
            bmi088->cali_mode = BMI088_LOAD_PRE_CALI_MODE;
            break;
        }

        for (uint16_t i = 0U; i < BMI088_CALIBRATE_SAMPLE_COUNT; i++)
        {
            float g_norm;

            if ((DWT_GetTimeline_s() - start_time) > BMI088_CALIBRATE_TIMEOUT_S)
            {
                LOGERROR("[bmi088] calibration timeout during sampling, load offline params");
                bmi088->cali_mode = BMI088_LOAD_PRE_CALI_MODE;
                break;
            }

            if (BMI088Acquire(bmi088, &sample) == 0U)
            {
                LOGWARNING("[bmi088] calibration sample read failed");
                break;
            }

            g_norm = NormOf3d(sample.acc);

            if (valid_samples == 0U)
            {
                g_norm_max = g_norm;
                g_norm_min = g_norm;
                for (uint8_t axis = 0U; axis < 3U; axis++)
                {
                    gyro_max[axis] = sample.gyro[axis];
                    gyro_min[axis] = sample.gyro[axis];
                }
            }
            else
            {
                g_norm_max = (g_norm > g_norm_max) ? g_norm : g_norm_max;
                g_norm_min = (g_norm < g_norm_min) ? g_norm : g_norm_min;
                for (uint8_t axis = 0U; axis < 3U; axis++)
                {
                    gyro_max[axis] = (sample.gyro[axis] > gyro_max[axis]) ? sample.gyro[axis] : gyro_max[axis];
                    gyro_min[axis] = (sample.gyro[axis] < gyro_min[axis]) ? sample.gyro[axis] : gyro_min[axis];
                }
            }

            g_norm_sum += g_norm;
            for (uint8_t axis = 0U; axis < 3U; axis++)
            {
                gyro_sum[axis] += sample.gyro[axis];
                gyro_diff[axis] = gyro_max[axis] - gyro_min[axis];
            }
            g_norm_diff = g_norm_max - g_norm_min;
            valid_samples++;

            if (g_norm_diff > BMI088_CALIBRATE_G_NORM_DIFF_LIMIT ||
                gyro_diff[0] > BMI088_CALIBRATE_GYRO_DIFF_LIMIT ||
                gyro_diff[1] > BMI088_CALIBRATE_GYRO_DIFF_LIMIT ||
                gyro_diff[2] > BMI088_CALIBRATE_GYRO_DIFF_LIMIT)
            {
                LOGWARNING("[bmi088] calibration interrupted by motion");
                break;
            }

            DWT_Delay(BMI088_CALIBRATE_SAMPLE_DELAY_S);
        }

        if (bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE)
        {
            break;
        }

        if (valid_samples == 0U)
        {
            /*
             * 可能是上电早期偶发SPI失败,不要立刻使用离线参数。
             * 继续重试直到总超时,让在线标定有机会恢复。
             */
            LOGWARNING("[bmi088] calibration sample incomplete: no valid sample");
            DWT_Delay(BMI088_CALIBRATE_RETRY_DELAY_S);
            continue;
        }

        if (valid_samples < BMI088_CALIBRATE_SAMPLE_COUNT)
        {
            /*
             * 在线标定需要足够长的静止采样窗口。若中途SPI读取失败或检测到明显运动,
             * 本轮样本不用于生成零偏,直接进入下一轮重试。
             */
            LOGWARNING("[bmi088] calibration sample incomplete: %u/%u",
                       (unsigned int)valid_samples,
                       (unsigned int)BMI088_CALIBRATE_SAMPLE_COUNT);
            DWT_Delay(BMI088_CALIBRATE_RETRY_DELAY_S);
            continue;
        }

        /*
         * 候选结果先保存在局部变量中。只有通过最终合理性检查后,
         * 才一次性写入BMI088实例,避免失败候选值污染下一轮重试。
         */
        candidate_g_norm = g_norm_sum / (float)valid_samples;
        if (candidate_g_norm < BMI088_CALIBRATE_MIN_G_NORM)
        {
            LOGWARNING("[bmi088] calibration candidate rejected: invalid g norm");
            DWT_Delay(BMI088_CALIBRATE_RETRY_DELAY_S);
            continue;
        }

        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            candidate_gyro_offset[axis] = gyro_sum[axis] / (float)valid_samples;
        }
        candidate_acc_coef = bmi088->BMI088_ACCEL_SEN * BMI088_STANDARD_GRAVITY / candidate_g_norm;

        if (g_norm_diff <= BMI088_CALIBRATE_G_NORM_DIFF_LIMIT &&
            fabsf(candidate_g_norm - BMI088_STANDARD_GRAVITY) <= BMI088_CALIBRATE_G_NORM_ERROR_LIMIT &&
            gyro_diff[0] <= BMI088_CALIBRATE_GYRO_DIFF_LIMIT &&
            gyro_diff[1] <= BMI088_CALIBRATE_GYRO_DIFF_LIMIT &&
            gyro_diff[2] <= BMI088_CALIBRATE_GYRO_DIFF_LIMIT &&
            fabsf(candidate_gyro_offset[0]) <= BMI088_CALIBRATE_GYRO_OFFSET_LIMIT &&
            fabsf(candidate_gyro_offset[1]) <= BMI088_CALIBRATE_GYRO_OFFSET_LIMIT &&
            fabsf(candidate_gyro_offset[2]) <= BMI088_CALIBRATE_GYRO_OFFSET_LIMIT)
        {
            bmi088->gNorm = candidate_g_norm;
            for (uint8_t axis = 0U; axis < 3U; axis++)
            {
                bmi088->gyro_offset[axis] = candidate_gyro_offset[axis];
            }
            bmi088->temp_when_cali = sample.temperature;
            bmi088->acc_coef = candidate_acc_coef;
            bmi088->work_mode = saved_mode;
            return;
        }

        LOGWARNING(
            "[bmi088] calibration candidate rejected: g_norm_milli=%ld, g_diff_milli=%ld, gyro_offset_uradps=[%ld,%ld,%ld]",
            (long)(candidate_g_norm * 1000.0f),
            (long)(g_norm_diff * 1000.0f),
            (long)(candidate_gyro_offset[0] * 1000000.0f),
            (long)(candidate_gyro_offset[1] * 1000000.0f),
            (long)(candidate_gyro_offset[2] * 1000000.0f));
    }
    while (bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE);

    if (bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE)
    {
        LOGERROR("[bmi088] online calibration failed, use offline params");
        BMI088LoadOfflineParams(bmi088);
    }

    bmi088->work_mode = saved_mode;
}

BMI088Instance* BMI088Register(BMI088_Init_Config_s* config)
{
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;

    if (config == NULL)
    {
        LOGERROR("[bmi088] register failed: config is null");
        return NULL;
    }

    if (bmi088_init_ok != 0U)
    {
        return &bmi088_instance;
    }

    if (bmi088_resource_ready == 0U)
    {
        memset(&bmi088_instance, 0, sizeof(bmi088_instance));
        BMI088SetDefaultScale(&bmi088_instance);

        /*
         * 初始化发生在RobotInit阶段时RTOS尚未启动、全局中断也被关闭,
         * 因此BMI088底层寄存器访问统一使用阻塞SPI。后续即使使用DRDY事件,
         * 也只在任务上下文同步读取,避免在中断中启动SPI事务。
         */
        config->spi_acc_config.spi_work_mode = SPI_BLOCK_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_BLOCK_MODE;
        config->spi_acc_config.id = &bmi088_instance;
        config->spi_gyro_config.id = &bmi088_instance;

        bmi088_instance.work_mode = config->work_mode;
        bmi088_instance.cali_mode = config->cali_mode;
        bmi088_instance.spi_acc = SPIRegister(&config->spi_acc_config);
        bmi088_instance.spi_gyro = SPIRegister(&config->spi_gyro_config);

        bmi088_resource_ready = 1U;
    }
    else
    {
        bmi088_instance.work_mode = config->work_mode;
        bmi088_instance.cali_mode = config->cali_mode;
    }

    for (uint8_t retry = 0U; retry < BMI088_INIT_RETRY_MAX; retry++)
    {
        error = BMI088_NO_ERROR;
        error |= BMI088AccelInit(&bmi088_instance);
        error |= BMI088GyroInit(&bmi088_instance);
        if (error == BMI088_NO_ERROR)
        {
            break;
        }

        LOGWARNING("[bmi088] init retry %u/%u failed, error=0x%x",
                   (unsigned int)(retry + 1U),
                   (unsigned int)BMI088_INIT_RETRY_MAX,
                   (unsigned int)error);
        DWT_Delay(0.01f);
    }

    if (error != BMI088_NO_ERROR)
    {
        LOGERROR("[bmi088] init failed after retries, error=0x%x", (unsigned int)error);
        return NULL;
    }

    BMI088CalibrateIMU(&bmi088_instance);
    bmi088_instance.work_mode = config->work_mode;

    if (config->work_mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        config->acc_int_config.id = &bmi088_instance;
        config->gyro_int_config.id = &bmi088_instance;
        config->acc_int_config.gpio_model_callback = BMI088AccINTCallback;
        config->gyro_int_config.gpio_model_callback = BMI088GyroINTCallback;

        if (bmi088_instance.acc_int == NULL)
        {
            bmi088_instance.acc_int = GPIORegister(&config->acc_int_config);
        }
        if (bmi088_instance.gyro_int == NULL)
        {
            bmi088_instance.gyro_int = GPIORegister(&config->gyro_int_config);
        }

        if (bmi088_instance.acc_int == NULL || bmi088_instance.gyro_int == NULL)
        {
            LOGERROR("[bmi088] trigger mode gpio register failed");
            return NULL;
        }

        /*
         * EXTI15_10_IRQn由CubeMX在Core/Src/gpio.c中配置和使能。
         * BMI088模块只注册GPIO回调,不在模块层改动NVIC优先级。
         */
    }

    bmi088_init_ok = 1U;
    return &bmi088_instance;
}
