#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_spi.h"
#include <math.h>
#include <string.h>

#pragma message "this is a legacy support. test the new BMI088 module as soon as possible."

float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;

IMU_Data_t BMI088;

#if defined(BMI088_USE_SPI)

static SPIInstance *bmi088_acc_spi = NULL;
static SPIInstance *bmi088_gyro_spi = NULL;

#define BMI088_accel_write_single_reg(reg, data) BMI088AccelWriteSingleReg((reg), (data))
#define BMI088_accel_read_single_reg(reg, data) BMI088AccelReadSingleReg((reg), &(data))
#define BMI088_accel_read_muli_reg(reg, data, len) BMI088AccelReadMultiReg((reg), (data), (len))

#define BMI088_gyro_write_single_reg(reg, data) BMI088GyroWriteSingleReg((reg), (data))
#define BMI088_gyro_read_single_reg(reg, data) BMI088GyroReadSingleReg((reg), &(data))
#define BMI088_gyro_read_muli_reg(reg, data, len) BMI088GyroReadMultiReg((reg), (data), (len))

static void BMI088RegisterSPIBus(SPI_HandleTypeDef *bmi088_SPI);
static HAL_StatusTypeDef BMI088AccelWriteSingleReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef BMI088AccelReadSingleReg(uint8_t reg, uint8_t *return_data);
static HAL_StatusTypeDef BMI088AccelReadMultiReg(uint8_t reg, uint8_t *buf, uint8_t len);
static HAL_StatusTypeDef BMI088GyroWriteSingleReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef BMI088GyroReadSingleReg(uint8_t reg, uint8_t *return_data);
static HAL_StatusTypeDef BMI088GyroReadMultiReg(uint8_t reg, uint8_t *buf, uint8_t len);
static void BMI088LoadOfflineParams(IMU_Data_t *bmi088);
static void BMI088LogReadFailure(const char *op, HAL_StatusTypeDef status);

#elif defined(BMI088_USE_IIC)
#endif

static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088);

static void BMI088LoadOfflineParams(IMU_Data_t *bmi088)
{
    if (bmi088 == NULL)
    {
        return;
    }

    bmi088->GyroOffset[0] = GxOFFSET;
    bmi088->GyroOffset[1] = GyOFFSET;
    bmi088->GyroOffset[2] = GzOFFSET;
    bmi088->gNorm = gNORM;
    bmi088->AccelScale = 9.81f / bmi088->gNorm;
    bmi088->TempWhenCali = 40;
}

/**
 * @brief 限频记录BMI088运行期读取失败
 *
 * INS任务以1kHz运行,若SPI持续异常,每次都打印日志会进一步拖慢系统。
 * 因此前10次全部打印,之后每1000次打印一次,既能看到故障,也避免刷屏。
 */
static void BMI088LogReadFailure(const char *op, HAL_StatusTypeDef status)
{
    static uint32_t fail_count = 0;

    fail_count++;
    if (fail_count <= 10U || (fail_count % 1000U) == 0U)
    {
        LOGWARNING("[bmi088] %s failed: status=%u, count=%lu",
                   op,
                   (unsigned int)status,
                   (unsigned long)fail_count);
    }
}

uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    BMI088RegisterSPIBus(bmi088_SPI);
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    if (error != BMI088_NO_ERROR)
    {
        return error;
    }

    if (calibrate)
    {
        Calibrate_MPU_Offset(&BMI088);
    }
    else
    {
        BMI088LoadOfflineParams(&BMI088);
    }

    return error;
}

void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 6000;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;
    uint16_t valid_samples;
    HAL_StatusTypeDef status;

    startTime = DWT_GetTimeline_s();
    do
    {
        if (DWT_GetTimeline_s() - startTime > 12)
        {
            BMI088LoadOfflineParams(bmi088);
            LOGERROR("[BMI088] Calibrate Failed! Use offline params");
            break;
        }

        DWT_Delay(0.005);
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;
        valid_samples = 0;
        gNormDiff = 0.0f;
        gyroDiff[0] = 0.0f;
        gyroDiff[1] = 0.0f;
        gyroDiff[2] = 0.0f;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            status = BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            if (status != HAL_OK)
            {
                BMI088LogReadFailure("calibrate acc", status);
                break;
            }
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);

            status = BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (status != HAL_OK)
            {
                BMI088LogReadFailure("calibrate gyro", status);
                break;
            }
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }
            else
            {
                LOGWARNING("[bmi088] calibration gyro chip id mismatch: 0x%x", (unsigned int)buf[0]);
                break;
            }

            if (valid_samples == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
                }
            }

            bmi088->gNorm += gNormTemp;
            valid_samples++;

            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
            {
                LOGWARNING("[bmi088] calibration was interrupted\n");
                break;
            }

            DWT_Delay(0.0005);
        }

        if (valid_samples == 0)
        {
            BMI088LoadOfflineParams(bmi088);
            LOGERROR("[BMI088] Calibrate Failed: no valid SPI sample, use offline params");
            break;
        }

        bmi088->gNorm /= (float)valid_samples;
        for (uint8_t i = 0; i < 3; ++i)
            bmi088->GyroOffset[i] /= (float)valid_samples;

        status = BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        if (status == HAL_OK)
        {
            bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
            if (bmi088_raw_temp > 1023)
                bmi088_raw_temp -= 2048;
            bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        }
        else
        {
            BMI088LogReadFailure("calibrate temp", status);
            bmi088->TempWhenCali = 40;
        }

        caliCount++;
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    bmi088->AccelScale = 9.81f / bmi088->gNorm;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t init_error = BMI088_NO_ERROR;
    HAL_StatusTypeDef status;

    // check commiunication
    status = BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 acc chip id, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);
    status = BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 acc chip id, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);
    // accel software reset
    status = BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not reset bmi088 acc, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    DWT_Delay(0.08);
    // check commiunication is normal after reset
    status = BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 acc chip id after reset, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);
    status = BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 acc chip id after reset, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        LOGERROR("[bmi088] Can not read bmi088 acc chip id");
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        status = BMI088_accel_write_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], BMI088_Accel_Init_Table[write_reg_num][1]);
        if (status != HAL_OK)
        {
            init_error |= BMI088_Accel_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Can not write acc reg 0x%x, status=%u",
                     (unsigned int)BMI088_Accel_Init_Table[write_reg_num][0],
                     (unsigned int)status);
            continue;
        }
        DWT_Delay(0.001);

        status = BMI088_accel_read_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], res);
        if (status != HAL_OK)
        {
            init_error |= BMI088_Accel_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Can not read acc reg 0x%x, status=%u",
                     (unsigned int)BMI088_Accel_Init_Table[write_reg_num][0],
                     (unsigned int)status);
            continue;
        }
        DWT_Delay(0.001);

        if (res != BMI088_Accel_Init_Table[write_reg_num][1])
        {
            init_error |= BMI088_Accel_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Acc reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)BMI088_Accel_Init_Table[write_reg_num][0],
                     (unsigned int)res,
                     (unsigned int)BMI088_Accel_Init_Table[write_reg_num][1]);
        }
    }
    return init_error;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t init_error = BMI088_NO_ERROR;
    HAL_StatusTypeDef status;

    // check commiunication
    status = BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 gyro chip id, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);
    status = BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 gyro chip id, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);

    // reset the gyro sensor
    status = BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not reset bmi088 gyro, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    DWT_Delay(0.08);
    // check commiunication is normal after reset
    status = BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 gyro chip id after reset, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);
    status = BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    if (status != HAL_OK)
    {
        LOGERROR("[bmi088] Can not read bmi088 gyro chip id after reset, status=%u", (unsigned int)status);
        return BMI088_NO_SENSOR;
    }
    DWT_Delay(0.001);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        LOGERROR("[bmi088] Can not read bmi088 gyro chip id");
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        status = BMI088_gyro_write_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], BMI088_Gyro_Init_Table[write_reg_num][1]);
        if (status != HAL_OK)
        {
            init_error |= BMI088_Gyro_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Can not write gyro reg 0x%x, status=%u",
                     (unsigned int)BMI088_Gyro_Init_Table[write_reg_num][0],
                     (unsigned int)status);
            continue;
        }
        DWT_Delay(0.001);

        status = BMI088_gyro_read_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], res);
        if (status != HAL_OK)
        {
            init_error |= BMI088_Gyro_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Can not read gyro reg 0x%x, status=%u",
                     (unsigned int)BMI088_Gyro_Init_Table[write_reg_num][0],
                     (unsigned int)status);
            continue;
        }
        DWT_Delay(0.001);

        if (res != BMI088_Gyro_Init_Table[write_reg_num][1])
        {
            init_error |= BMI088_Gyro_Init_Table[write_reg_num][2];
            LOGERROR("[bmi088] Gyro reg 0x%x verify failed: read=0x%x, expect=0x%x",
                     (unsigned int)BMI088_Gyro_Init_Table[write_reg_num][0],
                     (unsigned int)res,
                     (unsigned int)BMI088_Gyro_Init_Table[write_reg_num][1]);
        }
    }

    return init_error;
}

void BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;
    HAL_StatusTypeDef status;

    status = BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    if (status != HAL_OK)
    {
        BMI088LogReadFailure("read acc", status);
        return;
    }

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;

    status = BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (status != HAL_OK)
    {
        BMI088LogReadFailure("read gyro", status);
        return;
    }
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    else
    {
        BMI088LogReadFailure("read gyro chip id", HAL_ERROR);
        return;
    }

    status = BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    if (status != HAL_OK)
    {
        BMI088LogReadFailure("read temp", status);
        return;
    }

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

/**
 * @brief 注册当前主流程使用的BMI088 SPI从设备
 *
 * 旧版INS主流程仍然调用BMI088Init(&hspi2, 1),因此这里在保持原接口不变的前提下,
 * 将加速度计和陀螺仪分别注册为bsp/spi实例。初始化阶段FreeRTOS尚未运行,
 * 所以默认使用阻塞模式;进入任务后仍会通过bsp/spi的总线互斥保护SPI2。
 */
static void BMI088RegisterSPIBus(SPI_HandleTypeDef *bmi088_SPI)
{
    SPI_Init_Config_s acc_spi_config;
    SPI_Init_Config_s gyro_spi_config;

    if (bmi088_acc_spi != NULL && bmi088_gyro_spi != NULL)
    {
        return;
    }

    memset(&acc_spi_config, 0, sizeof(acc_spi_config));
    acc_spi_config.spi_handle = bmi088_SPI;
    acc_spi_config.GPIOx = CS2_ACCEL_GPIO_Port;
    acc_spi_config.cs_pin = CS2_ACCEL_Pin;
    acc_spi_config.spi_work_mode = SPI_BLOCK_MODE;
    bmi088_acc_spi = SPIRegister(&acc_spi_config);

    memset(&gyro_spi_config, 0, sizeof(gyro_spi_config));
    gyro_spi_config.spi_handle = bmi088_SPI;
    gyro_spi_config.GPIOx = CS2_GYRO_GPIO_Port;
    gyro_spi_config.cs_pin = CS2_GYRO_Pin;
    gyro_spi_config.spi_work_mode = SPI_BLOCK_MODE;
    bmi088_gyro_spi = SPIRegister(&gyro_spi_config);
}

static HAL_StatusTypeDef BMI088AccelWriteSingleReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};

    return SPITransmit(bmi088_acc_spi, tx, sizeof(tx));
}

static HAL_StatusTypeDef BMI088AccelReadSingleReg(uint8_t reg, uint8_t *return_data)
{
    uint8_t tx[3] = {(uint8_t)(reg | 0x80U), 0x55U, 0x55U};
    uint8_t rx[3] = {0};
    HAL_StatusTypeDef status;

    status = SPITransRecv(bmi088_acc_spi, rx, tx, sizeof(tx));
    if (status == HAL_OK && return_data != NULL)
    {
        *return_data = rx[2];
    }

    return status;
}

static HAL_StatusTypeDef BMI088AccelReadMultiReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    HAL_StatusTypeDef status;

    if (buf == NULL || len > 6U)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(reg | 0x80U);
    tx[1] = 0x55U; // BMI088加速度计SPI读取需要额外dummy byte
    memset(&tx[2], 0x55, len);

    status = SPITransRecv(bmi088_acc_spi, rx, tx, (uint16_t)(len + 2U));
    if (status == HAL_OK)
    {
        memcpy(buf, &rx[2], len);
    }

    return status;
}

static HAL_StatusTypeDef BMI088GyroWriteSingleReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};

    return SPITransmit(bmi088_gyro_spi, tx, sizeof(tx));
}

static HAL_StatusTypeDef BMI088GyroReadSingleReg(uint8_t reg, uint8_t *return_data)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0x55U};
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef status;

    status = SPITransRecv(bmi088_gyro_spi, rx, tx, sizeof(tx));
    if (status == HAL_OK && return_data != NULL)
    {
        *return_data = rx[1];
    }

    return status;
}

static HAL_StatusTypeDef BMI088GyroReadMultiReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};
    HAL_StatusTypeDef status;

    if (buf == NULL || len > 8U)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(reg | 0x80U);
    memset(&tx[1], 0x55, len);

    status = SPITransRecv(bmi088_gyro_spi, rx, tx, (uint16_t)(len + 1U));
    if (status == HAL_OK)
    {
        memcpy(buf, &rx[1], len);
    }

    return status;
}
#elif defined(BMI088_USE_IIC)

#endif
