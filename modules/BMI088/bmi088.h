#ifndef __BMI088_H__
#define __BMI088_H__

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include <stdint.h>

/**
 * @brief BMI088读取方式
 *
 * 周期模式由上层任务主动调用BMI088Acquire()读取一次完整IMU数据。
 * 触发模式只把EXTI事件记录下来,真正的SPI读取仍然放在任务上下文中完成,
 * 避免在中断里访问SPI或调用会阻塞的RTOS接口。
 */
typedef enum
{
    BMI088_BLOCK_PERIODIC_MODE = 0,
    BMI088_BLOCK_TRIGGER_MODE,
} BMI088_Work_Mode_e;

/**
 * @brief BMI088标定方式
 */
typedef enum
{
    BMI088_CALIBRATE_ONLINE_MODE = 0, // 上电初始化时在线标定
    BMI088_LOAD_PRE_CALI_MODE, // 直接加载预设离线标定参数
} BMI088_Calibrate_Mode_e;

/**
 * @brief BMI088一次采样结果
 */
typedef struct
{
    float gyro[3]; // 陀螺仪角速度,单位rad/s
    float acc[3]; // 加速度,单位m/s^2
    float temperature; // 温度,单位摄氏度
} BMI088_Data_t;

/**
 * @brief BMI088实例
 *
 * 达妙主控板上BMI088的加速度计和陀螺仪共用SPI2,但拥有两根片选线。
 * 这里保留两个SPIInstance,由bsp/spi负责总线互斥与片选控制。
 */
typedef struct
{
    BMI088_Work_Mode_e work_mode;
    BMI088_Calibrate_Mode_e cali_mode;

    SPIInstance* spi_gyro; // 陀螺仪SPI从设备实例
    SPIInstance* spi_acc; // 加速度计SPI从设备实例

    GPIOInstance* gyro_int; // 可选:陀螺仪DRDY EXTI实例
    GPIOInstance* acc_int; // 可选:加速度计DRDY EXTI实例

    float gyro[3]; // 最近一次有效陀螺仪数据,单位rad/s
    float acc[3]; // 最近一次有效加速度数据,单位m/s^2
    float temperature; // 最近一次有效温度,单位摄氏度
    uint16_t temp_read_count; // 温度读取降频计数,避免1kHz姿态任务每次都访问温度寄存器

    float gyro_offset[3]; // 陀螺仪零偏,单位rad/s
    float gNorm; // 标定得到的重力加速度模长
    float acc_coef; // 加速度计原始值到m/s^2的换算系数,包含gNorm修正
    float temp_when_cali; // 标定时温度,用于后续扩展温漂补偿

    float BMI088_ACCEL_SEN; // 当前加速度计量程对应灵敏度
    float BMI088_GYRO_SEN; // 当前陀螺仪量程对应灵敏度

    struct
    {
        uint8_t gyro : 1; // 1:有新的陀螺仪DRDY事件
        uint8_t acc : 1; // 1:有新的加速度计DRDY事件
        uint8_t temp : 1; // 预留:温度更新事件
        uint8_t gyro_overrun : 1;
        uint8_t acc_overrun : 1;
        uint8_t temp_overrun : 1;
        uint8_t imu_ready : 1; // 1:至少有一个IMU事件等待任务处理
    } update_flag;

    volatile uint32_t acc_drdy_count; // EXTI侧加速度计DRDY计数
    volatile uint32_t gyro_drdy_count; // EXTI侧陀螺仪DRDY计数
    uint32_t acc_read_count; // 任务侧已处理到的加速度计计数
    uint32_t gyro_read_count; // 任务侧已处理到的陀螺仪计数
} BMI088Instance;

/**
 * @brief BMI088初始化配置
 *
 * 温控PID和加热PWM属于板级IMU策略,不放在BMI088芯片驱动中。
 */
typedef struct
{
    BMI088_Work_Mode_e work_mode;
    BMI088_Calibrate_Mode_e cali_mode;
    SPI_Init_Config_s spi_gyro_config;
    SPI_Init_Config_s spi_acc_config;
    GPIO_Init_Config_s gyro_int_config;
    GPIO_Init_Config_s acc_int_config;
} BMI088_Init_Config_s;

/**
 * @brief 注册并初始化BMI088
 *
 * @param config BMI088硬件连接和工作模式配置
 * @return 初始化成功返回实例指针,失败返回NULL
 */
BMI088Instance* BMI088Register(BMI088_Init_Config_s* config);


/**
 * @brief 读取一次BMI088数据
 *
 * @param bmi088 BMI088实例
 * @param data_store 用于保存本次数据的缓冲区
 * @return 1表示本次至少获得一组新的有效IMU数据,0表示读取失败或触发模式下暂无新事件
 */
uint8_t BMI088Acquire(BMI088Instance* bmi088, BMI088_Data_t* data_store);


/**
 * @brief 标定BMI088
 *
 * @note 该函数会执行阻塞式采样,应只在系统初始化阶段调用。
 *
 * @param bmi088 BMI088实例
 */
void BMI088CalibrateIMU(BMI088Instance* bmi088);

#endif // __BMI088_H__
