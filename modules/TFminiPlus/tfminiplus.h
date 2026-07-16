/**
 * @file tfminiplus.h
 * @brief TFmini Plus单点激光雷达模块封装。
 */
#ifndef TFMINIPLUS_H
#define TFMINIPLUS_H

#include "bsp_iic.h"
#include <stdint.h>

#define TFMINIPLUS_I2C_ADDR 0x10U       // TFmini Plus默认7位I2C地址,传给BSP时不需要左移
#define TFMINIPLUS_FRAME_LEN 11U        // 单次读取的测距数据帧长度
#define TFMINIPLUS_INSTANCE_CNT 1U      // 当前工程仅预留一个TFmini Plus实例
#define TFMINIPLUS_POWER_ON_DELAY_MS 500U // 传感器上电/复位后的稳定等待时间

/* TFmini Plus运行期实例 */
typedef struct
{
    IICInstance* iic; // 底层I2C实例
    uint8_t mode; // 模块返回的工作模式字段
    uint16_t distance; // 最近一次有效距离值,单位由TFmini Plus当前输出配置决定
    uint16_t strength; // 最近一次有效信号强度
    uint8_t raw_frame[TFMINIPLUS_FRAME_LEN]; // 最近一次原始测距数据帧
    uint8_t online; // 最近一次读取是否成功
    uint16_t error_count; // 连续/累计读取错误计数,用于调试观察
} TFMiniPlusInstance;

/* TFmini Plus初始化配置 */
typedef struct
{
    I2C_HandleTypeDef* hi2c; // CubeMX生成的I2C句柄
    IIC_Work_Mode_e work_mode; // I2C工作模式,未配置DMA时建议使用IIC_BLOCK_MODE或IIC_IT_MODE
} TFMiniPlus_Init_Config_s;

/**
 * @brief 注册TFmini Plus模块。
 *
 * @param config 初始化配置
 * @return TFMiniPlusInstance* 成功返回实例指针,失败返回NULL
 */
TFMiniPlusInstance* TFMiniPlusRegister(const TFMiniPlus_Init_Config_s* config);


/**
 * @brief 读取一次TFmini Plus测距数据。
 *
 * @param instance TFmini Plus实例
 * @return HAL_StatusTypeDef HAL_OK表示读取并校验成功
 */
HAL_StatusTypeDef TFMiniPlusRead(TFMiniPlusInstance* instance);


/**
 * @brief 获取最近一次有效距离值。
 *
 * @param instance TFmini Plus实例
 * @return uint16_t 距离值,若参数为空返回0
 */
uint16_t TFMiniPlusGetDistance(const TFMiniPlusInstance* instance);


/**
 * @brief 获取最近一次有效信号强度。
 *
 * @param instance TFmini Plus实例
 * @return uint16_t 信号强度,若参数为空返回0
 */
uint16_t TFMiniPlusGetStrength(const TFMiniPlusInstance* instance);


/**
 * @brief 判断最近一次读取是否成功。
 *
 * @param instance TFmini Plus实例
 * @return uint8_t 1表示在线/读取成功,0表示离线或读取失败
 */
uint8_t TFMiniPlusIsOnline(const TFMiniPlusInstance* instance);

#endif // TFMINIPLUS_H
