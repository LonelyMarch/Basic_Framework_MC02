/**
 * @file bsp_adc.h
 * @brief ADC DMA循环采样的最小BSP封装。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "adc.h"
#include <stdint.h>

#define BSP_ADC_DEVICE_CNT       1U
#define BSP_ADC_CHANNEL_CNT_MAX 16U

#ifndef BSP_ADC_USE_DMA_CACHE_MAINTENANCE
#define BSP_ADC_USE_DMA_CACHE_MAINTENANCE 1U
#endif

#ifndef BSP_ADC_DEFAULT_VREF
#define BSP_ADC_DEFAULT_VREF 3.3f
#endif

typedef struct
{
    ADC_HandleTypeDef *adc_handle; // ADC硬件句柄
    uint8_t channel_count;         // 扫描通道数量,需要和CubeMX中的NbrOfConversion一致
    float vref;                    // ADC参考电压,传入0时使用BSP_ADC_DEFAULT_VREF
    void *id;                      // 上层模块指针,按需使用
} ADC_Init_Config_s;

typedef struct
{
    ADC_HandleTypeDef *adc_handle; // ADC硬件句柄
    uint16_t *dma_buffer;          // ADC DMA循环缓冲区,位于RAM_D2的.dma_buffer段
    uint8_t channel_count;         // 扫描通道数量
    float vref;                    // ADC参考电压
    volatile uint8_t started;      // DMA循环采样是否已经启动
    volatile uint32_t update_count;// DMA一轮转换完成次数
    volatile uint32_t error_count; // ADC错误回调触发次数
    volatile uint32_t last_error;  // 最近一次HAL ADC错误码
    void *id;                      // 上层模块指针,按需使用
} ADCInstance;

/**
 * @brief 注册一个ADC实例
 *
 * @param config ADC初始化配置
 * @return ADCInstance* 成功返回实例指针,失败返回NULL
 */
ADCInstance *ADCRegister(ADC_Init_Config_s *config);

/**
 * @brief 启动ADC DMA循环采样
 */
HAL_StatusTypeDef ADCStart(ADCInstance *instance);

/**
 * @brief 停止ADC DMA循环采样
 */
HAL_StatusTypeDef ADCStop(ADCInstance *instance);

/**
 * @brief 按扫描序号读取ADC原始值
 *
 * @param channel_index 扫描序号,从0开始,对应CubeMX Regular Rank顺序
 */
uint16_t ADCGetRaw(ADCInstance *instance, uint8_t channel_index);

/**
 * @brief 按扫描序号读取ADC电压值
 */
float ADCGetVoltage(ADCInstance *instance, uint8_t channel_index);

uint32_t ADCGetUpdateCount(ADCInstance *instance);
uint32_t ADCGetErrorCount(ADCInstance *instance);
uint32_t ADCGetLastError(ADCInstance *instance);

#endif // BSP_ADC_H
