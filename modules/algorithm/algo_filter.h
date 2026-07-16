/**
 ******************************************************************************
 * @file    algo_filter.h
 * @brief   通用标量滤波器接口
 ******************************************************************************
 */
#ifndef ALGO_FILTER_H
#define ALGO_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float alpha; // 滤波系数,范围0~1;越大越相信新数据
    float value; // 当前滤波输出
    uint8_t inited; // 首次输入时直接使用输入值初始化输出
} ExpAverageFilter_t;

typedef struct
{
    float alpha; // 固定周期下预计算出的滤波系数,运行期不再做除法
    float value; // 当前滤波输出
    uint8_t inited; // 首次输入时直接使用输入值初始化输出
} LowPassFilter_t;

typedef struct
{
    float* buffer; // 调用者提供的窗口缓存
    uint16_t size; // 窗口长度
    uint16_t index; // 下次写入位置
    uint16_t count; // 已填入样本数
    float sum; // 窗口累加值
} WindowAverageFilter_t;


void ExpAverageFilterInit(ExpAverageFilter_t* filter, float alpha);


float ExpAverageFilterUpdate(ExpAverageFilter_t* filter, float input);


void ExpAverageFilterReset(ExpAverageFilter_t* filter, float value);


void LowPassFilterInit(LowPassFilter_t* filter, float rc, float dt);


void LowPassFilterInitByAlpha(LowPassFilter_t* filter, float alpha);


float LowPassFilterUpdate(LowPassFilter_t* filter, float input);


void LowPassFilterReset(LowPassFilter_t* filter, float value);


void WindowAverageFilterInit(WindowAverageFilter_t* filter, float* buffer, uint16_t size);


float WindowAverageFilterUpdate(WindowAverageFilter_t* filter, float input);


void WindowAverageFilterReset(WindowAverageFilter_t* filter, float value);

#ifdef __cplusplus
}
#endif

#endif // ALGO_FILTER_H
