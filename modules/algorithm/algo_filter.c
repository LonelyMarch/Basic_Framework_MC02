/**
 ******************************************************************************
 * @file    algo_filter.c
 * @brief   通用标量滤波器实现
 ******************************************************************************
 */
#include "algo_filter.h"
#include <string.h>

static float FilterClamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

void ExpAverageFilterInit(ExpAverageFilter_t *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    filter->alpha = FilterClamp01(alpha);
    filter->value = 0.0f;
    filter->inited = 0U;
}

float ExpAverageFilterUpdate(ExpAverageFilter_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    if (filter->inited == 0U)
    {
        filter->value = input;
        filter->inited = 1U;
        return filter->value;
    }

    filter->value += filter->alpha * (input - filter->value);
    return filter->value;
}

void ExpAverageFilterReset(ExpAverageFilter_t *filter, float value)
{
    if (filter == NULL)
    {
        return;
    }

    filter->value = value;
    filter->inited = 1U;
}

void LowPassFilterInit(LowPassFilter_t *filter, float rc, float dt)
{
    if (filter == NULL)
    {
        return;
    }

    /*
     * 固定周期任务中dt不变,alpha可以在初始化阶段预计算。
     * 这样运行期只做一次乘加,避免每个采样点都执行除法。
     */
    if (rc <= 0.0f || dt <= 0.0f)
    {
        filter->alpha = 1.0f;
    }
    else
    {
        filter->alpha = FilterClamp01(dt / (rc + dt));
    }
    filter->value = 0.0f;
    filter->inited = 0U;
}

void LowPassFilterInitByAlpha(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    filter->alpha = FilterClamp01(alpha);
    filter->value = 0.0f;
    filter->inited = 0U;
}

float LowPassFilterUpdate(LowPassFilter_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    if (filter->inited == 0U)
    {
        filter->value = input;
        filter->inited = 1U;
        return filter->value;
    }

    filter->value += filter->alpha * (input - filter->value);
    return filter->value;
}

void LowPassFilterReset(LowPassFilter_t *filter, float value)
{
    if (filter == NULL)
    {
        return;
    }

    filter->value = value;
    filter->inited = 1U;
}

void WindowAverageFilterInit(WindowAverageFilter_t *filter, float *buffer, uint16_t size)
{
    if (filter == NULL)
    {
        return;
    }

    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0U;
    filter->count = 0U;
    filter->sum = 0.0f;
    if (buffer != NULL && size != 0U)
    {
        memset(buffer, 0, sizeof(float) * size);
    }
}

float WindowAverageFilterUpdate(WindowAverageFilter_t *filter, float input)
{
    if (filter == NULL || filter->buffer == NULL || filter->size == 0U)
    {
        return input;
    }

    if (filter->count < filter->size)
    {
        filter->buffer[filter->index] = input;
        filter->sum += input;
        filter->count++;
    }
    else
    {
        filter->sum -= filter->buffer[filter->index];
        filter->buffer[filter->index] = input;
        filter->sum += input;
    }

    filter->index++;
    if (filter->index >= filter->size)
    {
        filter->index = 0U;
    }

    return filter->sum / (float)filter->count;
}

void WindowAverageFilterReset(WindowAverageFilter_t *filter, float value)
{
    if (filter == NULL || filter->buffer == NULL || filter->size == 0U)
    {
        return;
    }

    for (uint16_t i = 0; i < filter->size; ++i)
    {
        filter->buffer[i] = value;
    }
    filter->index = 0U;
    filter->count = filter->size;
    filter->sum = value * (float)filter->size;
}
