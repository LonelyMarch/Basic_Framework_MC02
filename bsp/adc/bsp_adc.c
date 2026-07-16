#include "bsp_adc.h"

#include "bsp_log.h"
#include <string.h>

#define BSP_ADC_DCACHE_LINE_SIZE 32U

#if defined(__GNUC__)
#define BSP_ADC_DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define BSP_ADC_DMA_BUFFER_ATTR
#endif

static ADCInstance adc_instances[BSP_ADC_DEVICE_CNT];
static uint8_t adc_instance_used[BSP_ADC_DEVICE_CNT];
static uint8_t adc_idx;

/*
 * STM32H7的DMA1/DMA2不能访问DTCM。ADC DMA循环缓冲区统一放在RAM_D2的
 * .dma_buffer段,避免把普通栈/全局变量直接交给DMA。
 */
static uint16_t adc_dma_buffer[BSP_ADC_DEVICE_CNT][BSP_ADC_CHANNEL_CNT_MAX] BSP_ADC_DMA_BUFFER_ATTR;

static void ADCAlignDCacheRange(uintptr_t address, uint32_t size, uintptr_t* aligned_address, int32_t* aligned_size)
{
    uintptr_t start = address & ~((uintptr_t)BSP_ADC_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (address + size + BSP_ADC_DCACHE_LINE_SIZE - 1U) & ~((uintptr_t)BSP_ADC_DCACHE_LINE_SIZE - 1U);

    *aligned_address = start;
    *aligned_size = (int32_t)(end - start);
}

static void ADCCleanInvalidateDCacheByAddr(const void* buffer, uint32_t len)
{
#if BSP_ADC_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    ADCAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

static void ADCInvalidateDCacheByAddr(const void* buffer, uint32_t len)
{
#if BSP_ADC_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    ADCAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

static ADCInstance* ADCFindInstance(ADC_HandleTypeDef* hadc)
{
    for (uint8_t i = 0; i < adc_idx; i++)
    {
        if (adc_instance_used[i] != 0U && adc_instances[i].adc_handle == hadc)
        {
            return &adc_instances[i];
        }
    }

    return NULL;
}

static float ADCGetResolutionMax(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL)
    {
        return 65535.0f;
    }

    switch (hadc->Init.Resolution)
    {
#ifdef ADC_RESOLUTION_16B
    case ADC_RESOLUTION_16B:
        return 65535.0f;
#endif
#ifdef ADC_RESOLUTION_14B
    case ADC_RESOLUTION_14B:
        return 16383.0f;
#endif
#ifdef ADC_RESOLUTION_12B
    case ADC_RESOLUTION_12B:
        return 4095.0f;
#endif
#ifdef ADC_RESOLUTION_10B
    case ADC_RESOLUTION_10B:
        return 1023.0f;
#endif
#ifdef ADC_RESOLUTION_8B
    case ADC_RESOLUTION_8B:
        return 255.0f;
#endif
    default:
        return 65535.0f;
    }
}

ADCInstance* ADCRegister(ADC_Init_Config_s* config)
{
    ADCInstance* instance;

    if (config == NULL || config->adc_handle == NULL)
    {
        LOGERROR("[bsp_adc] ADC register with invalid config");
        return NULL;
    }

    if (config->channel_count == 0U || config->channel_count > BSP_ADC_CHANNEL_CNT_MAX)
    {
        LOGERROR("[bsp_adc] invalid channel count:%u", (unsigned int)config->channel_count);
        return NULL;
    }

    if (ADCFindInstance(config->adc_handle) != NULL)
    {
        LOGERROR("[bsp_adc] ADC handle already registered");
        return NULL;
    }

    if (adc_idx >= BSP_ADC_DEVICE_CNT)
    {
        LOGERROR("[bsp_adc] ADC instance count exceeds limit");
        return NULL;
    }

    instance = &adc_instances[adc_idx];
    memset(instance, 0, sizeof(*instance));
    memset(adc_dma_buffer[adc_idx], 0, sizeof(adc_dma_buffer[adc_idx]));

    instance->adc_handle = config->adc_handle;
    instance->dma_buffer = adc_dma_buffer[adc_idx];
    instance->channel_count = config->channel_count;
    instance->vref = (config->vref > 0.0f) ? config->vref : BSP_ADC_DEFAULT_VREF;
    instance->id = config->id;

    adc_instance_used[adc_idx] = 1U;
    adc_idx++;
    return instance;
}

HAL_StatusTypeDef ADCStart(ADCInstance* instance)
{
    HAL_StatusTypeDef status;
    uint32_t buffer_size;

    if (instance == NULL || instance->adc_handle == NULL || instance->dma_buffer == NULL)
    {
        LOGERROR("[bsp_adc] ADC start with invalid instance");
        return HAL_ERROR;
    }

    buffer_size = (uint32_t)instance->channel_count * sizeof(uint16_t);
    memset(instance->dma_buffer, 0, buffer_size);
    ADCCleanInvalidateDCacheByAddr(instance->dma_buffer, buffer_size);

    status = HAL_ADC_Start_DMA(instance->adc_handle, (uint32_t*)instance->dma_buffer, instance->channel_count);
    if (status == HAL_OK)
    {
        instance->started = 1U;
    }
    else
    {
        LOGERROR("[bsp_adc] ADC start DMA failed, status:%u", (unsigned int)status);
    }

    return status;
}

HAL_StatusTypeDef ADCStop(ADCInstance* instance)
{
    HAL_StatusTypeDef status;

    if (instance == NULL || instance->adc_handle == NULL)
    {
        LOGERROR("[bsp_adc] ADC stop with invalid instance");
        return HAL_ERROR;
    }

    status = HAL_ADC_Stop_DMA(instance->adc_handle);
    if (status == HAL_OK)
    {
        instance->started = 0U;
    }
    else
    {
        LOGERROR("[bsp_adc] ADC stop DMA failed, status:%u", (unsigned int)status);
    }

    return status;
}

uint16_t ADCGetRaw(ADCInstance* instance, uint8_t channel_index)
{
    if (instance == NULL || instance->dma_buffer == NULL || channel_index >= instance->channel_count)
    {
        return 0U;
    }

    ADCInvalidateDCacheByAddr(instance->dma_buffer, (uint32_t)instance->channel_count * sizeof(uint16_t));
    return instance->dma_buffer[channel_index];
}

float ADCGetVoltage(ADCInstance* instance, uint8_t channel_index)
{
    uint16_t raw = ADCGetRaw(instance, channel_index);

    if (instance == NULL)
    {
        return 0.0f;
    }

    return ((float)raw * instance->vref) / ADCGetResolutionMax(instance->adc_handle);
}

uint32_t ADCGetUpdateCount(ADCInstance* instance)
{
    return instance == NULL ? 0U : instance->update_count;
}

uint32_t ADCGetErrorCount(ADCInstance* instance)
{
    return instance == NULL ? 0U : instance->error_count;
}

uint32_t ADCGetLastError(ADCInstance* instance)
{
    return instance == NULL ? 0U : instance->last_error;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    ADCInstance* instance = ADCFindInstance(hadc);

    if (instance != NULL)
    {
        instance->update_count++;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    ADCInstance* instance = ADCFindInstance(hadc);

    if (instance != NULL)
    {
        instance->error_count++;
        instance->last_error = HAL_ADC_GetError(hadc);
    }
}
