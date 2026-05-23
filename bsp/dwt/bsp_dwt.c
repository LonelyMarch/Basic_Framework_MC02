/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief DWT高精度时间轴、短延时和CYCCNT 64位扩展实现
 */

#include "bsp_dwt.h"

#define DWT_DELAY_MAX_TICK (UINT32_MAX / 2U)

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

/**
 * @brief 读取扩展后的64位DWT计数值
 *
 * @attention 此函数假设两次调用之间的时间间隔不超过一次CYCCNT溢出
 */
static uint64_t DWT_GetCYCCNT64(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
    {
        CYCCNT_RountCount++;
    }

    CYCCNT_LAST = cnt_now;
    CYCCNT64 = ((uint64_t)CYCCNT_RountCount << 32) | cnt_now;

    __set_PRIMASK(primask);

    return CYCCNT64;
}

/**
 * @brief 根据64位DWT计数值计算系统时间
 */
static void DWT_CalcSysTime(DWT_Time_t *sys_time)
{
    uint64_t cyccnt64 = DWT_GetCYCCNT64();
    uint64_t cnt_temp1, cnt_temp2, cnt_temp3;

    cnt_temp1 = cyccnt64 / CPU_FREQ_Hz;
    cnt_temp2 = cyccnt64 - cnt_temp1 * CPU_FREQ_Hz;
    sys_time->s = (uint32_t)cnt_temp1;
    sys_time->ms = (uint16_t)(cnt_temp2 / CPU_FREQ_Hz_ms);
    cnt_temp3 = cnt_temp2 - (uint64_t)sys_time->ms * CPU_FREQ_Hz_ms;
    sys_time->us = (uint16_t)(cnt_temp3 / CPU_FREQ_Hz_us);
}

/**
 * @brief 获取一份当前系统时间快照
 */
static DWT_Time_t DWT_GetSysTimeSnapshot(void)
{
    DWT_Time_t sys_time;

    DWT_CalcSysTime(&sys_time);

    return sys_time;
}

/**
 * @brief 清空DWT软件时间状态
 */
static void DWT_ResetSysTime(void)
{
    CYCCNT_RountCount = 0;
    CYCCNT_LAST = 0;
    CYCCNT64 = 0;
}

/**
 * @brief STM32H7部分内核调试组件带锁,写入解锁值后再访问DWT更稳妥
 */
static void DWT_Unlock(void)
{
    if (DWT->LSR & 1UL)
    {
        DWT->LAR = 0xC5ACCE55;
    }
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_Unlock();

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000U;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    DWT_ResetSysTime();

    (void)DWT_GetCYCCNT64();
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = (uint32_t)DWT_GetCYCCNT64();
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = (uint32_t)DWT_GetCYCCNT64();
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    (void)DWT_GetCYCCNT64();
}

float DWT_GetTimeline_s(void)
{
    DWT_Time_t sys_time = DWT_GetSysTimeSnapshot();

    float DWT_Timelinef32 = sys_time.s + sys_time.ms * 0.001f + sys_time.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_Time_t sys_time = DWT_GetSysTimeSnapshot();

    float DWT_Timelinef32 = (float)sys_time.s * 1000.0f + sys_time.ms + sys_time.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_Time_t sys_time = DWT_GetSysTimeSnapshot();

    uint64_t DWT_Timelinef32 = (uint64_t)sys_time.s * 1000000 + (uint64_t)sys_time.ms * 1000 + sys_time.us;

    return DWT_Timelinef32;
}

void DWT_Delay(float Delay)
{
    if (Delay <= 0.0f)
    {
        return;
    }

    float delay_tick_f = Delay * (float)CPU_FREQ_Hz;
    if (delay_tick_f >= (float)DWT_DELAY_MAX_TICK)
    {
        LOGERROR("[DWT] delay too long, delay = [%d] ms", (int)(Delay * 1000.0f));
        delay_tick_f = (float)DWT_DELAY_MAX_TICK;
    }

    uint32_t tickstart = DWT->CYCCNT;
    uint32_t delay_tick = (uint32_t)delay_tick_f;

    while ((uint32_t)(DWT->CYCCNT - tickstart) < delay_tick)
    {
    }
}
