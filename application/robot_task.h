/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "lcd.h"

#include "bsp_init.h"
#include "bsp_log.h"

osThreadId_t insTaskHandle;
osThreadId_t robotTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t daemonTaskHandle;
osThreadId_t uiTaskHandle;

void StartINSTASK(void *argument);
void StartMOTORTASK(void *argument);
void StartDAEMONTASK(void *argument);
void StartROBOTTASK(void *argument);
void StartUITASK(void *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit(void)
{
    BSPTaskInit(); // 创建BSP运行期资源和后台任务,application层不直接关心具体BSP任务入口
    if (LCDTaskInit() != HAL_OK)
    {
        LOGWARNING("[freeRTOS] LCD Task Init failed, async LCD draw is disabled");
    }

    const osThreadAttr_t insTaskAttr = {
        .name = "instask",
        .stack_size = 1024 * 4,
        .priority = osPriorityAboveNormal,
    };
    insTaskHandle = osThreadNew(StartINSTASK, NULL, &insTaskAttr); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    const osThreadAttr_t motorTaskAttr = {
        .name = "motortask",
        .stack_size = 256 * 4,
        .priority = osPriorityNormal,
    };
    motorTaskHandle = osThreadNew(StartMOTORTASK, NULL, &motorTaskAttr);

    const osThreadAttr_t daemonTaskAttr = {
        .name = "daemontask",
        .stack_size = 128 * 4,
        .priority = osPriorityNormal,
    };
    daemonTaskHandle = osThreadNew(StartDAEMONTASK, NULL, &daemonTaskAttr);

    const osThreadAttr_t robotTaskAttr = {
        .name = "robottask",
        .stack_size = 1024 * 4,
        .priority = osPriorityNormal,
    };
    robotTaskHandle = osThreadNew(StartROBOTTASK, NULL, &robotTaskAttr);

    const osThreadAttr_t uiTaskAttr = {
        .name = "uitask",
        .stack_size = 512 * 4,
        .priority = osPriorityNormal,
    };
    uiTaskHandle = osThreadNew(StartUITASK, NULL, &uiTaskAttr);

    HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%d] us", (int)(ins_dt * 1000.0f));
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%d] us", (int)(motor_dt * 1000.0f));
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void *argument)
{
    static float daemon_dt;
    static float daemon_start;
    uint32_t wake_tick;

    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    wake_tick = osKernelGetTickCount();
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > DAEMON_TASK_PERIOD_MS)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%d] us", (int)(daemon_dt * 1000.0f));

        wake_tick += DAEMON_TASK_PERIOD_MS;
        if ((int32_t)(wake_tick - osKernelGetTickCount()) <= 0)
        {
            /*
             * 如果本任务因为高优先级任务或系统繁忙而错过了计划唤醒点,
             * 则从当前tick重新规划下一周期,避免连续补跑多轮DaemonTask/BuzzerTask。
             */
            wake_tick = osKernelGetTickCount() + DAEMON_TASK_PERIOD_MS;
        }
        osDelayUntil(wake_tick);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%d] us", (int)(robot_dt * 1000.0f));
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartUITASK(void *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    if (MyUIInit() != 0U)
    {
        LOGINFO("[freeRTOS] UI init done, referee communication established");
    }
    else
    {
        LOGWARNING("[freeRTOS] UI init pending, waiting referee data in task");
    }
    for (;;)
    {
        // UITask内部会泵裁判系统发送队列,并按协议频率限制逐帧启动DMA发送。
        UITask();
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
