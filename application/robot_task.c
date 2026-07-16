/**
 * @file robot_task.c
 * @brief 无具体业务的 APP 层周期任务框架。
 *
 * @details 当前创建三类基础任务：
 *          1. 电机管理任务：周期调用 modules/motor 的统一通信入口；
 *          2. daemon 任务：周期维护已注册模块的在线计数；
 *          3. APP 控制任务：按固定顺序调用 cmd/gimbal/chassis/shoot 空入口。
 *
 *          迁入新业务时应优先复用这些任务，不要为每个物理设备单独创建线程。
 */

#include "robot_task.h"

#include "cmsis_os2.h"
#include "daemon.h"
#include "main.h"
#include "motor_task.h"
#include "robot.h"

#define MOTOR_TASK_PERIOD_MS 1U  // 通信型电机统一管理周期，默认 1 kHz。
#define APP_TASK_PERIOD_MS 5U    // APP 状态机与目标更新周期，默认 200 Hz。

static osThreadId_t motor_task_handle;  // 通信型电机统一管理任务句柄。
static osThreadId_t daemon_task_handle; // 模块在线监测任务句柄。
static osThreadId_t app_task_handle;    // APP 控制调度任务句柄。

/**
 * @brief 电机统一管理任务入口。
 *
 * @param argument CMSIS-RTOS2 任务参数，当前未使用。
 */
static __attribute__((noreturn)) void StartMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // MotorControlTask() 会遍历已注册的通信型电机；当前无实例时不会产生控制报文。
        MotorControlTask();
        osDelay(MOTOR_TASK_PERIOD_MS);
    }
}

/**
 * @brief daemon 在线监测任务入口。
 *
 * @param argument CMSIS-RTOS2 任务参数，当前未使用。
 */
static __attribute__((noreturn)) void StartDaemonTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // DaemonTask() 只处理已注册实例；当前无实例时保持空转等待后续模块接入。
        DaemonTask();
        osDelay(DAEMON_TASK_PERIOD_MS);
    }
}

/**
 * @brief APP 控制任务入口。
 *
 * @param argument CMSIS-RTOS2 任务参数，当前未使用。
 */
static __attribute__((noreturn)) void StartAppTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // RobotTask() 固定按 cmd -> gimbal -> chassis -> shoot 的顺序执行一次控制调度。
        RobotTask();
        osDelay(APP_TASK_PERIOD_MS);
    }
}

/**
 * @brief 创建 APP 层基础周期任务。
 *
 * @details 电机任务优先级高于普通 APP 控制任务，保证通信输出节奏；daemon 和 APP
 *          控制任务使用普通优先级。任务创建失败属于框架无法继续运行的致命错误。
 */
void RobotTaskInit(void)
{
    const osThreadAttr_t motor_task_attr = {
        .name = "motor_task",
        .stack_size = 256U * 4U,
        .priority = osPriorityAboveNormal,
    };
    const osThreadAttr_t daemon_task_attr = {
        .name = "daemon_task",
        .stack_size = 256U * 4U,
        .priority = osPriorityNormal,
    };
    const osThreadAttr_t app_task_attr = {
        .name = "app_task",
        .stack_size = 512U * 4U,
        .priority = osPriorityNormal,
    };

    motor_task_handle = osThreadNew(StartMotorTask, NULL, &motor_task_attr);
    daemon_task_handle = osThreadNew(StartDaemonTask, NULL, &daemon_task_attr);
    app_task_handle = osThreadNew(StartAppTask, NULL, &app_task_attr);

    if ((motor_task_handle == NULL) || (daemon_task_handle == NULL) || (app_task_handle == NULL))
    {
        // 基础调度任务不完整时禁止继续启动，避免系统处于部分功能运行的不确定状态。
        Error_Handler();
    }
}
