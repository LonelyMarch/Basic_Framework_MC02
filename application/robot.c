/**
 * @file robot.c
 * @brief APP 层最小启动壳，仅负责建立框架运行所需的初始化边界。
 *
 * @note 原有底盘、云台、发射和命令解析业务已经全部移除。后续迁移新项目时，
 *       应在本文件中重新组织模块实例初始化，并在独立的 APP 源文件中实现业务逻辑。
 */

#include "robot.h"

#include "bsp_init.h"
#include "chassis.h"
#include "gimbal.h"
#include "main.h"
#include "message_center.h"
#include "robot_cmd.h"
#include "robot_task.h"
#include "shoot.h"

/**
 * @brief 初始化调度器启动前必须存在的框架资源。
 *
 * @details 当前只初始化 BSP 基础资源和消息中心，不注册任何机器人业务模块或硬件实例。
 *          后续迁移新 APP 时，应把模块实例注册放在 MessageCenterInit() 与
 *          MessageCenterLockRegistration() 之间，确保 topic 注册完成后再锁定消息中心。
 *
 * @warning 本函数在 FreeRTOS 调度器启动前执行，禁止调用依赖任务调度的延时接口。
 */
void RobotInit(void)
{
    // 初始化期间关闭全局中断，避免尚未完成注册的外设提前进入回调路径。
    __disable_irq();

    BSPInit(); // 初始化 DWT、日志、BSP 服务和片内 Flash 等基础资源。
    MessageCenterInit(); // 建立消息中心注册表，为后续迁入的新 APP 预留通信基础设施。

    // 以下初始化函数当前均为空壳；迁移业务时在对应 APP 内注册模块实例和消息 topic。
    RobotCMDInit();
    GimbalInit();
    ChassisInit();
    ShootInit();

    // 所有 APP 初始化完成后锁定注册表，运行期只允许消息发布和读取。
    MessageCenterLockRegistration();

    // 所有调度器启动前资源均已就绪，恢复全局中断响应。
    __enable_irq();
}

/**
 * @brief 创建维持 BSP 异步机制运行所需的 RTOS 资源和后台任务。
 *
 * @details BSPTaskInit() 会创建 CAN 处理、BSP 服务和异步 Flash 等框架后台任务；
 *          RobotTaskInit() 会创建无具体业务的电机、daemon 和 APP 控制任务。
 *          后续迁移项目时，可在现有任务中接入业务，或按明确的实时性需求增加任务。
 */
void RobotOSTaskInit(void)
{
    BSPTaskInit(); // 必须在 osKernelInitialize() 之后、osKernelStart() 之前调用。
    RobotTaskInit(); // 创建电机管理、daemon 和 APP 控制任务。
}

/**
 * @brief 执行一次 APP 层控制调度。
 *
 * @details 调用顺序体现数据依赖：命令入口先更新目标，随后各执行 APP 消费目标。
 *          当前各入口均为空实现，不会产生控制输出。
 */
void RobotTask(void)
{
    RobotCMDTask(); // 读取输入并生成控制目标；当前为空实现。
    GimbalTask(); // 消费云台目标并设置执行器；当前为空实现。
    ChassisTask(); // 消费底盘目标并设置执行器；当前为空实现。
    ShootTask(); // 消费发射目标并设置执行器；当前为空实现。
}
