/**
 * @file robot.h
 * @brief APP 层最小启动入口。
 */

#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief 初始化调度器启动前必须存在的框架资源。
 *
 * @note 当前不初始化任何具体机器人业务；后续 APP 迁移以该函数作为模块注册入口。
 */
void RobotInit(void);

/**
 * @brief 创建 BSP 运行期资源和后台任务。
 *
 * @note 必须在 osKernelInitialize() 之后、osKernelStart() 之前调用。
 */
void RobotOSTaskInit(void);

/**
 * @brief 执行一次 APP 层控制调度。
 *
 * @note 由 APP 控制任务周期调用，迁移业务时保持命令生成先于执行 APP 更新。
 */
void RobotTask(void);

#endif // ROBOT_H
