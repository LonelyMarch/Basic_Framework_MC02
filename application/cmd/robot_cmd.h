/**
 * @file robot_cmd.h
 * @brief 控制命令 APP 空框架。
 */

#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 初始化控制输入、消息发布者和输入设备实例。
 *
 * @note 当前为空实现，迁入项目时在此注册遥控器、上位机和命令输出 topic。
 */
void RobotCMDInit(void);

/**
 * @brief 执行一次控制命令更新。
 *
 * @note 当前为空实现，后续应在此完成输入快照读取、状态机更新和目标发布。
 */
void RobotCMDTask(void);

#endif // ROBOT_CMD_H
