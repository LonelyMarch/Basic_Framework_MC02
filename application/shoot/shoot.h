/**
 * @file shoot.h
 * @brief 发射机构 APP 空框架。
 */

#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief 初始化发射机构使用的消息、传感器和执行器实例。
 */
void ShootInit(void);

/**
 * @brief 执行一次发射机构状态机与目标更新。
 */
void ShootTask(void);

#endif // SHOOT_H
