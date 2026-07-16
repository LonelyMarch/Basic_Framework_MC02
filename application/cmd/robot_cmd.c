/**
 * @file robot_cmd.c
 * @brief 控制命令 APP 的无业务迁移骨架。
 */

#include "robot_cmd.h"

/**
 * @brief 初始化控制命令 APP。
 */
void RobotCMDInit(void)
{
    /*
     * 迁移入口：
     * 1. 注册遥控器、上位机或其他输入模块实例；
     * 2. 注册面向 gimbal/chassis/shoot 的消息发布者；
     * 3. 初始化只属于命令层的状态机。
     */
}

/**
 * @brief 执行一次控制命令更新。
 */
void RobotCMDTask(void)
{
    /*
     * 迁移入口：读取输入快照，执行模式切换和安全状态判断，然后发布控制目标。
     * 该层只生成目标，不应直接控制电机或其他执行器。
     */
}
