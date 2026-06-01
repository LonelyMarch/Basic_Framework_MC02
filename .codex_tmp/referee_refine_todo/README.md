# modules/referee 临时 @TODO

本文档记录本轮 `modules/referee` 精修项。当前工程已经将 USART 接收解析从中断回调迁移到 `BSPServiceTask` 任务上下文，因此本轮重点放在裁判系统模块自身的解析鲁棒性、任务间数据一致性和 UI 发送安全性上。

## 1. UI 任务仍会覆盖真实机器人状态

问题：`UITask()` 每周期调用 `RobotModeTest()`，该测试函数会主动改写 `chassis_mode`、`gimbal_mode`、`shoot_mode` 等 UI 输入数据。真实运行时，UI 应显示 application 层传入的状态，而不是测试函数伪造的状态。

方案：增加 `REFEREE_UI_TEST_ENABLE` 宏，默认关闭测试逻辑。测试函数和调用点只在该宏开启时参与编译，避免影响正式流程。

## 2. 裁判系统接收解析抗错帧能力弱

问题：旧解析逻辑只检查接收 buffer 的第 0 字节是否为 `0xA5`。如果前面混入噪声、半帧或 CRC 错帧，后面即使存在完整合法帧，也可能被整包丢弃。

方案：把 `JudgeReadData()` 改成循环扫描解析。遇到 `0xA5` 后依次检查 CRC8、长度、CRC16；合法则解析并跳到下一帧，非法则滑动 1 字节继续寻找下一帧。

## 3. 裁判数据在任务间共享但缺少快照接口

问题：USART 解析任务会更新 `referee_info`，UI 任务和 application 层可能读取同一份数据。多字节字段和 `float` 字段在任务切换点上理论上可能被读到半更新状态。

方案：新增 `RefereeGet(referee_info_t *snapshot)` 快照接口。解析写入和快照读取都使用很短的关中断临界区，保证任务间复制的一致性。保留 `RefereeInit()` 返回内部指针的旧接口，兼容当前调用方式，但新代码优先使用快照接口。

## 4. `rm_referee.h` 中运行期结构体不应整体 pack

问题：协议帧结构体需要 `#pragma pack(1)`，但运行期状态结构体 `referee_info_t` 不应该被额外强制 1 字节对齐。否则对齐访问和可维护性都较差。

方案：移除 `rm_referee.h` 外层 `#pragma pack(1)`。`referee_protocol.h` 中的线上协议结构体继续保持 pack，保证协议字节布局不变。

## 5. UI 字符绘制存在格式化溢出风险

问题：`UICharDraw()` 使用 `vsprintf()` 写入 30 字节显示区，格式化字符串过长时会越界。

方案：改用 `vsnprintf()`，并把显示长度限制到 `show_Data` 的实际大小。

## 6. UI 图形刷新没有校验图形数量

问题：裁判系统图形刷新协议只支持一次发送 1、2、5、7 个图形。旧代码传入其他数量时不会报错，可能发送未正确设置 `data_cmd_id` 的帧。

方案：`UIGraphRefresh()` 入口检查 `cnt`，非法数量直接记录错误并返回。

## 7. UI 构帧静态缓存不适合未来多任务调用

问题：`UIDelete()`、`UICharRefresh()` 和 `UIGraphRefresh()` 使用静态发送结构体或静态 buffer。当前只有 UI 任务调用时风险较低，但未来多任务调用会互相覆盖。

方案：本轮改为局部构帧变量，降低共享状态。仍建议所有 UI 绘制接口只在 UI 任务中调用，后续如果需要多任务绘制，再统一改成 UI 发送队列。

## 8. `UIGraphRefresh()` 没有递增 UI 序号

问题：删除和字符刷新会递增 `UI_Seq`，但图形刷新没有递增，会造成多个图形包复用同一个序号。

方案：所有 UI 发送函数在 `RefereeSend()` 成功启动发送后递增 `UI_Seq`。

## 9. `RefereeSend()` 没有返回发送结果

问题：旧接口内部吞掉发送结果，并固定 `osDelay(115)`。上层无法知道发送是否成功。

方案：`RefereeSend()` 改为返回 `HAL_StatusTypeDef`。本轮仍保留裁判系统交互数据 10Hz 节流，但只在 RTOS 运行且发送成功时延时；后续若 UI 内容增多，再升级为 UI 发送队列。

## 10. 裁判系统离线回调会重复重启接收并刷屏日志

问题：daemon 离线后会周期调用回调，旧代码每次都重新启动 USART 接收并打印 warning。

方案：增加离线日志标志。离线期间只执行一次重启和一次日志；收到合法帧后清除标志。

## 11. `0x0003` 血量数据长度宏错误

问题：`ext_game_robot_HP_t` 包含红蓝双方机器人、前哨站和基地血量，共 16 个 `uint16_t`，协议数据长度应为 32 字节；当前 `LEN_game_robot_HP` 写成 2，会导致只更新前 2 字节。

方案：把 `LEN_game_robot_HP` 修正为 32，保证 `ID_game_robot_survivors` 能完整刷新血量数据。
