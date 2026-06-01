# daemon

`daemon` 是 module 层的在线检测组件,用于判断遥控器、电机、裁判系统、视觉、超级电容、板间通信等模块是否长时间没有收到有效反馈。

模块收到合法反馈、完成一次有效通信或确认设备仍在线时,调用 `DaemonReload()` 喂狗。若计数递减到 0,`DaemonTask()` 会认为该模块离线,并调用注册时传入的离线回调函数。

## 初始化

使用前包含 `daemon.h`,并在模块实例中保存 `DaemonInstance *`:

```c
static void ModuleLostCallback(void *owner)
{
    ModuleInstance *module = (ModuleInstance *)owner;
    /* 设置离线标志、打印一次日志或执行轻量恢复 */
}

Daemon_Init_Config_s daemon_conf = {
    .reload_count = 10,
    .init_count = 0,
    .callback = ModuleLostCallback,
    .owner_id = module,
};

module->daemon = DaemonRegister(&daemon_conf);
if (module->daemon == NULL)
{
    return NULL;
}
```

`DaemonRegister()` 使用静态实例池,不会申请 heap。若配置为空或实例数量超过 `DAEMON_MX_CNT`,会返回 `NULL`。

## 计数规则

`DaemonTask()` 由 application 层的 daemon 任务周期调用。当前周期定义在 `daemon.h`:

```c
#define DAEMON_TASK_PERIOD_MS 10U
```

因此 daemon 默认以 `100Hz` 运行。

配置项含义:

- `reload_count`: 每次 `DaemonReload()` 后重装的计数值。
- `init_count`: 注册后的初始等待计数,用于给刚启动的模块预留上线时间。为 `0` 时默认等于最终的 `reload_count`。
- `DAEMON_DEFAULT_TIMEOUT_MS`: 未配置 `reload_count` 时的默认超时时间,当前为 `1000ms`。

超时时间约为:

```text
timeout_ms = reload_count * DAEMON_TASK_PERIOD_MS
```

例如:

- `reload_count = 2`: 约 `20ms` 未喂狗判定离线。
- `reload_count = 10`: 约 `100ms` 未喂狗判定离线。
- `reload_count = 30`: 约 `300ms` 未喂狗判定离线。

## 运行流程

`DaemonTask()` 每轮遍历所有已注册实例:

1. 若 `temp_count > 0`,递减一次计数。
2. 若 `temp_count == 0` 且存在离线回调,调用该回调。
3. 模块重新收到合法反馈后调用 `DaemonReload()`,计数恢复为 `reload_count`。

`temp_count--` 属于读-改-写操作。代码中只用极短临界区保护计数判断和递减,离线回调始终在临界区外执行。

当前设计保留“离线期间周期性调用 callback”的行为。如果模块不希望离线期间重复打印日志或重复执行恢复动作,应在自身模块中做限流或状态判断。

## 回调要求

离线 `callback` 运行在 daemon 任务上下文,不是中断上下文。

callback 适合做:

- 设置离线标志。
- 打印一次性告警日志。
- 清理轻量状态。
- 尝试一次轻量恢复操作。

callback 不建议做:

- 长时间阻塞。
- 忙等延时。
- 循环重试。
- 高频发送恢复指令。

如果模块需要周期性重连,推荐在自身任务或控制流程中调用 `DaemonIsOnline()` 判断在线状态,再按自己的节流周期执行恢复动作。

## @TODO

后续可以把 daemon 离线状态接入 `modules/alarm` 的分级报警机制。推荐做法不是让 `daemon.c` 直接依赖蜂鸣器模块,而是由 application 层或各模块离线 callback 根据模块重要程度调用 `AlarmSetStatus()`。

例如:

- 遥控器、电机等关键模块离线可映射到 `ALARM_LEVEL_HIGH`。
- 裁判系统、视觉等非急停类模块可映射到较低报警等级。
- 模块恢复在线后,由收到合法反馈的位置关闭对应报警。

这样可以保持 `daemon` 只负责在线检测,报警策略仍由 module/application 层决定。

## 常用接口

```c
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
void DaemonTask(void);
```

### DaemonRegister

注册一个 daemon 实例。

- 返回非 `NULL`: 注册成功。
- 返回 `NULL`: 配置为空或实例数量超限。

### DaemonReload

喂狗接口。模块收到合法数据或确认设备在线后调用。

传入 `NULL` 时函数直接返回。

### DaemonIsOnline

查询当前在线状态。

- 返回 `1`: 当前计数大于 0。
- 返回 `0`: 实例为空或当前计数已经为 0。

### DaemonTask

由 application 层 daemon 任务周期调用。当前工程中 `StartDAEMONTASK()` 使用 `osDelayUntil()` 按 `DAEMON_TASK_PERIOD_MS` 调度,并避免系统繁忙后连续补跑历史周期。
