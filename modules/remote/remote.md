# remote_control

`remote_control` 是 DJI DT7/DR16 遥控器 DBUS 协议解析模块。当前达妙 MC02 工程只使用一台遥控器,因此模块按单例设计。

模块不直接在 UART 中断中解析协议。UART 接收由 `bsp_usart` 完成,收到完整帧后由 `BSPServiceTask` 在任务上下文调用遥控器解析回调。

## 初始化

```c
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);
```

初始化遥控器模块,并把指定 UART 注册到 `bsp_usart`。当前工程在 `RobotCMDInit()` 中使用:

```c
RemoteControlInit(&huart5);
```

DBUS 固定帧长为 18 字节。注册成功后,模块会同时注册 `daemon`,用于检测遥控器离线。

当前 `DaemonTask()` 以 100Hz 运行,遥控器 `reload_count = 10`,约 100ms 没有收到合法 DBUS 帧即判定为离线。

## 接收流程

1. UART5 收到 DBUS 数据。
2. `bsp_usart` 在 UART 回调中缓存完整帧,并唤醒 `BSPServiceTask`。
3. `USARTProcess()` 在任务上下文调用 `RemoteControlRxCallback()`。
4. `RemoteControlRxCallback()` 先检查帧长和关键字段合法性。
5. 合法帧先重载 `daemon`,再解析到遥控器快照。
6. 非法帧直接丢弃,不会刷新在线状态。

合法性检查包括:

- 4 个摇杆通道和拨轮通道 raw 值必须在 `RC_CH_VALUE_MIN` 到 `RC_CH_VALUE_MAX` 之间。
- 左右拨杆必须为 `RC_SW_UP`、`RC_SW_MID` 或 `RC_SW_DOWN`。
- 鼠标左右键必须为 0 或 1。

## 数据获取

```c
uint8_t RemoteControlGet(RC_ctrl_t *rc_snapshot);
```

获取遥控器数据快照。`rc_snapshot` 必须指向长度为 2 的 `RC_ctrl_t` 数组:

- `rc_snapshot[RC_TEMP]`: 当前帧数据。
- `rc_snapshot[RC_LAST]`: 上一帧数据。

上层任务应通过 `RemoteControlGet()` 读取遥控器数据,不要直接访问模块内部静态变量。该接口内部使用短临界区复制快照,避免 `BSPServiceTask` 更新数据时上层读到半更新状态。

## 在线状态

```c
uint8_t RemoteControlIsOnline(void);
```

返回遥控器在线状态:

- 返回 `1`: 已初始化,收到过合法 DBUS 帧,且 `daemon` 未超时。
- 返回 `0`: 未初始化、尚未收到合法帧或 `daemon` 判定离线。

遥控器离线时,模块会清空内部快照并只打印一次离线日志。若离线回调和新合法帧刚好交错,代码会检测 `daemon` 是否已经被重新喂狗,避免误清空刚更新的数据。

## 数据结构

`RC_ctrl_t` 保存遥控器、鼠标和键盘数据:

```c
typedef struct
{
    struct
    {
        int16_t rocker_l_;
        int16_t rocker_l1;
        int16_t rocker_r_;
        int16_t rocker_r1;
        int16_t dial;
        uint8_t switch_left;
        uint8_t switch_right;
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3];
    uint8_t key_count[3][16];
} RC_ctrl_t;
```

### 摇杆与拨轮

| 字段 | 含义 |
| --- | --- |
| `rocker_r_` | 右摇杆水平通道 |
| `rocker_r1` | 右摇杆竖直通道 |
| `rocker_l_` | 左摇杆水平通道 |
| `rocker_l1` | 左摇杆竖直通道 |
| `dial` | 左侧拨轮 |

原始通道中值为 `RC_CH_VALUE_OFFSET = 1024`。解析后会减去中值偏置,若绝对值超过 `REMOTE_CONTROL_CHANNEL_VALID_LIMIT`,则认为该通道异常并清零。

### 开关

| 宏 | 数值 | 含义 |
| --- | --- | --- |
| `RC_SW_UP` | 1 | 开关向上 |
| `RC_SW_DOWN` | 2 | 开关向下 |
| `RC_SW_MID` | 3 | 开关居中 |

判断宏:

```c
switch_is_up(s);
switch_is_mid(s);
switch_is_down(s);
```

### 键盘

`key[]` 有 3 组状态:

- `RC_KEY_PRESS`: 普通按键状态。
- `RC_KEY_PRESS_WITH_CTRL`: Ctrl 组合键状态。
- `RC_KEY_PRESS_WITH_SHIFT`: Shift 组合键状态。

按键编号使用 `RC_KEY_W`、`RC_KEY_S`、`RC_KEY_D`、`RC_KEY_A`、`RC_KEY_Z` 等宏。

`key_count[group][key]` 在检测到按键上升沿时自增,适合用于模式切换。Ctrl 和 Shift 位只作为组合键修饰键使用,不会单独统计上升沿。

## 上层使用

`RobotCMDTask()` 每周期调用 `RemoteControlGet()` 获取快照,再根据遥控器状态生成底盘、云台、发射命令。

当前控制约定:

- 左侧开关上: 键鼠控制。
- 左侧开关中: 视觉控制。
- 左侧开关下: 遥控器控制。
- 右侧开关上: 从急停状态恢复。
- 右侧开关中: 底盘云台分离。
- 右侧开关下: 底盘跟随云台。
- 拨轮向下超过阈值: 进入急停。
- 拨轮向上: 摩擦轮/拨弹控制。

遥控器离线时,`RobotCMDTask()` 会进入急停,并禁止通过清零后的遥控器数据误恢复。
