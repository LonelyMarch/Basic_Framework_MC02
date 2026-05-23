# buzzer

蜂鸣器报警模块用于把不同来源的报警请求统一映射到一个 PWM 蜂鸣器输出。

## 工作方式

- `BuzzerInit()` 注册底层 PWM，当前使用 `TIM12 CH2`。
- `BuzzerRegister()` 注册一个报警实例，报警等级 `alarm_level` 同时作为内部数组下标。
- `AlarmSetStatus()` 设置报警实例的开关状态。
- `BuzzerTask()` 周期扫描所有报警实例，并播放当前处于 `ALARM_ON` 状态且优先级最高的报警。

报警优先级从高到低为：

```c
ALARM_LEVEL_HIGH = 0
ALARM_LEVEL_ABOVE_MEDIUM
ALARM_LEVEL_MEDIUM
ALARM_LEVEL_BELOW_MEDIUM
ALARM_LEVEL_LOW
```

同一报警等级只能注册一个实例。

## 音名与响度

`note` 用于选择蜂鸣器频率：

```c
BUZZER_NOTE_DO
BUZZER_NOTE_RE
BUZZER_NOTE_MI
BUZZER_NOTE_FA
BUZZER_NOTE_SO
BUZZER_NOTE_LA
BUZZER_NOTE_SI
```

`loudness` 表示 PWM 占空比，范围为 `0.0f ~ 1.0f`。注册时模块会自动限制到该范围。

## 使用示例

```c
static BuzzerInstance *robot_alarm;

void RobotAlarmInit(void)
{
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .note = BUZZER_NOTE_DO,
        .loudness = 0.4f,
    };

    robot_alarm = BuzzerRegister(&buzzer_config);
}

void RobotAlarmOn(void)
{
    AlarmSetStatus(robot_alarm, ALARM_ON);
}

void RobotAlarmOff(void)
{
    AlarmSetStatus(robot_alarm, ALARM_OFF);
}
```

## FreeRTOS 说明

当前工程中 `BuzzerInit()` 在 daemon 任务启动前调用，`BuzzerTask()` 由 daemon 任务以 `100Hz` 周期调用。

`AlarmSetStatus()` 可以被其他任务调用，模块内部会用短临界区保护报警状态写入。`BuzzerTask()` 读取报警状态时也会先取快照，避免任务间读写同一报警状态时出现不一致。

## 注意事项

- 注册失败会返回 `NULL`，上层应保存并检查返回值。
- 未注册的报警等级会被 `BuzzerTask()` 跳过。
- 如果没有任何报警处于 `ALARM_ON`，蜂鸣器占空比会被设置为 `0`。
- 当前模块只负责报警音输出，不负责报警来源判断；离线检测、故障判断等逻辑应由对应 module 或 application 完成。

## 后续扩展

@todo: 将单音报警扩展为可配置音序，例如通过 `"DoReMi"` 这类字符串描述报警旋律。
