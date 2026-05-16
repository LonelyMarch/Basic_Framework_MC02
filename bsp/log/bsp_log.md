# bsp_log

<p align='right'>neozng1@hnu.edu.cn</p>

## 使用说明

bsp_log是基于SEGGER RTT实现的日志打印模块。RTT不需要在CubeMX中单独开启,工程已经携带SEGGER RTT源码并在CMake中加入构建。使用时需要保留SWD/JTAG调试接口,并通过支持RTT的调试器或上位机工具读取日志,推荐使用J-Link。

推荐使用`bsp_log.h`中提供了三级日志：

```c
#define LOGINFO(format,...)
#define LOGWARNING(format,...)
#define LOGERROR(format,...)
```

分别用于输出不同等级的日志。注意RTT不支持直接使用`%f`进行浮点格式化,建议将浮点量换算成整数单位后再打印,例如将秒换算成毫秒或微秒。

当前CMake配置中,Debug默认开启日志,Release、RelWithDebInfo、MinSizeRel默认定义`DISABLE_LOG_SYSTEM=1`并关闭`LOGINFO`、`LOGWARNING`、`LOGERROR`。

当前日志为同步RTT输出。高频任务中应避免连续刷日志,中断上下文中也不建议直接输出复杂日志。

## 调试工具

CLion环境下,如果使用J-Link GDB Server调试,可以配合J-Link RTT Viewer、Ozone或SEGGER RTT Client查看日志。

VSCode/Ozone环境仍然兼容。若使用原有VSCode调试流程,可以通过`launch.json`中的`debug-jlink`启动调试。若使用cmsis-dap或daplink配合J-Link工具链,请在*jlink*调试任务启动之后再打开`log`任务,否则可能出现RTT Viewer无法连接客户端的情况。

在Ozone中查看日志时,可以打开console调试任务台和terminal调试终端查看输出。

> 由于Ozone版本或终端支持情况不同,可能出现日志不换行或没有颜色。

## 自定义输出

你也可以自定义输出格式，详见Segger RTT的文档。

```c
int PrintLog(const char *fmt, ...);
```

调用第一个函数，可以通过jlink或dap-link向调试器连接的上位机发送信息，格式和printf相同，示例如下：

```c
PrintLog("Hello World!\n");
PrintLog("Motor %d met some problem, error code %d!\n", 3, 1);
```

## @TODO

后续计划将日志系统改为异步输出链路:

```text
LOG宏 -> 格式化到短buffer -> 投递到日志队列 -> LogTask统一输出RTT/USB
```

该方案用于降低高频任务和中断上下文直接输出日志对实时性的影响。实现时需要处理FreeRTOS启动前日志、ISR中队列投递、队列满后的丢弃策略、LogTask优先级、RTT/USB后端选择以及USB CDC与其他业务通信共用通道的问题。
