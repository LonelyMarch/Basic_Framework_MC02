# bsp_flash

<p align='right'>neozng1@hnu.edu.cn</p>

## 简介

`bsp_flash` 是 STM32H723 片上 Flash 的 BSP 层封装，用于在程序运行时读写一块专门预留的用户参数区。它适合保存少量掉电不丢失的数据，例如校准参数、PID 参数、设备配置、版本标记和运行计数。

本模块只允许访问链接脚本中预留的 `USER_FLASH` 区域，不提供任意片上 Flash 绝对地址写入接口，避免应用层误擦除程序段。

## Flash 分区

当前工程使用的链接脚本为：

```text
STM32H723XG_FLASH.ld
```

片上 Flash 总容量为 1024KB，最后 128KB 被预留为用户参数区：

```ld
FLASH      (rx) : ORIGIN = 0x08000000, LENGTH = 896K
USER_FLASH (rx) : ORIGIN = 0x080E0000, LENGTH = 128K
```

链接脚本导出了用户区边界：

```ld
PROVIDE(__user_flash_start__ = ORIGIN(USER_FLASH));
PROVIDE(__user_flash_end__ = ORIGIN(USER_FLASH) + LENGTH(USER_FLASH));
```

因此程序段只能使用 `0x08000000 ~ 0x080DFFFF`，用户参数区为 `0x080E0000 ~ 0x080FFFFF`。如果程序超过 896KB，链接阶段会报错，不会悄悄占用用户参数区。

## STM32H723 Flash 特性

STM32H723 的片上 Flash 为单 Bank，共 8 个 sector，每个 sector 为 128KB。

本模块使用最后一个 sector 作为用户区：

```c
#define BSP_FLASH_SECTOR_SIZE   FLASH_SECTOR_SIZE
#define BSP_FLASH_PROGRAM_UNIT  (FLASH_NB_32BITWORD_IN_FLASHWORD * 4U)
```

在 STM32H723 上，`BSP_FLASH_PROGRAM_UNIT` 为 32 字节。片上 Flash 写入前必须先擦除，擦除后数据为 `0xFF`，写入只能把 bit 从 1 写成 0，不能直接把 0 写回 1。

## FreeRTOS 适配

调度器运行后，擦除和写入接口会通过递归互斥锁串行化，避免多个任务同时操作片上 Flash。

中断服务函数中不允许调用本模块接口。片上 Flash 擦写可能耗时较长，并且当前代码通常也从片上 Flash 取指，擦写期间会影响系统实时性。

不建议在高频控制任务中直接擦写片上 Flash。参数保存等操作建议放在低优先级任务或明确的安全时机中执行。

## 外部接口

```c
int8_t BSP_Flash_Init(void);

uint32_t BSP_Flash_GetUserStart(void);
uint32_t BSP_Flash_GetUserSize(void);

int8_t BSP_Flash_Read(uint32_t offset, void *buffer, uint32_t size);
int8_t BSP_Flash_Erase(uint32_t offset, uint32_t size);
int8_t BSP_Flash_EraseAll(void);
int8_t BSP_Flash_Write(uint32_t offset, const void *buffer, uint32_t size);

uint32_t BSP_Flash_GetLastError(void);
```

所有读写接口中的 `offset` 都是用户区内偏移，不是片上 Flash 的绝对地址。`offset = 0` 表示 `BSP_FLASH_USER_START`。

## 使用示例

```c
typedef struct
{
    uint32_t magic;
    float yaw_zero;
    float pitch_zero;
} GimbalParam;

GimbalParam param = {
    .magic = 0x47494D42,
    .yaw_zero = 0.0f,
    .pitch_zero = 0.0f,
};

BSP_Flash_EraseAll();
BSP_Flash_Write(0, &param, sizeof(param));
```

读取：

```c
GimbalParam param_read;

BSP_Flash_Read(0, &param_read, sizeof(param_read));
```

## 对齐要求

`BSP_Flash_Erase()` 的 `offset` 和 `size` 必须按 `BSP_FLASH_SECTOR_SIZE` 对齐。当前用户区只有一个 128KB sector，常用方式是直接调用：

```c
BSP_Flash_EraseAll();
```

`BSP_Flash_Write()` 的 `offset` 必须按 `BSP_FLASH_PROGRAM_UNIT` 对齐，也就是 32 字节对齐。`size` 可以不是 32 字节整数倍，最后不足 32 字节的部分会使用 `0xFF` 补齐后写入。

写入前必须先擦除目标区域。模块会检查目标 32 字节 flash word 是否仍为擦除态，如果不是擦除态，会返回 `BSP_FLASH_ERROR_NOT_ERASED`。

## 注意事项

- 本模块操作的是 STM32H723 片上 Flash，不是板载 W25Q64 外部 Flash。
- 程序区和用户区通过链接脚本隔离。
- 应用层只能通过用户区 offset 访问，不能传入任意绝对地址。
- 擦除单位为 128KB sector。
- 编程单位为 32 字节 flash word。
- 写入前必须擦除。
- 擦写接口不能在中断中调用。
- 擦写片上 Flash 会影响实时性，不建议在高频控制任务中调用。
- `BSP_Flash_GetLastError()` 可用于读取最近一次 HAL Flash 错误码。
