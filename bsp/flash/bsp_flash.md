# bsp_flash

`bsp_flash` 封装 STM32H723 片上 Flash 用户区的读取、擦除和写入。用户区由链接脚本单独保留,避免和程序代码段冲突。

## Flash分区

链接脚本中普通程序段使用 `FLASH`,用户数据区使用 `USER_FLASH`:

```ld
FLASH      (rx) : ORIGIN = 0x08000000, LENGTH = 896K
USER_FLASH (rx) : ORIGIN = 0x080E0000, LENGTH = 128K
```

并通过符号暴露给 BSP:

```ld
PROVIDE(__user_flash_start__ = ORIGIN(USER_FLASH));
PROVIDE(__user_flash_end__ = ORIGIN(USER_FLASH) + LENGTH(USER_FLASH));
```

BSP 接口使用用户区内偏移,`offset = 0` 表示 `BSP_FLASH_USER_START`。

## 初始化

```c
BSP_Flash_Init();
```

初始化只创建内部递归互斥锁并清理状态,不会主动擦写 Flash。

## 同步接口

```c
int8_t BSP_Flash_Read(uint32_t offset, void *buffer, uint32_t size);
int8_t BSP_Flash_Erase(uint32_t offset, uint32_t size);
int8_t BSP_Flash_EraseAll(void);
int8_t BSP_Flash_Write(uint32_t offset, const void *buffer, uint32_t size);
uint32_t BSP_Flash_GetUserStart(void);
uint32_t BSP_Flash_GetUserSize(void);
uint32_t BSP_Flash_GetLastError(void);
```

`BSP_Flash_Erase()` 的 `offset` 和 `size` 必须按 Flash sector 对齐。`BSP_Flash_Write()` 的 `offset` 必须按 Flash program
unit 对齐,写入前目标区域必须已经擦除。

## 异步接口

如果不希望业务任务直接等待擦写完成,可以使用 `bsp_flash_async`:

```c
BSP_FlashAsyncPostOnchipErase(offset, size);
BSP_FlashAsyncPostOnchipWrite(offset, data, size);
```

异步层会复制写入数据并投递到 FreeRTOS 静态 Queue,由低优先级 `BSP_FlashAsyncTask` 调用本模块同步接口执行。

## FreeRTOS约束

片上 Flash 擦写耗时较长,并且程序通常也从片上 Flash 取指。同步擦写接口不能在中断中调用,也不建议放在高优先级实时任务中。

模块内部使用互斥锁串行化 Flash 操作。

## 注意事项

- Flash 只能把 bit 从 1 写为 0,写入前必须擦除。
- 擦除粒度是 sector,写入粒度是 Flash word。
- 用户区大小由链接脚本决定,不是 CubeMX MPU 配置决定。
