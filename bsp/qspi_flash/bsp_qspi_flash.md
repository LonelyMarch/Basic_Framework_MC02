# bsp_qspi_flash

`bsp_qspi_flash` 封装板载 W25Q64 外部 Flash。当前通过 STM32H723 的 OCTOSPI 外设工作在 QSPI 兼容模式。

## 基本参数

- JEDEC ID: `0xEF4017`
- 容量: 8 MiB
- 页大小: 256 B
- 扇区大小: 4 KiB
- 32 KiB block erase: `0x52`
- 64 KiB block erase: `0xD8`
- 内存映射基地址: `0x90000000`

## 初始化

```c
BSP_QSPI_Flash_Init();
```

初始化会读取 JEDEC ID,确认外部 Flash 通信正常。失败会返回错误码并输出日志。

## 同步接口

```c
uint32_t BSP_QSPI_Flash_ReadID(void);
int8_t BSP_QSPI_Flash_Read(uint32_t addr, uint8_t *buffer, uint32_t size);
int8_t BSP_QSPI_Flash_Write(uint32_t addr, const uint8_t *buffer, uint32_t size);
int8_t BSP_QSPI_Flash_WritePage(uint32_t addr, const uint8_t *buffer, uint16_t size);
int8_t BSP_QSPI_Flash_EraseSector(uint32_t addr);
int8_t BSP_QSPI_Flash_EraseBlock32K(uint32_t addr);
int8_t BSP_QSPI_Flash_EraseBlock64K(uint32_t addr);
int8_t BSP_QSPI_Flash_EraseRange(uint32_t addr, uint32_t size);
int8_t BSP_QSPI_Flash_ChipErase(void);
```

写入前必须先擦除目标区域。`BSP_QSPI_Flash_Write()` 会自动按页拆分,但不会替你擦除。

`BSP_QSPI_Flash_EraseRange()` 要求起始地址和长度都按 4 KiB 对齐,内部会优先使用 64 KiB / 32 KiB 块擦除,剩余部分使用 4 KiB
扇区擦除。

## 内存映射

```c
BSP_QSPI_Flash_MemoryMappedMode();
BSP_QSPI_Flash_AbortMemoryMappedMode();
```

进入内存映射模式后,CPU 可以从 `0x90000000` 直接读取外部 Flash。该模式适合只读访问。后续如需擦写,本 BSP 会先退出内存映射模式。

## 异步接口

低优先级异步擦写由 `bsp_flash_async` 提供:

```c
BSP_FlashAsyncPostQspiErase(addr, size);
BSP_FlashAsyncPostQspiWrite(addr, data, size);
```

异步层只负责排队和复制数据,最终仍调用本模块同步接口。

## FreeRTOS约束

QSPI Flash 擦写和自动轮询可能耗时较长,不能在中断中调用。同步接口内部使用互斥锁串行化访问。

## CubeMX要求

OCTOSPI 引脚、时钟、片选、时序和 FIFO 等配置由 CubeMX 生成。若修改 QSPI 时钟或 Flash 型号,需要重新检查 dummy cycle、命令和容量宏。

## 注意事项

- Flash 写入只能把 bit 从 1 写成 0。
- Chip erase 耗时很长,应谨慎使用。
- 默认不长期保持内存映射模式,避免影响后续间接命令擦写。
