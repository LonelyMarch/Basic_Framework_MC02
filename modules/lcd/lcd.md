# lcd

`modules/lcd` 是达妙 DM_TFT-V1.1 外接 LCD 的模块层驱动，显示控制器为 ST7789V2，屏幕有效显示区域为 `240x280`
。当前默认按照达妙官方例程配置为横屏：`LCD_USE_HORIZONTAL=2`，因此 `LCD_W=280`、`LCD_H=240`。

## 硬件连接

主控板外接 LCD 连接器中，显示部分使用 SPI1 和 3 个控制 GPIO：

- `SPI1_CS`：`PE15`
- `SPI1_SCK`：`PB03`
- `SPI1_MOSI`：`PD07`
- `LCD_DC`：`PD10`
- `LCD_RES`：`PB11`
- `LCD_BLK`：`PB10`

原理图中还引出了 `SPI1_MISO`、`I2C2_SCL/SDA` 和 `ADC1_CH18_KEY`，它们用于模块扩展能力，例如读屏、触摸或按键采样；当前 LCD
驱动只负责显示，不处理触摸和按键。

## 工作模式

LCD 模块通过当前工程的 `bsp/spi` 和 `bsp/gpio` 接入 BSP 系统：

- `LCD_Init()` 内部注册 SPI1 LCD 从设备，片选脚为 `LCD_CS`。
- `LCD_DC`、`LCD_RES`、`LCD_BLK` 通过 `GPIORegister()` 注册为普通输出。
- SPI 总线互斥由 `bsp/spi` 负责。
- LCD 命令序列互斥由模块内部 `lcd_mutex` 负责，避免多个任务同时绘图时命令和显存数据交错。
- `LCDTaskInit()` 会先确保 `LCD_Init()` 已完成，再统一创建 LCD 互斥锁、低优先级绘图任务和命令队列，application 层在
  `OSTaskInit()` 中调用。

默认 `LCD_SPI_WORK_MODE=SPI_BLOCK_MODE`，这样 LCD 初始化可以在 FreeRTOS 调度器启动前执行。若确认 LCD 只在任务中绘制，并且
CubeMX 已开启 SPI1 TX DMA，可以在工程宏里改为：

```c
#define LCD_SPI_WORK_MODE SPI_DMA_MODE
```

## 初始化

```c
if (LCD_Init() == HAL_OK)
{
    LCD_Clear(LCD_BLACK);
}
```

初始化流程沿用达妙官方例程：

- 拉低 `RES` 复位。
- 打开背光 `BLK`。
- 发送 ST7789V2 `Sleep Out`、像素格式、显示方向、Gamma 等寄存器配置。
- 发送 `Display On`。

## 常用接口

```c
LCD_Clear(LCD_BLACK);
LCD_Fill(0, 0, LCD_W, LCD_H, LCD_BLACK);
LCD_DrawPoint(20, 20, LCD_RED);
LCD_DrawLine(10, 10, 100, 80, LCD_WHITE);
LCD_DrawRectangle(20, 20, 120, 80, LCD_GREEN);
LCD_DrawCircle(140, 120, 30, LCD_BLUE);
```

`LCD_Fill()` 的 `xend/yend` 是右边界和下边界的后一位，和常见半开区间一致。例如填满全屏应写
`LCD_Fill(0, 0, LCD_W, LCD_H, color)`。

`LCD_Fill()`、水平线、垂直线和矩形边框会通过区域窗口连续写入同一个颜色，适合绘制背景块、边框和简单
UI。斜线和圆形仍按像素点绘制，不建议在高频任务中大量刷新。

## 异步绘图

同步绘图接口会在调用者任务中完成 SPI 刷屏。全屏填充需要写入 `280 * 240 * 2` 字节，在当前 SPI1 约 `30 Mbits/s`
的配置下会占用几十毫秒，因此高优先级任务不应直接调用大面积刷新接口。

异步接口只把绘图命令投递到 LCD 队列，由低优先级 `lcdtask` 串行执行：

```c
LCD_AsyncClear(LCD_BLACK);
LCD_AsyncFill(0, 0, LCD_W, 24, LCD_DARKBLUE);
LCD_AsyncDrawRectangle(4, 4, 120, 60, LCD_GREEN);
LCD_AsyncPrintf(8, 8, LCD_WHITE, LCD_DARKBLUE, 16, LCD_TEXT_MODE_NORMAL, "fps:%u", fps);
```

异步接口只适合在 FreeRTOS 调度器启动后使用。当前工程在 `OSTaskInit()` 中调用 `LCDTaskInit()`，该函数会先确保 `LCD_Init()`
成功，再创建异步绘图队列。队列深度由 `LCD_ASYNC_QUEUE_DEPTH` 控制，默认 8；队列满时接口返回 `HAL_BUSY`，上层可以直接丢弃本次调试刷新。

`LCD_ShowPicture()` 仍保留为同步接口。图片数据往往较大，若异步化需要保证图片缓冲区在 LCDTask 消费前一直有效，后续确实需要时再单独扩展。

## 文本显示

ASCII 字符支持 `12/16/24/32` 像素高度，宽度为高度的一半：

```c
LCD_ShowString(10, 20, (const uint8_t *)"DM LCD", LCD_WHITE, LCD_BLACK, 24, LCD_TEXT_MODE_NORMAL);
LCD_ShowIntNum(10, 60, 1234, 5, LCD_YELLOW, LCD_BLACK, 24);
LCD_ShowFloatNum(10, 100, -12.34f, 3, 2, LCD_CYAN, LCD_BLACK, 24);
LCD_Printf(10, 140, LCD_GREEN, LCD_BLACK, 16, LCD_TEXT_MODE_NORMAL, "fps:%u", fps);
```

中文显示沿用官方字库文件 `lcd_font.h`，只支持字库中已经取模的汉字。官方字库内置了“达妙科技”四个字，驱动已经对这四个字做了
UTF-8 到 GBK 字库索引的映射，因此工程源码按 UTF-8 保存时可以直接这样写：

```c
LCD_ShowChinese(10, 180, (const uint8_t *)"达妙", LCD_WHITE, LCD_BLACK, 32, LCD_TEXT_MODE_NORMAL);
```

显示模式：

- `LCD_TEXT_MODE_NORMAL`：同时写前景色和背景色，会覆盖字符矩形区域。
- `LCD_TEXT_MODE_OVERLAY`：只写前景像素，背景保持不变。

## 图片显示

图片接口使用 RGB565 数据，数据顺序为高字节在前：

```c
LCD_ShowPicture(0, 0, width, height, image_rgb565);
```

## 注意事项

- LCD 是调试显示设备，不建议在高优先级控制任务中频繁全屏刷新。
- 若使用 `SPI_DMA_MODE`，必须保证 `SPI1_TX DMA` 和 `SPI1 global interrupt` 已在 CubeMX 中开启。
- 当前模块不主动修改 `.ioc`，引脚和 DMA 配置需要在 CubeMX 中保持与主控板原理图一致。
