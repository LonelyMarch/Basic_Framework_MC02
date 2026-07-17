# ESP32-S3 HTTP 转 UART 通信协议文档

## 1. 系统概述

ESP32-S3 作为 HTTP 与 UART 的中转模块，负责接收客户端、视觉端和事件端的数据，并通过串口发送给 STM32。

ESP32-S3 提供四个 HTTP 接口：

```text
POST /api/command    接收控制命令，并立即转发给 STM32
POST /api/vision     接收视觉信息，并立即转发给 STM32，同时缓存最新一帧视觉信息
POST /api/event      接收事件信息，并立即转发给 STM32
GET  /api/fetch      获取 ESP32-S3 缓存的最新一次视觉信息
```

ESP32-S3 只负责：

```text
HTTP 接收
JSON 解析
转换为二进制 payload
通过 UART 发送给 STM32
```

ESP32-S3 不负责运动控制，不负责视觉决策。

---

# 2. HTTP 接口总览

| 接口             | 方法   | 功能       | 是否转发 STM32 |
| -------------- | ---- | -------- | ---------- |
| `/api/command` | POST | 接收控制命令   | 是          |
| `/api/vision`  | POST | 接收视觉信息   | 是          |
| `/api/event`   | POST | 接收事件信息   | 是          |
| `/api/fetch`   | GET  | 获取最新视觉信息 | 否          |

---

# 3. UART 数据类型

ESP32-S3 发给 STM32 的 UART 帧使用 `type` 区分数据类型。

|   type | 含义   | 对应接口           | payload          |
| -----: | ---- | -------------- | ---------------- |
| `0x01` | 命令信息 | `/api/command` | `CommandPayload` |
| `0x02` | 视觉信息 | `/api/vision`  | `VisionPayload`  |
| `0x03` | 事件信息 | `/api/event`   | `EventPayload`   |

---

# 4. 数据流

## 4.1 控制命令数据流

```text
控制客户端
    |
    | POST /api/command
    v
ESP32-S3
    |
    | UART type = 0x01
    v
STM32
```

控制客户端固定帧率发送，例如：

```text
30Hz / 50Hz / 100Hz
```

每一帧包含：

```text
WASD 按键状态
鼠标 dx / dy
旋转状态
时间戳
帧编号
```

---

## 4.2 视觉数据流

```text
视觉端
    |
    | POST /api/vision
    v
ESP32-S3
    |
    | UART type = 0x02
    v
STM32
```

ESP32-S3 收到视觉信息后会：

```text
1. 缓存最新一次视觉 JSON
2. 转换成 VisionPayload
3. 立即通过 UART 发给 STM32
```

---

## 4.3 事件数据流

```text
事件客户端
    |
    | POST /api/event
    v
ESP32-S3
    |
    | UART type = 0x03
    v
STM32
```

事件数据用于发送离散事件，例如：

```text
紧急停机
开始比赛
发球
```

事件不是固定帧率发送，而是发生时发送。

---

## 4.4 获取最新视觉数据流

```text
客户端
    |
    | GET /api/fetch
    v
ESP32-S3
    |
    | 返回最新一次视觉 JSON
    v
客户端
```

`/api/fetch` 只读取缓存，不会向 STM32 发送 UART 数据。

---

# 5. `/api/command` 控制命令接口

## 5.1 功能说明

`/api/command` 用于接收客户端发送的控制数据。

ESP32-S3 收到后，会将 JSON 转换为 `CommandPayload` 二进制结构体，并通过 UART 发送给 STM32。

UART 帧类型：

```text
type = 0x01
```

---

## 5.2 请求方式

```text
POST /api/command
Content-Type: application/json
```

---

## 5.3 推荐 JSON 格式

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 1,

  "w": true,
  "a": false,
  "s": false,
  "d": false,

  "dx": 3.5,
  "dy": -1.2,

  "rotate": -1
}
```

其中：

```text
w / a / s / d:
  true  = 按下
  false = 未按下

dx / dy:
  鼠标本帧位移增量

rotate:
  -1 = 逆时针
   0 = 不转
   1 = 顺时针
```

---

## 5.4 数组格式

也支持数组格式：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 1,

  "move": [1, 0, 0, 0],
  "mouse": [3.5, -1.2],

  "rotate": 1
}
```

含义：

```text
move[0] = w
move[1] = a
move[2] = s
move[3] = d

mouse[0] = dx
mouse[1] = dy
```

---

## 5.5 字段说明

| 字段       |         类型 | 说明                |
| -------- | ---------: | ----------------- |
| `sec`    |     uint32 | 秒级时间戳             |
| `ms`     |     uint16 | 毫秒，建议范围 `0 ~ 999` |
| `id`     |     uint32 | 帧编号，建议递增          |
| `w`      | bool / int | W 键状态             |
| `a`      | bool / int | A 键状态             |
| `s`      | bool / int | S 键状态             |
| `d`      | bool / int | D 键状态             |
| `dx`     |      float | 鼠标 X 方向本帧增量       |
| `dy`     |      float | 鼠标 Y 方向本帧增量       |
| `rotate` |       int8 | 旋转状态              |

`rotate` 最终约定：

```text
-1    逆时针
 0    不转 / 停止
 1    顺时针
```

---

## 5.6 成功响应

```json
{
  "ok": true,
  "msg": "command sent"
}
```

---

## 5.7 失败响应

空请求体：

```json
{
  "ok": false,
  "msg": "empty body"
}
```

JSON 格式错误：

```json
{
  "ok": false,
  "msg": "bad json"
}
```

---

# 6. `/api/vision` 视觉信息接口

## 6.1 功能说明

`/api/vision` 用于接收视觉端发送的识别结果。

ESP32-S3 收到后会：

```text
1. 保存该 JSON 作为最新视觉信息
2. 将 JSON 转换为 VisionPayload 二进制结构体
3. 通过 UART 发送给 STM32
```

UART 帧类型：

```text
type = 0x02
```

---

## 6.2 请求方式

```text
POST /api/vision
Content-Type: application/json
```

---

## 6.3 请求 JSON 示例

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 0,
  "result": -1,

  "order": [
    "禾", "人", "氺",
    "而", "王", "山",
    "雨", "口", "木"
  ],

  "goal": [
    0, -1, 1,
    0, -1, 1,
    0, -1, 1
  ],

  "coords": [
    [0, 0], [0, 0], [0, 0],
    [0, 0], [0, 0], [0, 0],
    [0, 0], [0, 0], [0, 0]
  ],

  "extrinsics": {
    "rvec": [0, 0, 0],
    "tvec": [0, 0, 0]
  }
}
```

---

## 6.4 字段说明

| 字段                |          类型 | 说明                   |
| ----------------- | ----------: | -------------------- |
| `sec`             |      uint32 | 秒级时间戳                |
| `ms`              |      uint16 | 毫秒                   |
| `id`              |      uint32 | 视觉帧编号                |
| `result`          |        int8 | 视觉识别结果，`-1` 表示无结果或失败 |
| `order`           |   string[9] | 九宫格字符顺序              |
| `goal`            |     int8[9] | 每个格子的目标状态            |
| `coords`          | float[9][2] | 九个目标的二维坐标            |
| `extrinsics.rvec` |    float[3] | 外参旋转向量               |
| `extrinsics.tvec` |    float[3] | 外参平移向量               |

---

## 6.5 九宫格顺序

`order`、`goal`、`coords` 均按照九宫格顺序排列：

```text
0 1 2
3 4 5
6 7 8
```

例如：

```text
coords[0] 对应左上角
coords[4] 对应中心
coords[8] 对应右下角
```

---

## 6.6 字符编号映射

HTTP 中 `order` 使用汉字字符串。

ESP32-S3 发送给 STM32 时，会转换为数字编号：

| 字符 | 编号 |
| -- | -: |
| 禾  |  0 |
| 人  |  1 |
| 氺  |  2 |
| 而  |  3 |
| 王  |  4 |
| 山  |  5 |
| 雨  |  6 |
| 口  |  7 |
| 木  |  8 |

未知字符填入：

```text
255
```

STM32 可将 `255` 视为无效字符。

---

## 6.7 成功响应

```json
{
  "ok": true,
  "msg": "vision sent"
}
```

---

## 6.8 失败响应

空请求体：

```json
{
  "ok": false,
  "msg": "empty body"
}
```

JSON 格式错误：

```json
{
  "ok": false,
  "msg": "bad json"
}
```

---

# 7. `/api/event` 事件接口

## 7.1 功能说明

`/api/event` 用于接收离散事件。

目前支持三种事件：

```text
紧急停机
开始比赛
发球
```

ESP32-S3 收到事件后，会将 JSON 转换为 `EventPayload` 二进制结构体，并立即通过 UART 发送给 STM32。

UART 帧类型：

```text
type = 0x03
```

---

## 7.2 请求方式

```text
POST /api/event
Content-Type: application/json
```

---

## 7.3 事件编号

| 事件   | 字符串              |     编号 |
| ---- | ---------------- | -----: |
| 紧急停机 | `emergency_stop` | `0x01` |
| 开始比赛 | `start_match`    | `0x02` |
| 发球   | `serve`          | `0x03` |

支持别名：

| 事件   | 支持写法                                       |
| ---- | ------------------------------------------ |
| 紧急停机 | `emergency_stop`、`estop`、`stop`、`紧急停机`、`1` |
| 开始比赛 | `start_match`、`start`、`开始比赛`、`2`           |
| 发球   | `serve`、`发球`、`3`                           |

---

## 7.4 请求 JSON 示例

### 紧急停机

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 1,
  "event": "emergency_stop"
}
```

或：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 1,
  "event": 1
}
```

---

### 开始比赛

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 2,
  "event": "start_match"
}
```

或：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 2,
  "event": 2
}
```

---

### 发球

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 3,
  "event": "serve"
}
```

或：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 3,
  "event": 3
}
```

---

## 7.5 可选 value 参数

事件帧中预留 `value` 字段，用于扩展。

例如：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 3,
  "event": "serve",
  "value": 1.0
}
```

当前如果不需要扩展参数，`value` 默认为：

```text
0.0
```

---

## 7.6 字段说明

| 字段      |           类型 | 说明              |
| ------- | -----------: | --------------- |
| `sec`   |       uint32 | 秒级时间戳           |
| `ms`    |       uint16 | 毫秒              |
| `id`    |       uint32 | 事件帧编号，建议递增      |
| `event` | string / int | 事件类型            |
| `value` |        float | 可选附加参数，默认 `0.0` |

---

## 7.7 成功响应

```json
{
  "ok": true,
  "msg": "event sent"
}
```

---

## 7.8 失败响应

空请求体：

```json
{
  "ok": false,
  "msg": "empty body"
}
```

JSON 格式错误：

```json
{
  "ok": false,
  "msg": "bad json"
}
```

未知事件：

```json
{
  "ok": false,
  "msg": "unknown event"
}
```

---

# 8. `/api/fetch` 获取最新视觉信息接口

## 8.1 功能说明

`/api/fetch` 用于获取 ESP32-S3 最近一次收到的视觉信息。

该接口只返回缓存的视觉 JSON，不会转发给 STM32。

```text
POST /api/vision 会更新缓存，并转发 STM32
GET  /api/fetch 只读取缓存，不转发 STM32
```

---

## 8.2 请求方式

```text
GET /api/fetch
```

---

## 8.3 成功响应

如果 ESP32-S3 已经收到过 `/api/vision` 数据，则返回最近一次视觉 JSON。

示例：

```json
{
  "sec": 1772183380,
  "ms": 0,
  "id": 0,
  "result": -1,

  "order": [
    "禾", "人", "氺",
    "而", "王", "山",
    "雨", "口", "木"
  ],

  "goal": [
    0, -1, 1,
    0, -1, 1,
    0, -1, 1
  ],

  "coords": [
    [0, 0], [0, 0], [0, 0],
    [0, 0], [0, 0], [0, 0],
    [0, 0], [0, 0], [0, 0]
  ],

  "extrinsics": {
    "rvec": [0, 0, 0],
    "tvec": [0, 0, 0]
  }
}
```

注意：成功时返回原始视觉 JSON，不额外包一层 `ok` 字段。

---

## 8.4 无视觉数据时响应

如果 ESP32-S3 启动后还没有收到过任何 `/api/vision` 数据，则返回：

```json
{
  "ok": false,
  "msg": "no vision data"
}
```

HTTP 状态码：

```text
404
```

---

## 8.5 缓存规则

ESP32-S3 内部只缓存最新一帧视觉信息。

```text
每次 POST /api/vision 成功后，都会覆盖旧的视觉缓存
GET /api/fetch 永远返回最近一次成功收到的视觉 JSON
```

---

# 9. UART 串口协议

## 9.1 串口配置

推荐 UART 配置：

```text
波特率：921600
数据位：8
校验位：None
停止位：1
流控：None
```

即：

```text
921600 8N1
```

接线：

```text
ESP32-S3 TX  -> STM32 RX
ESP32-S3 RX  -> STM32 TX
ESP32-S3 GND -> STM32 GND
```

注意：两边必须共地。

---

## 9.2 串口帧格式

ESP32-S3 通过 UART 向 STM32 发送二进制帧。

完整帧格式：

```text
AA 55 | type | len_low | len_high | payload | checksum
```

字段说明：

| 字段         |   长度 | 说明              |
| ---------- | ---: | --------------- |
| `AA 55`    | 2 字节 | 帧头              |
| `type`     | 1 字节 | 数据类型            |
| `len_low`  | 1 字节 | payload 长度低 8 位 |
| `len_high` | 1 字节 | payload 长度高 8 位 |
| `payload`  | N 字节 | 实际数据内容          |
| `checksum` | 1 字节 | 校验和             |

---

## 9.3 payload 长度

payload 长度由 `len_low` 和 `len_high` 组成：

```c
uint16_t length = len_low | (len_high << 8);
```

低字节在前，高字节在后，即小端格式。

---

## 9.4 checksum 校验

校验和计算范围：

```text
type + len_low + len_high + payload 所有字节
```

计算方式：

```c
uint8_t checksum = 0;

checksum += type;
checksum += len_low;
checksum += len_high;

for (int i = 0; i < length; i++) {
    checksum += payload[i];
}
```

最终结果取低 8 位。

接收端计算出的 checksum 必须与帧尾 checksum 相等，否则丢弃该帧。

---

# 10. Payload 说明

`payload` 是 UART 帧中的实际数据内容。

ESP32-S3 收到 HTTP JSON 后，会将 JSON 转换成固定二进制结构体，然后把结构体的原始字节作为 `payload` 发送给 STM32。

因此：

```text
HTTP JSON 是网络传输格式
UART payload 是 STM32 实际解析的数据格式
```

STM32 不需要解析 JSON，只需要按照固定结构体解析二进制 payload。

---

# 11. CommandPayload 命令数据

## 11.1 使用条件

当 UART 帧中：

```text
type = 0x01
```

表示该帧的 payload 是命令数据。

---

## 11.2 结构体定义

ESP32-S3 和 STM32 使用相同结构体：

```c
typedef struct __attribute__((packed)) {
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    uint8_t w;
    uint8_t a;
    uint8_t s;
    uint8_t d;

    float dx;
    float dy;

    int8_t rotate;
} CommandPayload;
```

---

## 11.3 字段说明

| 字段       |       类型 | 说明                 |
| -------- | -------: | ------------------ |
| `sec`    | uint32_t | 秒级时间戳              |
| `ms`     | uint16_t | 毫秒                 |
| `id`     | uint32_t | 命令帧编号              |
| `w`      |  uint8_t | W 键，`0` 未按下，`1` 按下 |
| `a`      |  uint8_t | A 键，`0` 未按下，`1` 按下 |
| `s`      |  uint8_t | S 键，`0` 未按下，`1` 按下 |
| `d`      |  uint8_t | D 键，`0` 未按下，`1` 按下 |
| `dx`     |    float | 鼠标 X 方向本帧增量        |
| `dy`     |    float | 鼠标 Y 方向本帧增量        |
| `rotate` |   int8_t | 旋转状态               |

`rotate` 约定：

```text
-1    逆时针
 0    不转 / 停止
 1    顺时针
```

---

## 11.4 Payload 长度

```text
sizeof(CommandPayload) = 23 字节
```

计算方式：

```text
uint32_t sec      4 字节
uint16_t ms       2 字节
uint32_t id       4 字节

uint8_t w         1 字节
uint8_t a         1 字节
uint8_t s         1 字节
uint8_t d         1 字节

float dx          4 字节
float dy          4 字节

int8_t rotate     1 字节
-------------------------
总计              23 字节
```

---

## 11.5 字节偏移

| 字段       |       类型 | 偏移 | 长度 |
| -------- | -------: | -: | -: |
| `sec`    | uint32_t |  0 |  4 |
| `ms`     | uint16_t |  4 |  2 |
| `id`     | uint32_t |  6 |  4 |
| `w`      |  uint8_t | 10 |  1 |
| `a`      |  uint8_t | 11 |  1 |
| `s`      |  uint8_t | 12 |  1 |
| `d`      |  uint8_t | 13 |  1 |
| `dx`     |    float | 14 |  4 |
| `dy`     |    float | 18 |  4 |
| `rotate` |   int8_t | 22 |  1 |

---

## 11.6 STM32 使用示例

```c
if (type == 0x01 && length == sizeof(CommandPayload)) {
    CommandPayload cmd;
    memcpy(&cmd, payload, sizeof(CommandPayload));

    if (cmd.w) {
        // W 按下
    }

    if (cmd.a) {
        // A 按下
    }

    if (cmd.s) {
        // S 按下
    }

    if (cmd.d) {
        // D 按下
    }

    float dx = cmd.dx;
    float dy = cmd.dy;

    if (cmd.rotate == -1) {
        // 逆时针
    } else if (cmd.rotate == 1) {
        // 顺时针
    } else {
        // 不转
    }
}
```

---

# 12. VisionPayload 视觉数据

## 12.1 使用条件

当 UART 帧中：

```text
type = 0x02
```

表示该帧的 payload 是视觉数据。

---

## 12.2 结构体定义

ESP32-S3 和 STM32 使用相同结构体：

```c
typedef struct __attribute__((packed)) {
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    int8_t result;

    uint8_t order[9];
    int8_t goal[9];

    float coords[9][2];

    float rvec[3];
    float tvec[3];
} VisionPayload;
```

---

## 12.3 Payload 长度

```text
sizeof(VisionPayload) = 125 字节
```

计算方式：

```text
uint32_t sec          4 字节
uint16_t ms           2 字节
uint32_t id           4 字节

int8_t result         1 字节

uint8_t order[9]      9 字节
int8_t goal[9]        9 字节

float coords[9][2]   72 字节
float rvec[3]        12 字节
float tvec[3]        12 字节
-----------------------------
总计                125 字节
```

---

## 12.4 字节偏移

| 字段             |         类型 |  偏移 | 长度 |
| -------------- | ---------: | --: | -: |
| `sec`          |   uint32_t |   0 |  4 |
| `ms`           |   uint16_t |   4 |  2 |
| `id`           |   uint32_t |   6 |  4 |
| `result`       |     int8_t |  10 |  1 |
| `order[9]`     | uint8_t[9] |  11 |  9 |
| `goal[9]`      |  int8_t[9] |  20 |  9 |
| `coords[9][2]` |  float[18] |  29 | 72 |
| `rvec[3]`      |   float[3] | 101 | 12 |
| `tvec[3]`      |   float[3] | 113 | 12 |

---

## 12.5 order 字段说明

`order[9]` 是字符编号数组。

编号规则：

```text
禾 -> 0
人 -> 1
氺 -> 2
而 -> 3
王 -> 4
山 -> 5
雨 -> 6
口 -> 7
木 -> 8
未知字符 -> 255
```

---

## 12.6 goal 字段说明

`goal[9]` 表示每个格子的目标状态。

当前常见取值为：

```text
-1
 0
 1
```

九个元素对应九宫格顺序：

```text
0 1 2
3 4 5
6 7 8
```

---

## 12.7 coords 字段说明

`coords[9][2]` 表示九个目标点的二维坐标。

```c
coords[i][0] = 第 i 个目标的 x 坐标
coords[i][1] = 第 i 个目标的 y 坐标
```

其中 `i` 的范围：

```text
0 ~ 8
```

---

## 12.8 rvec / tvec 字段说明

`rvec[3]` 表示视觉外参旋转向量。

`tvec[3]` 表示视觉外参平移向量。

```text
rvec[0], rvec[1], rvec[2]
tvec[0], tvec[1], tvec[2]
```

---

## 12.9 STM32 使用示例

```c
if (type == 0x02 && length == sizeof(VisionPayload)) {
    VisionPayload vision;
    memcpy(&vision, payload, sizeof(VisionPayload));

    int8_t result = vision.result;

    for (int i = 0; i < 9; i++) {
        uint8_t char_id = vision.order[i];
        int8_t goal = vision.goal[i];

        float x = vision.coords[i][0];
        float y = vision.coords[i][1];
    }

    float rx = vision.rvec[0];
    float ry = vision.rvec[1];
    float rz = vision.rvec[2];

    float tx = vision.tvec[0];
    float ty = vision.tvec[1];
    float tz = vision.tvec[2];
}
```

---

# 13. EventPayload 事件数据

## 13.1 使用条件

当 UART 帧中：

```text
type = 0x03
```

表示该帧的 payload 是事件数据。

---

## 13.2 结构体定义

ESP32-S3 和 STM32 使用相同结构体：

```c
typedef struct __attribute__((packed)) {
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    uint8_t event;

    float value;
} EventPayload;
```

---

## 13.3 Payload 长度

```text
sizeof(EventPayload) = 15 字节
```

计算方式：

```text
uint32_t sec      4 字节
uint16_t ms       2 字节
uint32_t id       4 字节

uint8_t event     1 字节
float value       4 字节
-------------------------
总计              15 字节
```

---

## 13.4 字节偏移

| 字段      |       类型 | 偏移 | 长度 |
| ------- | -------: | -: | -: |
| `sec`   | uint32_t |  0 |  4 |
| `ms`    | uint16_t |  4 |  2 |
| `id`    | uint32_t |  6 |  4 |
| `event` |  uint8_t | 10 |  1 |
| `value` |    float | 11 |  4 |

---

## 13.5 event 字段说明

|  event | 含义   |
| -----: | ---- |
| `0x01` | 紧急停机 |
| `0x02` | 开始比赛 |
| `0x03` | 发球   |

---

## 13.6 value 字段说明

`value` 是事件附加参数，当前可以默认不用。

默认值：

```text
0.0
```

预留用途：

```text
发球方向
发球力度
事件附加参数
调试参数
```

---

## 13.7 STM32 使用示例

```c
if (type == 0x03 && length == sizeof(EventPayload)) {
    EventPayload event_payload;
    memcpy(&event_payload, payload, sizeof(EventPayload));

    switch (event_payload.event) {
        case 0x01:
            // 紧急停机
            break;

        case 0x02:
            // 开始比赛
            break;

        case 0x03:
            // 发球
            break;

        default:
            // 未知事件
            break;
    }

    float value = event_payload.value;
}
```

---

# 14. STM32 串口接收流程

STM32 端建议使用状态机解析 UART 数据。

解析流程：

```text
1. 查找帧头 0xAA
2. 查找第二个帧头 0x55
3. 读取 type
4. 读取 len_low
5. 读取 len_high
6. 计算 payload 长度
7. 读取 payload
8. 读取 checksum
9. 校验 checksum
10. 根据 type 解析 payload
```

伪代码：

```c
if (head1 == 0xAA && head2 == 0x55) {
    type = read_byte();

    len_low = read_byte();
    len_high = read_byte();

    length = len_low | (len_high << 8);

    read payload[length];

    checksum_recv = read_byte();

    checksum_calc = calc_checksum(type, len_low, len_high, payload, length);

    if (checksum_recv == checksum_calc) {
        if (type == 0x01) {
            parse CommandPayload;
        } else if (type == 0x02) {
            parse VisionPayload;
        } else if (type == 0x03) {
            parse EventPayload;
        }
    }
}
```

---

# 15. STM32 错误处理建议

STM32 收到以下情况时，应直接丢弃当前帧：

```text
帧头错误
type 未知
payload 长度异常
checksum 校验失败
payload 长度超过缓冲区
```

长度检查建议：

```c
if (type == 0x01 && length != sizeof(CommandPayload)) {
    discard_frame();
}

if (type == 0x02 && length != sizeof(VisionPayload)) {
    discard_frame();
}

if (type == 0x03 && length != sizeof(EventPayload)) {
    discard_frame();
}
```

建议最大 payload 缓冲区：

```c
#define UART_MAX_PAYLOAD_SIZE 256
```

当前最大 payload 为：

```text
VisionPayload = 125 字节
```

因此 256 字节缓冲区足够。

---

# 16. 字节序与浮点格式

协议规定：

```text
整数使用小端格式
float 使用 IEEE754 单精度浮点数
```

ESP32-S3 和常见 STM32 默认均为小端格式。

所有结构体必须使用 `packed`，避免编译器自动字节对齐：

```c
typedef struct __attribute__((packed)) {
    ...
} Payload;
```

STM32 端建议使用 `memcpy` 解析 payload：

```c
CommandPayload cmd;
memcpy(&cmd, payload_buffer, sizeof(CommandPayload));
```

不建议直接强转指针：

```c
CommandPayload* cmd = (CommandPayload*)payload_buffer;
```

因为未对齐访问在部分 MCU 上可能导致问题。

---

# 17. 协议常量汇总

```c
#define FRAME_HEAD_1          0xAA
#define FRAME_HEAD_2          0x55

#define FRAME_TYPE_COMMAND    0x01
#define FRAME_TYPE_VISION     0x02
#define FRAME_TYPE_EVENT      0x03

#define EVENT_EMERGENCY_STOP  0x01
#define EVENT_START_MATCH     0x02
#define EVENT_SERVE           0x03

#define COMMAND_PAYLOAD_SIZE  23
#define VISION_PAYLOAD_SIZE   125
#define EVENT_PAYLOAD_SIZE    15

#define UART_MAX_PAYLOAD_SIZE 256
```

更推荐写法：

```c
#define COMMAND_PAYLOAD_SIZE  ((uint16_t)sizeof(CommandPayload))
#define VISION_PAYLOAD_SIZE   ((uint16_t)sizeof(VisionPayload))
#define EVENT_PAYLOAD_SIZE    ((uint16_t)sizeof(EventPayload))
```

这样结构体修改后，长度会自动同步。

---

# 18. HTTP 测试示例

## 18.1 发送命令数据

```bash
curl -X POST http://esp32_ip/api/command \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 1,
    "w": true,
    "a": false,
    "s": false,
    "d": false,
    "dx": 3.5,
    "dy": -1.2,
    "rotate": -1
  }'
```

---

## 18.2 发送视觉数据

```bash
curl -X POST http://esp32_ip/api/vision \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 0,
    "result": -1,
    "order": ["禾", "人", "氺", "而", "王", "山", "雨", "口", "木"],
    "goal": [0, -1, 1, 0, -1, 1, 0, -1, 1],
    "coords": [
      [0, 0], [0, 0], [0, 0],
      [0, 0], [0, 0], [0, 0],
      [0, 0], [0, 0], [0, 0]
    ],
    "extrinsics": {
      "rvec": [0, 0, 0],
      "tvec": [0, 0, 0]
    }
  }'
```

---

## 18.3 获取最新视觉数据

```bash
curl http://esp32_ip/api/fetch
```

如果已经收到过视觉数据，会返回最近一次 `/api/vision` 的 JSON。

如果还没有收到过视觉数据，会返回：

```json
{
  "ok": false,
  "msg": "no vision data"
}
```

---

## 18.4 发送紧急停机事件

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 1,
    "event": "emergency_stop"
  }'
```

或：

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 1,
    "event": 1
  }'
```

---

## 18.5 发送开始比赛事件

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 2,
    "event": "start_match"
  }'
```

或：

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 2,
    "event": 2
  }'
```

---

## 18.6 发送发球事件

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 3,
    "event": "serve"
  }'
```

带附加参数：

```bash
curl -X POST http://esp32_ip/api/event \
  -H "Content-Type: application/json" \
  -d '{
    "sec": 1772183380,
    "ms": 0,
    "id": 3,
    "event": "serve",
    "value": 1.0
  }'
```

---

# 19. 最终接口行为总结

| 接口                  | 行为                                                     |
| ------------------- | ------------------------------------------------------ |
| `POST /api/command` | 接收控制命令，转换为 `CommandPayload`，通过 UART 发送给 STM32          |
| `POST /api/vision`  | 接收视觉信息，缓存最新 JSON，转换为 `VisionPayload`，通过 UART 发送给 STM32 |
| `POST /api/event`   | 接收事件信息，转换为 `EventPayload`，通过 UART 发送给 STM32            |
| `GET /api/fetch`    | 返回最近一次视觉 JSON，不发送 UART                                 |

核心原则：

```text
命令数据固定帧率发送
视觉数据有结果就发送
事件数据发生时发送
ESP32-S3 收到就发
UART 使用 type 区分数据类型
STM32 只解析二进制 payload，不解析 JSON
/api/fetch 只读取视觉缓存，不影响 STM32
```

最终 `CommandPayload` 中：

```text
w/a/s/d 使用 uint8_t
0 = 未按下
1 = 按下

rotate 使用 int8_t
-1 = 逆时针
 0 = 不转
 1 = 顺时针
```
