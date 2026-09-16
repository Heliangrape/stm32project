# RoboMaster CAN 通讯完整指南

## 1. CAN 总线物理层

### 信号传输

```
STM32 PD1(TX) ──→ TTL(0V/3.3V) ──→ CAN收发器(TJA1050) ──→ CAN_H / CAN_L 差分信号

隐性（逻辑1）：CAN_H ≈ 2.5V, CAN_L ≈ 2.5V, 差分 ≈ 0V
显性（逻辑0）：CAN_H ≈ 3.5V, CAN_L ≈ 1.5V, 差分 ≈ 2V
```

### 终端电阻

CAN 总线两端各需要一个 120Ω 终端电阻，CAN_H 和 CAN_L 之间测量 ≈ 60Ω。

DJI C620 电调内建 120Ω 终端电阻。4 个电调全部打开 = 30Ω（太低，需拔掉多余的跳线帽）。

DJI C 板**不内置**终端电阻，需外部接。

---

## 2. CAN 帧格式

### 标准帧（11 位 ID）

```
SOF | ID[10:0] | RTR | IDE | r0 | DLC[3:0] | Data[0~7] | CRC | ACK | EOF
1b  |   11b    | 1b  | 1b  | 1b |   4b     |  0~64b    | 16b | 2b  | 7b
```

- **SOF**：帧起始（1 bit 显性）
- **ID**：仲裁 ID，决定优先级（越小越高）+ 接收过滤
- **RTR**：0 = 数据帧，1 = 远程帧
- **DLC**：数据长度（0~8 字节）
- **Data**：实际载荷，最多 8 字节（CAN 2.0 硬件限制）
- **CRC**：16 位循环冗余校验
- **ACK**：接收方应答（显性位覆盖）
- **EOF**：帧结束

### 8 字节限制

CAN 2.0 标准规定每帧最多 8 字节数据，这是硬件限制。发送超过 8 字节的数据需要分包（框架的 `can_comm.c` 已实现自动分包）。

---

## 3. DJI 电机 CAN 协议

### 控制帧（你发给电机）

**M3508 / M2006（C620 / C610 电调）：**

```
CAN ID: 0x200（电机 ID 1~4）
╔════════╤════════╤════════╤════════╗
║ b0 b1  │ b2 b3  │ b4 b5  │ b6 b7  ║
║ ID=1   │ ID=2   │ ID=3   │ ID=4   ║
║ 电流(mA)│ 电流(mA)│ 电流(mA)│ 电流(mA)║
║ int16  │ int16  │ int16  │ int16  ║
║ ±16384 │ ±16384 │ ±16384 │ ±16384 ║
╚════════╧════════╧════════╧════════╝

CAN ID: 0x1FF（电机 ID 5~8）
```

**GM6020（直驱，无电调）：**

```
CAN ID: 0x1FF（ID 1~4）/ 0x2FF（ID 5~8）
格式同上，但数据是电压指令（mV），范围 ±30000
```

### 反馈帧（电机回给你）

```
CAN ID: 0x200 + 电机拨码ID
  ID=1 → 0x201, ID=2 → 0x202, ID=3 → 0x203, ID=4 → 0x204

╔════════╤═══════╤═══════╤══════╤══════╗
║ b0 b1  │ b2 b3 │ b4 b5 │ b6   │ b7   ║
║ 编码器  │ 转速   │ 实际电流│ 温度  │ 保留  ║
║ uint16 │ int16 │ int16 │ uint8│      ║
║ 0~8191 │ RPM   │ mA    │ °C   │      ║
╚════════╧═══════╧═══════╧══════╧══════╝
```

**GM6020 反馈帧：**
```
CAN ID: 0x204 + 电机拨码ID
  ID=1 → 0x205, ID=2 → 0x206, ...
```

### 电机识别机制（两层）

```
第一层：CAN ID（tx_id）
  → 区分电机类型/协议（M3508 用 0x200，GM6020 用 0x1FF）

第二层：data_bytes（帧内位置）
  → 区分同类电机的第几个（拨码 ID=1 在 bytes 0-1，ID=2 在 bytes 2-3...）
```

---

## 4. 代码实现

### 最简发送（HAL 裸调）

```c
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

TxHeader.StdId = 0x200;       // CAN ID
TxHeader.IDE   = CAN_ID_STD;  // 标准帧
TxHeader.RTR   = CAN_RTR_DATA; // 数据帧
TxHeader.DLC   = 8;            // 8 字节

int16_t cur1 = 3000, cur2 = 3000, cur3 = 3000, cur4 = 3000;

TxData[0] = cur1 >> 8;   TxData[1] = cur1 & 0xFF;  // 电机1
TxData[2] = cur2 >> 8;   TxData[3] = cur2 & 0xFF;  // 电机2
TxData[4] = cur3 >> 8;   TxData[5] = cur3 & 0xFF;  // 电机3
TxData[6] = cur4 >> 8;   TxData[7] = cur4 & 0xFF;  // 电机4

HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
```

### 框架封装（bsp_can.c）

```c
// 注册：分配 CANInstance，配置发送参数，添加过滤器
CANInstance *CANRegister(uint32_t tx_id, uint32_t rx_id);

// 发送：等邮箱空闲 → 写邮箱寄存器
void CANTransmit(CANInstance *_instance);

// 接收：FIFO 中断 → 遍历查找匹配的实例 → 调用回调
void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox);
```

### 电机分组发送（dji_motor.c）

7 个预定义发送组，遍历发送：

```c
sender_assignment[0] = {&hcan1, 0x1FF};  // CAN1 → GM6020 ID 1~4 / M3508 ID 5~8
sender_assignment[1] = {&hcan1, 0x200};  // CAN1 → M3508 ID 1~4
sender_assignment[2] = {&hcan1, 0x2FF};  // CAN1 → GM6020 ID 5~8
sender_assignment[3] = {&hcan2, 0x1FF};  // CAN2 → 同上
sender_assignment[4] = {&hcan2, 0x200};
sender_assignment[5] = {&hcan2, 0x2FF};
```

`MotorSenderGrouping()` 根据电机类型和拨码ID，算出 `sender_group`（走哪个 CAN ID）和 `message_num`（帧内第几个 int16）。

---

## 5. CAN 过滤器

DJI C 板（STM32F407）有 28 个过滤器组：
- 0~13 → CAN1
- 14~27 → CAN2

框架用 ID_LIST 模式（16bit 位宽），每个组可匹配 4 个 CAN ID：

```c
can_filter_conf.FilterMode   = CAN_FILTERMODE_IDLIST;
can_filter_conf.FilterScale  = CAN_FILTERSCALE_16BIT;
can_filter_conf.FilterIdLow  = rx_id << 5;   // 要接收的 CAN ID
can_filter_conf.FilterBank   = can1_filter_idx++;  // 自动分配组号
```

只有匹配的 ID 才进 FIFO + 触发中断，不匹配的被硬件直接丢弃（0 CPU 开销）。

---

## 6. 常见故障

### 发送阻塞（邮箱卡死）

**原因**：CAN 总线上没有其他节点 → 发出去的帧没人回 ACK → `AutoRetransmission = ENABLE` → 硬件一直重发 → 邮箱不释放

**现象**：`while (HAL_CAN_GetTxMailboxesFreeLevel() == 0)` 死循环，FreeRTOS 任务卡死

**解决**：确保 CAN 总线至少有一个节点（C620 电调上电 + CAN 线连好）

### 收不到帧（静默丢帧）

**原因**：FIFO 深度只有 3 帧，中断处理不够快

**解决**：提高接收任务优先级；奇数 ID → FIFO0，偶数 → FIFO1 分流

### Bus-Off（CAN 彻底不工作）

**原因**：总线错误累计超阈值（短路、波特率不匹配、终端电阻不对）

**现象**：CAN 外设自动关闭，所有收/发失效

**检查**：`hcan.ErrorCode` 寄存器

### 通用 Debug 步骤

```
1. 万用表量 CAN_H/CAN_L 对地电压 → 2.5V = 空闲，跳动 = 有数据
2. 万用表量 CAN_H-CAN_L 电阻（断电）→ ≈60Ω = 终端电阻 OK
3. 示波器看 CAN_H 波形 → 有方波 = CAN 在发
4. 检查蜂鸣器/LED 确认 FreeRTOS 任务在跑
5. 检查 CubeMX：CAN 引脚 GPIO、NVIC 中断
6. 检查 CubeMX：波特率配置（Prescaler + BS1 + BS2）
```

---

## 7. 链路总图

```
你的代码
  ↓ HAL_CAN_AddTxMessage()
STM32 bxCAN 控制器
  ↓ PD1 引脚（TTL）
CAN 收发器芯片（TJA1050）
  ↓ CAN_H / CAN_L（差分）
C620/C610 电调 CAN 收发器
  ↓
电调 MCU 解析 → MOS 管驱动 → 三相 PWM → M3508 旋转
```

---

## 8. 关键数值速查

| 参数 | M3508 | GM6020 |
|------|-------|--------|
| 控制 CAN ID | 0x200 / 0x1FF | 0x1FF / 0x2FF |
| 反馈 CAN ID | 0x200 + ID | 0x204 + ID |
| 控制值 | 电流 mA（int16） | 电压 mV（int16） |
| 范围 | ±16384 | ±30000 |
| 帧大小 | 固定 8 字节 | 固定 8 字节 |
| 反馈频率 | 1kHz | 1kHz |
