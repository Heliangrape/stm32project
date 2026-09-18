# STM32F407 四轮 M3508 速度闭环控制

基于 STM32F407 + HAL 库的 RoboMaster 四电机例程：4× M3508（C620 电调）速度闭环，
用 DBUS 遥控器（DT7 + DR16）实时控制转速。**裸机，无 RTOS**，CubeMX 生成骨架。

## 功能

- 4 × M3508 速度闭环，1 kHz 控制周期
- DBUS 遥控器收帧（DMA + 空闲中断），带掉线超时检测
- CAN1 收 4 路电调反馈（0x201~0x204），发 0x200 控制帧
- USART1 串口打印遥控器数据（115200，DMA 发送）
- 麦轮底盘：右摇杆 **上下=前后**、**左右=横移**（线性叠加，不含旋转）
- 每台电机的前后/左右方向系数可单独配置，适配镜像安装

## 硬件

| 器件 | 说明 |
|---|---|
| 主控 | STM32F407IG |
| 电机 | M3508 × 4 |
| 电调 | C620 × 4（ID 须分别设为 1/2/3/4） |
| 遥控器 | DJI DT7 + DR16 接收机 |
| 调试器 | ST-Link V2 |
| 其他 | CAN 收发器（如 TJA1050）、DBUS 反相转接 |

### 接线

| 功能 | 引脚 | 外设配置 |
|---|---|---|
| CAN 总线 | PD0 (RX) / PD1 (TX) | CAN1，**1 Mbps**，AF9 |
| 遥控器 DBUS | PC11 (RX) / PC10 (TX) | USART3，**100000**，9 位（8 数据位 + 偶校验） |
| 串口打印 | PB6 (TX) / PB7 (RX) | USART1，**115200**，8N1 |
| RGB LED | PH10 / PH11 / PH12 | 普通 GPIO |

> **CAN 总线**：两端各需 120Ω 终端电阻（实测 CAN_H–CAN_L ≈ 60Ω）。
> C620 内置**可切换**的终端电阻，总线上只保留**两端各一个**打开即可。
>
> **电调 ID**：C620 用自带的 `SET` 按键设置 —— 进入 ID 模式后再按 N 次即为 ID N，
> 静置 3 秒自动保存重启；两台电调 ID 相同会报错。详见 `CAN通讯指南.md`。

## 目录结构

```
├── Core/              # CubeMX 生成 + 用户代码（主要逻辑在 Core/Src/main.c）
├── Drivers/           # STM32F4xx HAL 库 + CMSIS
├── build/             # 编译产物（不上传 Git）
├── Makefile           # GNU Make 构建脚本
├── STM32F407xx_FLASH.ld
├── startup_stm32f407xx.s
├── stm32project.ioc   # CubeMX 工程文件
└── CAN通讯指南.md      # CAN 物理层 / 帧格式 / C620 说明（中文）
```

## 编译与烧录

### 1. 环境准备（macOS）

- **ARM GNU Toolchain**：从 [ARM 官网](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  下载 macOS 版 `.pkg` 安装（默认装到 `/Applications/ArmGNUToolchain/`），
  或 `brew install --cask gcc-arm-embedded`
- **OpenOCD**：`brew install openocd`
- VS Code 插件：**Cortex-Debug**

> ⚠️ **改路径**：`Makefile` 里有两条写死的绝对路径，工具链版本/安装位置不同会编译失败：
> - `PREFIX = /Applications/ArmGNUToolchain/<版本>/arm-none-eabi/bin/arm-none-eabi-`
> - 文件末尾 `C_INCLUDES += -I/opt/homebrew/Cellar/arm-none-eabi-gcc/<版本>/arm-none-eabi/include`
>
> 改成你机器上的实际路径即可。

### 2. 编译

```bash
make -j$(sysctl -n hw.ncpu)
```

产物在 `build/`：`stm32project.elf` / `.hex` / `.bin`

### 3. 烧录 / 调试

**VS Code**：按 `F5` 选 `STLink (Mac)`（自动先编译再下载，支持 Live Watch）。

**命令行**：

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/stm32project.elf verify reset exit"
```

## 使用

右摇杆两个方向分别控制前后和横移（摇杆满量程 ±660）：

| 通道 | 方向 | 目标转速 |
|---|---|---|
| `rc.ch[1]` | 右摇杆 **上下** | `fakeUseSpeed = ch[1] * 2`（前后，±1320 RPM） |
| `rc.ch[0]` | 右摇杆 **左右** | `fakeUseSpeedLR = ch[0] * 2`（横移，同强度） |

```c
fakeUseSpeed = rc_ctrl.rc.ch[1] * 2;   // 目标转速：±1320 RPM
```

主循环 1 kHz：读摇杆 → 算目标 → 速度环算电流 → CAN 发送。
每 100 ms 从 USART1 打印一行遥控器原始数据，可直接用串口助手（115200，8N1）查看。

## 代码结构（`Core/Src/main.c`）

| 函数 / 位置 | 作用 |
|---|---|
| `CAN1_Init()` | CAN1 初始化 + 过滤器（只放行 0x201~0x204） |
| `SpeedLoop_Fake()` | 速度环：算 4 路电流 + 限幅 ±2000 + 发送 0x200 |
| `HAL_CAN_RxFifo0MsgPendingCallback()` | 按 ID 解析反馈帧 → 更新 `Rx*4[]` 数组 |
| `HAL_UARTEx_RxEventCallback()` | 解析 DBUS 18 字节帧 → `rc_ctrl` |
| 主循环 | 1 kHz 速度环 + 100 ms 串口打印 |

**控制模型**：`电流 = 前后项 + 左右项 − 电机反馈转速`（比例控制，限幅 ±2000）。
`SpeedLoop_Fake()` 里每台电机有两个**方向系数**：前后系数、左右系数（各取 `+1` / `-1`）：

```c
fakeUseCurrent1 = (-1) * fakeUseSpeed + (-1) * fakeUseSpeedLR - RxSpeed4[0];
fakeUseCurrent2 = ( 1) * fakeUseSpeed + (-1) * fakeUseSpeedLR - RxSpeed4[1];
fakeUseCurrent3 = ( 1) * fakeUseSpeed + ( 1) * fakeUseSpeedLR - RxSpeed4[2];
fakeUseCurrent4 = (-1) * fakeUseSpeed + ( 1) * fakeUseSpeedLR - RxSpeed4[3];
```

> **左右系数怎么来的**：麦轮横移时左右两侧轮子的世界转向相反，所以
> `本体左右系数 = 前后系数 × 横移分配`（左侧 `+1`、右侧 `−1`）。若横移方向反了，
> 把 4 个左右系数整体取反即可。

> 哪几路取 `-1` **取决于你自己的装车方向**，每台车都不一样 —— 上电后给个小的目标转速实测，
> 方向不对就把那一行的符号反过来。上面是本工程当前的配置，仅供参考。
>
> **"装反"和"接反"是两回事**（现象不同，处理方式也不同）：
> - 电机**安装方向**相反 → 正常转、只是转向反了 → **改该行的方向系数**即可
> - 电机**三相线有两根接反** → 出现**飞车或剧烈振荡** → 必须**对调两根相线**，软件救不回来

## 调试提示

- 反馈数据在各全局数组里，用 Cortex-Debug 的 **Live Watch** 直接看（本工程已配 `samplesPerSecond: 4`）：
  - `RxCount4[0..3]` —— 每路电调的收帧计数（**不涨 = 这路没回帧**）
  - `RxSpeed4[]` 转速 / `RxCurrent4[]` 实际电流 / `RxTemp4[]` 温度 / `RxEcd4[]` 角度
  - `fakeUseCurrent1..4` —— 本机算出的输出电流（验证控制链路有没有算对）
- 电调不回帧时：先量 CAN_H–CAN_L 电阻（总线两端各 120Ω → 实测 ≈60Ω），
  再看电调指示灯是否绿灯常亮、ID 有没有设对
- `USER CODE BEGIN/END` 之外的代码是 CubeMX 生成的，**重新生成会被覆盖**；自己的逻辑请写在标记区内

## 参考

- `CAN通讯指南.md` —— CAN 物理层、标准帧格式、终端电阻、C620 说明
- 《RoboMaster C620 无刷电机调速器使用说明》
