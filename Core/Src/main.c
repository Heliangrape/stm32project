/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>     /* va_list / va_start / va_end   → 实现变参 printf */
#include <stdio.h>      /* vsnprintf                     → 格式化字符串   */
#include <math.h>       /* sinf / cosf                   → 地面系旋转     */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ============ 遥控器（DBUS）数据结构 ============ */
typedef struct
{
  struct
  {
    int16_t ch[5];          /* 4 个摇杆 + 1 个预留；已减去中位 1024 → 中位为 0 */
    char    s[2];           /* 左右拨杆：1=上(拨向上) 2=中 3=下 */
  } rc;
  struct
  {
    int16_t x, y, z;        /* 鼠标位移（x=左右，y=上下，z=滚轮） */
    uint8_t press_l;        /* 鼠标左键：1=按下 */
    uint8_t press_r;        /* 鼠标右键：1=按下 */
  } mouse;
  union
  {
    uint16_t v;             /* 键盘值（16 位位图，WATCH 里看这个最直观） */
    struct
    {
      uint16_t W : 1, S : 1, A : 1, D : 1, SHIFT : 1, CTRL : 1;
      uint16_t Q : 1, E : 1, R : 1, F : 1, G : 1, Z : 1, X : 1, C : 1, V : 1, B : 1;
    } bit;
  } key;
} RC_ctrl_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ============ 遥控器 DBUS 协议参数 ============ */
#define RC_FRAME_LENGTH    18     /* DBUS 一帧固定 18 字节 */
#define RC_CH_VALUE_OFFSET 1024   /* 摇杆中位（11 位数据的中值：1024） */
#define RC_TIMEOUT_MS      100    /* 超过 100ms 没收到帧 → 判定遥控器掉线 */

/* ============ printf 发送缓冲区大小 ============ */
#define PRINT_BUF_SIZE     256

/* ============ BMI088 陀螺仪 ============ */
/* 上电复位后的默认量程是 ±2000 dps（芯片默认值，不用写寄存器配置）
   → 1 LSB = 1/16.384 °/s = 0.0010653 rad/s */
#define BMI088_GYRO_SEN    0.0010653f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;          // CAN1 句柄
CAN_TxHeaderTypeDef TxHeader;     // 发送帧头
uint8_t TxData[8];                // 8 字节数据（一帧最多 8 字节）

/* --- 接收用：调试窗口（WATCH / Live Watch）直接看这几个变量 --- */
volatile uint32_t RxStdId;        // 收到的帧 ID（1号电机 = 0x201）
volatile uint8_t  RxData[8];      // 收到的 8 字节原始数据
volatile uint32_t RxCount;        // 累计收到帧数（用于验证是否在收）
volatile uint32_t RxLastTick;     // 最后一次收到反馈帧的时刻（HAL_GetTick 毫秒时间戳）

/* --- 拼好的物理量：调试窗口直接看，不用自己拼字节 --- */
volatile uint16_t RxEcd;          // 编码器机械角度 0~8191
volatile int16_t  RxSpeed;        // 转速 RPM（正=正转）
volatile int16_t  RxCurrent;      // 实际转矩电流
volatile uint8_t  RxTemp; 

/* --- 4 个电机的反馈：下标 [0]~[3] 对应 CAN ID 0x201~0x204 --- */
volatile uint16_t RxEcd4[4];      // 各电机角度
volatile int16_t  RxSpeed4[4];    // 各电机转速 RPM
volatile int16_t  RxCurrent4[4];  // 各电机实际电流
volatile uint8_t  RxTemp4[4];     // 各电机温度
volatile uint32_t RxCount4[4];    // 各电机收帧计数

volatile int16_t initAngle;        // 机械角度（0~8191）
volatile int16_t fakeUseCurrent  = 500;   // 老变量保留
volatile int16_t fakeUseCurrent1 = 0;     // 1 号电机输出电流
volatile int16_t fakeUseCurrent2 = 0;     // 2 号电机输出电流
volatile int16_t fakeUseCurrent3 = 0;     // 3 号电机输出电流
volatile int16_t fakeUseCurrent4 = 0;     // 4 号电机输出电流
volatile int16_t fakeUseSpeed   = 1500;
volatile int16_t fakeUseSpeedLR = 0;      // 左右平移目标转速（0号通道，同强度） 
volatile int16_t fakeUseOmega = 0;

/* --- BMI088 陀螺仪（SPI1：PB3=SCK / PB4=MISO / PA7=MOSI；CS：PA4=加速度计 / PB0=陀螺仪）--- */
volatile uint8_t BMI088_gyro_id;      /* 陀螺仪 ID（正常 0x0F，用来验证 SPI 通不通） */
volatile float   BMI088_gyro[3];      /* 三轴角速度 rad/s：X / Y / Z */
volatile float   gyro_offset[3];      /* 上电零漂标定值（rad/s） */
volatile float   INS_yaw;             /* ★ 绝对角度（rad，上电时刻 = 0，逆时针为正） */

/* --- 遥控器（DBUS）：WATCH 里直接看 rc_ctrl.rc.ch[0]~[3] --- */
uint8_t  rc_buf[RC_FRAME_LENGTH];     /* DMA 接收缓冲区：原始 18 字节（生数据） */
RC_ctrl_t rc_ctrl;                    /* 解码后的遥控器数据（熟数据，WATCH 看这个） */
volatile uint32_t rc_last_tick;       /* 最后一帧到达时刻（HAL_GetTick），用于掉线判断 */
uint32_t rc_print_tick;               /* printf 打印节流用的时间戳 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============ CAN1 实现（收发）：M3508 + C620 电调 ============ */

int MOTOR1_CURRENT = 500;      // 1号电机电流：正=正转，负=反转，±16384≈±20A

/* ============ 速度环（内环）参数 ============ */
#define SPD_MAX      800       // 转速上限：超过就断电流（保护电机）
#define KP_SPD       5        // 比例系数（整数）
#define KI_SPD       1         // 积分系数（整数；原 0.5f 在整数运算里会被截断成 0，故取 1）
#define CUR_LIMIT    3000      // 电流限幅（整数，±16384≈±20A，3000≈3.7A）

/*
 * CAN1 初始化，整个程序只调用一次
 * 波特率 = APB1 / (Prescaler x (1 + TimeSeg1 + TimeSeg2))
 *        = 42MHz / (3 x (1 + 11 + 2)) = 42MHz / 42 = 1Mbps
 * 注意：APB1 频率由时钟配置决定，换了晶振/时钟树后这里必须重新算
 */
static void CAN1_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* HAL宏：使能 CAN1 外设的时钟。STM32 外设默认断电，不开时钟则 CAN1 完全不工作 */
  __HAL_RCC_CAN1_CLK_ENABLE();                 // 开 CAN1 时钟

  /* ★ 硬复位 CAN 外设，让下面的 HAL_CAN_Init 从干净状态重新配置

     为什么需要：CubeMX 生成代码时给 main() 加了一个 MX_CAN1_Init()，它已经用
     Prescaler=16 / TSeg1=1TQ / TSeg2=1TQ（=875kbps）初始化过一次 CAN，并且
     把 CAN 拉到总线上处于 Normal 模式。这里如果不复位，会出现两个问题：
       ① BTR 可能沿用 875kbps → 电调完全收不到指令
       ② 前面若已在总线上出错进入 bus-off，HAL_CAN_Init 等 INAK 会超时（10ms）
          → hcan1.State 变成 ERROR → 之后 ConfigFilter / Start 会静默失败

     复位 + 把 State 手动置回 RESET 后，HAL_CAN_Init 会：
       - 重新调用 HAL_CAN_MspInit（再开一次时钟，无害）
       - 用下面 hcan1.Init 里的 1Mbps 参数重新写 BTR 寄存器         */
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();
  hcan1.State = HAL_CAN_STATE_RESET;

  /* HAL宏：使能 GPIOD 的时钟。PD0/PD1 属于 D 口，不开时钟就配置不了这两个引脚 */
  __HAL_RCC_GPIOD_CLK_ENABLE();                // 开 GPIOD 时钟（PD0/PD1 在 D 口）

  gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;    // PD0=RX, PD1=TX
  gpio.Mode      = GPIO_MODE_AF_PP;            // 复用推挽
  gpio.Pull      = GPIO_PULLUP;                // 上拉
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  // 高速
  gpio.Alternate = GPIO_AF9_CAN1;              // 复用功能 = CAN1 (AF9)
  /* HAL函数：按 gpio 结构体里的配置初始化 GPIO
     作用：把 PD0/PD1 从"普通IO"切换成"CAN1 复用功能"（AF9） */
  HAL_GPIO_Init(GPIOD, &gpio);

  hcan1.Instance           = CAN1;             // CAN1 外设
  hcan1.Init.Prescaler     = 3;                // 分频（APB1=42MHz，3x14=42 → 正好 1Mbps）
  hcan1.Init.Mode          = CAN_MODE_NORMAL;  // 正常模式
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;      // 同步跳转宽度
  hcan1.Init.TimeSeg1      = CAN_BS1_11TQ;     // 段1
  hcan1.Init.TimeSeg2      = CAN_BS2_2TQ;      // 段2（1+11+2=14 → 3x14=42 → 1Mbps，采样点 85.7%）
  hcan1.Init.TimeTriggeredMode    = DISABLE;
  hcan1.Init.AutoBusOff           = ENABLE;
  hcan1.Init.AutoWakeUp           = DISABLE;
  hcan1.Init.AutoRetransmission   = ENABLE;
  hcan1.Init.ReceiveFifoLocked    = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  /* HAL函数：用 hcan1.Init 里的参数初始化 CAN1 控制器
     内部动作：算波特率、把分频/时间段/工作模式写进 CAN 寄存器 */
  HAL_CAN_Init(&hcan1);

  /*
   * 过滤器（ID_LIST 列表模式）：只放行 0x201~0x204 四个电调反馈帧
   * 注意：16 位列表模式下这 4 个字段都是"ID 存储位"，标准 ID 需左移 5 位
   */
  CAN_FilterTypeDef filter = {0};
  filter.FilterMode           = CAN_FILTERMODE_IDLIST;   // 列表模式：只收列出的 ID
  filter.FilterScale          = CAN_FILTERSCALE_16BIT;   // 16 位 → 一组装 4 个 ID
  filter.FilterIdHigh         = 0x201 << 5;              // 1号电机
  filter.FilterIdLow          = 0x202 << 5;              // 2号电机
  filter.FilterMaskIdHigh     = 0x203 << 5;              // 3号电机
  filter.FilterMaskIdLow      = 0x204 << 5;              // 4号电机
  filter.FilterBank           = 0;                       // CAN1 只能用 0~13
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;        // 送进 FIFO0
  filter.FilterActivation     = ENABLE;
  /* HAL函数：把上面的 filter 配置写进硬件过滤器
     作用：决定总线上哪些 ID 的帧能被收进来（不匹配的硬件直接丢弃） */
  HAL_CAN_ConfigFilter(&hcan1, &filter);

  /* HAL函数：让 CAN 离开"初始化模式"、进入正常工作状态
     不调用的话，后面所有发送/接收都会直接失败 */
  HAL_CAN_Start(&hcan1);                       // 启动 CAN（不启动发送会失败）

  /* 打开接收中断：FIFO0 收到匹配帧 → 触发 CAN1_RX0 中断 → 调用回调 */
  /* HAL函数：打开"FIFO0 收到新消息"这个中断使能
     打开后：FIFO0 一有新帧，HAL 就会自动调用接收回调函数 */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* HAL函数：设置 CAN1_RX0 中断的优先级（抢占优先级0、子优先级0）
     数值越小优先级越高，0 是最高优先级 */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);   // 优先级
  /* HAL函数：在 NVIC（中断控制器）里真正"放行"这个中断
     不调用的话，即使 FIFO 有数据，CPU 也不会跳进中断函数 */
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);           // 放行中断
}

/*
 * 发送电流指令给 1~4 号电调（CAN ID = 0x200）
 * cur1~cur4：电流给定，正=正转，负=反转，±16384 对应 ±20A
 */
static void CAN_cmd_chassis(int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
  uint32_t TxMailbox;

  TxHeader.StdId = 0x200;         // CAN ID
  TxHeader.IDE   = CAN_ID_STD;    // 标准帧
  TxHeader.RTR   = CAN_RTR_DATA;  // 数据帧
  TxHeader.DLC   = 8;             // 8 字节

  TxData[0] = cur1 >> 8;   TxData[1] = cur1 & 0xFF;   // 电机1
  TxData[2] = cur2 >> 8;   TxData[3] = cur2 & 0xFF;   // 电机2
  TxData[4] = cur3 >> 8;   TxData[5] = cur3 & 0xFF;   // 电机3
  TxData[6] = cur4 >> 8;   TxData[7] = cur4 & 0xFF;   // 电机4

  /* HAL函数：把这一帧放进发送邮箱并请求发送
     参数依次：CAN句柄、帧头（含ID/长度）、8字节数据、回填"用了哪个邮箱"
     注意：返回 HAL_OK 只表示"投递成功"，不代表电调收到了（要等对方回 ACK） */
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

/*
 * 速度环（内环）：让电机稳定在 SPD_TARGET 转速
 * 输出 = PI(目标转速 - 实际转速) → 电流
 * 两道保护：① 反馈超时（50ms 收不到新帧）→ 断输出
 *           ② 转速超过 SPD_MAX            → 断输出
 */

static void SpeedLoop_Fake(void)
{
  /* 目标转速 = 前后项(fakeUseSpeed) + 左右项(fakeUseSpeedLR)，各自带自己的方向系数
     麦轮横移时左右两侧轮子的世界转向相反 → 本体系数 = 前后系数 × 横移分配 */
  /* 每行开头的方向系数：+1 = 与 1 号电机同向；-1 = 镜像安装（本体必须反转才跟大家同向）
     —— 哪个电机转反了，就改那一行的符号
     注意：RxSpeed4[i] 是"电机本体"的转速，不乘方向系数 */
  fakeUseCurrent1 = (-1) * fakeUseSpeed + (-1) * fakeUseSpeedLR + (1) * fakeUseOmega - RxSpeed4[0];   /* 1 号：本体反向 */
  fakeUseCurrent2 = ( 1) * fakeUseSpeed + (-1) * fakeUseSpeedLR + (1) * fakeUseOmega - RxSpeed4[1];   /* 2 号：本体反向 */
  fakeUseCurrent3 = ( 1) * fakeUseSpeed + ( 1) * fakeUseSpeedLR + (1) * fakeUseOmega - RxSpeed4[2];   /* 3 号：正常 */
  fakeUseCurrent4 = (-1) * fakeUseSpeed + ( 1) * fakeUseSpeedLR + (1) * fakeUseOmega - RxSpeed4[3];   /* 4 号：正常 */

  if (fakeUseCurrent1 > 2000) 
  {
    fakeUseCurrent1 = 2000;
  }
  if (fakeUseCurrent1 < -2000)
  {
    fakeUseCurrent1 = -2000;
  }
  /* 2/3/4 号电机同样的限幅 */
  if (fakeUseCurrent2 >  2000) fakeUseCurrent2 =  2000;
  if (fakeUseCurrent2 < -2000) fakeUseCurrent2 = -2000;
  if (fakeUseCurrent3 >  2000) fakeUseCurrent3 =  2000;
  if (fakeUseCurrent3 < -2000) fakeUseCurrent3 = -2000;
  if (fakeUseCurrent4 >  2000) fakeUseCurrent4 =  2000;
  if (fakeUseCurrent4 < -2000) fakeUseCurrent4 = -2000;

  CAN_cmd_chassis(fakeUseCurrent1, fakeUseCurrent2, fakeUseCurrent3, fakeUseCurrent4);
}

/* ============ 位置环（外环，只用 P 和 D）============ */
#define HALF_TURN_CNT   78656   /* 输出轴转半圈 = 转子 9.6 圈 × 8192 计数
                                   （若指"转子半圈"，改成 4096） */
#define KP_ANG_SHIFT    4       /* P 系数 = 1/8（数值越大刚度越强） */
#define KD_ANG_SHIFT    0       /* D 系数 = 1（数值越大阻尼越弱） */
#define SPD_CMD_LIMIT   1500    /* 外层输出的目标转速限幅（RPM） */

volatile int32_t errPos, spdCmd;
static uint16_t lastEcd  = 0;   /* 上一帧的单圈角度 */
static int32_t  totalCnt = 0;   /* 多圈累计角度（转子计数） */
static int32_t  startCnt = 0;   /* 起点：第一次调用时的累计角度 */
static uint8_t  started  = 0;   /* 是否已记起点 */

static void Rotate_Fake(int32_t targetAngle){
  
  

  /* ① 单圈角度 0~8191 累加成"多圈位移"
        差值转 int16_t：跨越 0/8191 边界时自动得到 ±小增量，天然处理回绕 */
  int32_t dCnt = (int32_t)RxEcd - (int32_t)lastEcd;   /* 本帧增量，范围 -8191 ~ +8191 */
  if (dCnt >  4096) dCnt -= 8192;                     /* 正向跨过一圈边界（8191→0） */
  if (dCnt < -4096) dCnt += 8192;                     /* 反向跨过一圈边界（0→8191） */
  totalCnt += dCnt;
  lastEcd   = RxEcd;

  if (!started)                     /* 第一次调用：记下起点 */
  {
    startCnt = totalCnt;
    started  = 1;
  }

  /* ② 位置误差（计数）：正 = 还差多少到目标
        targetAngle = "从起点还要转多少"（不是转到哪） */
  errPos = (startCnt + targetAngle) - totalCnt;

  /* ③ PD：P 项 = 误差右移（÷128）当目标转速；D 项 = 用实测转速当阻尼 */
  spdCmd = (errPos >> KP_ANG_SHIFT) - (RxSpeed >> KD_ANG_SHIFT);

  /* ④ 目标转速限幅 */
  if (spdCmd > SPD_CMD_LIMIT) spdCmd = SPD_CMD_LIMIT;
  if (spdCmd < -SPD_CMD_LIMIT) spdCmd = -SPD_CMD_LIMIT;

  /* ⑤ 交给内层执行（内层把它当成 SPD_TARGET 去追） */
  fakeUseSpeed = (int16_t)spdCmd;
  SpeedLoop_Fake();
}


/*
 * 接收回调：CAN1 FIFO0 收到匹配帧时由 HAL 自动调用（运行在中断里）
 * 功能：把 8 字节原始数据搬到全局变量，供调试窗口查看
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t tmp[8];

  /* HAL函数：从 FIFO0 取出 1 帧数据
     帧头信息写入 RxHeader（ID、长度等），8 字节数据写入 tmp，成功返回 HAL_OK */
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, tmp) == HAL_OK)
  {
    /* ① 刷新"还活着"的时间戳：速度环用它判断反馈有没有断线 */
    /* HAL函数：读取系统已运行的毫秒数（由 SysTick 中断每 1ms 累加一次）
       这里用于速度环的"反馈超时"判断 */
    RxLastTick = HAL_GetTick();

    /* ② 保存帧 ID 和原始 8 字节（调试器里可以直接看） */
    RxStdId = RxHeader.StdId;
    /* 只处理 0x201~0x204 四个电调反馈帧；idx = 0~3 对应电机 1~4 */
    if (RxStdId >= 0x201 && RxStdId <= 0x204)                           
      {
        int idx = (int)(RxStdId - 0x201);                // 0~3 对应电机 1~4
        for (int i = 0; i < 8; i++) RxData[i] = tmp[i];  // 拷贝 8 字节（原始）
        RxCount++;                                       // 帧数 +1
      
      /* ③ 心跳灯：每收到 500 帧就翻转一次绿灯
            电调回传频率是 1kHz → 约 2Hz 闪烁（肉眼明显）
            作用：不用调试器也能判断接收死活
                  绿灯稳定闪烁 = 接收正常 ✅
                  绿灯不动     = 接收断了 ❌                        */
      if ((RxCount % 2000) == 1)   // 4 个电机共约 4kHz 帧率 → 2000 帧仍约 2Hz
      {
        /* HAL函数：翻转引脚电平（当前是高就变低，是低就变高） */
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
      }

      /* 按电调反馈帧格式拼出物理量：高八位在前，低八位在后 */
      /* 每个电机只填自己那一格：idx 0~3 ←→ 电机 1~4 */
      RxEcd4[idx]     = (uint16_t)((tmp[0] << 8) | tmp[1]);  // 编码器角度 0~8191
      RxSpeed4[idx]   = (int16_t) ((tmp[2] << 8) | tmp[3]);  // 转速 RPM
      RxCurrent4[idx] = (int16_t) ((tmp[4] << 8) | tmp[5]);  // 实际电流
      RxTemp4[idx]    = tmp[6];                              // 温度
      RxCount4[idx]++;                                       // 该电机累计帧数

      /* 1 号电机的值同步留在老变量里（调试习惯 + Rotate_Fake 在用） */
      RxEcd     = RxEcd4[0];
      RxSpeed   = RxSpeed4[0];
      RxCurrent = RxCurrent4[0];
      RxTemp    = RxTemp4[0];          
                         
    }
  }
  //aaa
}
/* ==================================================================
 * 遥控器（DBUS）部分
 * 数据流：USART3 硬件 → DMA 搬运 → rc_buf[18] →(解码) rc_ctrl → 你使用
 * ================================================================== */

/*
 * 位拼接解码：把 18 字节生数据按 DBUS 协议还原成物理量
 * DBUS 把多个 11 位的通道值连续打包，所以要用"移位 + 或 + 掩码"跨字节拼
 * 0x07ff = 二进制 11 个 1 → 作用就是截取 11 位
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
  if (sbus_buf == NULL || rc_ctrl == NULL)
  {
    return;
  }

  /* ① 4 个摇杆通道：每通道 11 位，可能横跨 2~3 个字节 */
  rc_ctrl->rc.ch[0] =  (sbus_buf[0]       | (sbus_buf[1] << 8)) & 0x07ff;                       /* 右摇杆 左右 */
  rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;                       /* 右摇杆 上下 */
  rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff; /* 左摇杆 上下 */
  rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;                       /* 左摇杆 左右 */

  /* ② 左右拨杆：各 2 位（1=向上 2=中间 3=向下），都挤在第 6 个字节里 */
  rc_ctrl->rc.s[0]  =  (sbus_buf[5] >> 4) & 0x0003;
  rc_ctrl->rc.s[1]  = ((sbus_buf[5] >> 4) & 0x000C) >> 2;

  /* ③ 鼠标：x/y/z 各 16 位，左右键各占 1 个字节 */
  rc_ctrl->mouse.x       =  sbus_buf[6]  | (sbus_buf[7]  << 8);
  rc_ctrl->mouse.y       =  sbus_buf[8]  | (sbus_buf[9]  << 8);
  rc_ctrl->mouse.z       =  sbus_buf[10] | (sbus_buf[11] << 8);
  rc_ctrl->mouse.press_l =  sbus_buf[12];
  rc_ctrl->mouse.press_r =  sbus_buf[13];

  /* ④ 键盘（16 位位图：哪一位是 1 就表示哪个键按下了） */
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);

  /* ⑤ 第 5 通道（预留，遥控器一般不用） */
  rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);

  /* ⑥ 摇杆原始值是 364~1684，减掉中位 1024 → 变成 -1024~+1023，中位为 0，用起来最方便 */
  rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/*
 * 遥控器在线判断
 * 返回 1 = 掉线（超过 RC_TIMEOUT_MS 毫秒没收到任何一帧）
 * 用途：掉线时必须让电机停住，否则机器人会带着最后一条指令一直跑
 */
uint8_t RC_is_offline(void)
{
  if (rc_last_tick == 0)     /* 上电后一帧都没收到过 */
  {
    return 1;
  }
  return (HAL_GetTick() - rc_last_tick > RC_TIMEOUT_MS) ? 1 : 0;
}

/*
 * DMA 版 printf：把格式化好的字符串用 USART1 的 DMA 发到电脑串口助手
 * 与标准 printf 的区别：DMA 在后台搬数据，CPU 只做"格式化"这一步
 */
void usart_printf(const char *fmt, ...)
{
  static uint8_t tx_buf[PRINT_BUF_SIZE];   /* static：DMA 在后台读它，不能放栈上 */
  va_list ap;
  int len;
  uint32_t t0;

  /*
   * 先等上一次 DMA 发完，再往 tx_buf 写新内容
   * 不等的话会把"正在发送中的缓冲"覆盖掉 → 输出乱码
   * 加超时保护：串口万一异常也不会在这儿死等
   */
  t0 = HAL_GetTick();
  while (huart1.gState != HAL_UART_STATE_READY)
  {
    if (HAL_GetTick() - t0 > 10)
    {
      return;
    }
  }

  /* 变参三步走：va_start 取参数 → vsnprintf 格式化 → va_end 收尾 */
  va_start(ap, fmt);
  len = vsnprintf((char *)tx_buf, PRINT_BUF_SIZE, fmt, ap);
  va_end(ap);

  if (len <= 0)
  {
    return;
  }
  if (len > PRINT_BUF_SIZE - 1)
  {
    len = PRINT_BUF_SIZE - 1;      /* 内容超出缓冲时截断，防止越界发送 */
  }

  /* HAL函数：启动 DMA 发送（非阻塞，立刻返回，数据由 DMA 后台搬走） */
  HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)len);
}

/*
 * 遥控器接收完成回调（HAL 的弱函数，这里重写它）
 * HAL 已经替我们做完：开 IDLE 中断、进 USART3_IRQHandler、算出收到多少字节、
 * 关 DMA、重启 DMA —— 我们只拿到"这一趟收了多少字节(Size)"这一个结果
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  /* 半传输事件：缓冲区只收到一半，不是"一帧结束"，忽略 */
  if (huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  /* 必须正好是完整一帧（18 字节）才解码；长度不对说明是噪声或中间丢了字节 */
  if (Size == RC_FRAME_LENGTH)
  {
    sbus_to_rc(rc_buf, &rc_ctrl);      /* 生数据 → 物理量 */
    rc_last_tick = HAL_GetTick();      /* 刷新"遥控器还活着"的时间戳 */
  }

  /* 重新武装：再挂一次 DMA 接收，等下一帧（漏了这句就只能收到一帧） */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_buf, RC_FRAME_LENGTH);

  /* 关掉"半传输中断"：让 HAL 只在"空闲/缓冲区满"时才回调，少进几次中断 */
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/*
 * UART 错误回调：HAL 遇到校验错/帧错/溢出时会进这里，
 * 而且它**已经自动停止了接收**（UART_EndRxTransfer）—— 所以我们的任务就是
 * 清标志 + 把 DMA 接收重新挂上；不做的话，遇到一次错误就永久收不到数据了
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);          /* 读 SR + DR，清掉错误标志 */
  huart->ErrorCode = HAL_UART_ERROR_NONE;   /* 手动清错误码 */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_buf, RC_FRAME_LENGTH);   /* 重新武装 */
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/* ==========================================================================
 * BMI088 陀螺仪驱动（最小版：只读角速度，不做姿态融合）
 * 接线：SPI1 = PB3(SCK) / PB4(MISO) / PA7(MOSI)
 *       CS   = PA4(加速度计 CS1_ACCEL) / PB0(陀螺仪 CS1_GYRO)，都是低电平选中
 * 寄存器：0x00 = ID（陀螺仪应读 0x0F）
 *         0x02~0x07 = X/Y/Z 三轴角速度，每轴 16 位（低字节在前）
 * ========================================================================== */

#define ACC_CS_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ACC_CS_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define GYRO_CS_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define GYRO_CS_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

/*
 * SPI 全双工收发 1 字节：发出去的同时就在收（SPI 的物理特性决定的）
 * BMI088 用 SPI 模式 3（CPOL=1, CPHA=1），和 MX_SPI1_Init 里的配置一致
 */
static uint8_t BMI088_rw(uint8_t tx)
{
  uint8_t rx = 0;
  /* HAL函数：SPI 全双工收发
     参数：SPI句柄、发送缓冲、接收缓冲、字节数、超时(ms) */
  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);
  return rx;
}

/*
 * 读陀螺仪的连续 len 个寄存器
 * BMI088 的 SPI 协议规定：读操作的地址最高位要置 1，写操作清 0
 * 整个过程 CS 必须一直保持低电平，读完才拉高（拉高 = 一帧结束）
 */
static void BMI088_gyro_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  reg |= 0x80;                  /* 最高位 = 1 → 读操作 */
  GYRO_CS_L();
  BMI088_rw(reg);               /* 先发寄存器地址 */
  for (i = 0; i < len; i++)
  {
    buf[i] = BMI088_rw(0x00);   /* 之后每发一个空字节，就收回一个数据字节 */
  }
  GYRO_CS_H();
}

/*
 * 读三轴角速度 → rad/s
 * 寄存器 0x02~0x07 依次是 X_L, X_M, Y_L, Y_M, Z_L, Z_M（低字节在前）
 * 注意：教材 P182 的文字写成"RATE_Z_MSB 到 RATE_X_LSB"是写反的，
 *       以 P183 的代码为准（X 在前），这里和 P183 保持一致
 */
static void BMI088_read_gyro(float g[3])
{
  uint8_t buf[6];
  BMI088_gyro_read_regs(0x02, buf, 6);
  g[0] = (float)(int16_t)((buf[1] << 8) | buf[0]) * BMI088_GYRO_SEN;   /* X 轴 */
  g[1] = (float)(int16_t)((buf[3] << 8) | buf[2]) * BMI088_GYRO_SEN;   /* Y 轴 */
  g[2] = (float)(int16_t)((buf[5] << 8) | buf[4]) * BMI088_GYRO_SEN;   /* Z 轴 */
}

/*
 * 读陀螺仪 ID（寄存器 0x00），正常返回 0x0F
 * 作用：上电自检 —— 读不到 0x0F 说明 SPI 接线/片选/模式有问题，不用瞎猜
 */
static uint8_t BMI088_read_gyro_id(void)
{
  uint8_t id = 0;
  BMI088_gyro_read_regs(0x00, &id, 1);
  return id;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* ★ CAN1 必须最先初始化！
     原因：CubeMX 生成的 MX_CAN1_Init() 已经用 875kbps 把 CAN 拉到总线上了，
     如果在这里先做 BMI088 的 1.3 秒零漂标定，CAN 就会在错误波特率下污染 1Mbps
     总线 1.3 秒 → 4 个 C620 电调全部出错掉线（现象：RxCount=0、电机不动）。
     所以 CAN1_Init() 必须紧跟在外设初始化之后、任何耗时操作之前调用。 */
  CAN1_Init();                      /* CAN1 初始化：整个程序只做一次 */

  /* ---- BMI088 片选上电先拉高 ----
     CubeMX 把 PA4/PB0 的初始电平配成了低电平，而 CS 是低电平选中，
     不修的话上电瞬间加速度计和陀螺仪会同时被选中，两个芯片一起抢 MISO 总线 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   /* 加速度计 CS 拉高 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   /* 陀螺仪   CS 拉高 */

  /* ---- BMI088 上电自检：读陀螺仪 ID，正常应该是 0x0F ---- */
  BMI088_gyro_id = BMI088_read_gyro_id();
  usart_printf("[BMI088] gyro_id = 0x%02X (expect 0x0F)\r\n", BMI088_gyro_id);

  /* ---- 零漂标定：连续采 1000 次求平均（约 1.3 秒）
     陀螺仪静止时也会输出几十 LSB 的零偏，不减掉的话积分出的角度会一直漂
     ★ 这段时间小车必须静止不动！ */
  {
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float g[3];
    int i;
    for (i = 0; i < 1000; i++)
    {
      BMI088_read_gyro(g);
      sum[0] += g[0];
      sum[1] += g[1];
      sum[2] += g[2];
      HAL_Delay(1);
    }
    gyro_offset[0] = sum[0] / 1000.0f;
    gyro_offset[1] = sum[1] / 1000.0f;
    gyro_offset[2] = sum[2] / 1000.0f;
  }
  /* 用整数打印（放大 10 万倍），因为 usart_printf 没开浮点打印支持 */
  usart_printf("[BMI088] offset(1e5) = %d %d %d rad/s\r\n",
               (int)(gyro_offset[0] * 100000.0f),
               (int)(gyro_offset[1] * 100000.0f),
               (int)(gyro_offset[2] * 100000.0f));

  /* HAL函数：启动 USART3 的"DMA + 空闲中断"接收
     作用：DMA 在后台把遥控器发来的 18 字节搬进 rc_buf，
           收满一帧（或总线空闲）时 HAL 自动调用 HAL_UARTEx_RxEventCallback */
  __HAL_UART_CLEAR_OREFLAG(&huart3);   /* 先清掉上电瞬间可能残留的错误标志，
                                          否则启动时若已有挂起错误，接收会被立刻中止 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_buf, RC_FRAME_LENGTH);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);   /* 只要"收完一帧"，不要"收了一半"的回调 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
         // 速度环：目标 500 RPM，PI 输出电流（带双保护）
    /* HAL函数：阻塞式延时 1 毫秒（靠 SysTick 中断计时）
       这里的作用：构成固定 1kHz 的控制周期（PID 必须等间隔执行） */
    HAL_Delay(1); 
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

    /* ---- BMI088 陀螺仪：读角速度 + 积分成绝对角度 ---- */
    {
      float g[3];
      BMI088_read_gyro(g);
      BMI088_gyro[0] = g[0];      /* 存进全局变量，WATCH 里可以直接看 */
      BMI088_gyro[1] = g[1];
      BMI088_gyro[2] = g[2];
      /* 绝对角度 = Z 轴角速度积分（先减掉零漂），dt = 1ms
         注意：256 分频下每次读约 200us，主循环实际约 1.2ms；
               若转 90° 后 INS_yaw 明显小于 1.57，就把 0.001f 改成 0.0012f */
      INS_yaw += (g[2] - gyro_offset[2]) * 0.001f;
      /* 归一化到 ±π：每 1ms 的增量极小，两个 if 足够，不需要 while */
      if (INS_yaw >  3.14159265f) INS_yaw -= 6.28318531f;
      if (INS_yaw < -3.14159265f) INS_yaw += 6.28318531f;
    } 
    /* ---- 摇杆 → 地面系 → 旋转回车体系（yaw 补偿）----
       摇杆推的方向 = 车相对"地面"要走的方向（上电时刻为参考方向）
       用 INS_yaw 把地面系指令旋转回车体系，再交给轮子
       yaw 逆时针为正：vx' = vx·cosθ + vy·sinθ
                       vy' = -vx·sinθ + vy·cosθ */
    {
      float vx = rc_ctrl.rc.ch[1] * 3;      /* 摇杆前后（地面系） */
      float vy = rc_ctrl.rc.ch[0] * 3;      /* 摇杆左右（地面系） */
      float c  = cosf(INS_yaw);
      float s  = sinf(INS_yaw);
      fakeUseSpeed   = (int16_t)( vx * c + vy * s);
      fakeUseSpeedLR = (int16_t)(-vx * s + vy * c);
    }
    fakeUseOmega = rc_ctrl.rc.ch[2] * 3;    /* 自转照旧：本来就该是车体系，不参与旋转 */
    SpeedLoop_Fake();      // 1kHz 控制周期

    /* 每 100ms 往电脑串口助手打印一次遥控器数据（用 DMA 发送，不占 CPU） */
    if (HAL_GetTick() - rc_print_tick >= 100)
    {
      rc_print_tick = HAL_GetTick();

      if (RC_is_offline())
      {
        usart_printf("[RC] OFFLINE! no frame for > %d ms\r\n", RC_TIMEOUT_MS);
      }
      else
      {
        usart_printf("[RC] ch0=%5d ch1=%5d ch2=%5d ch3=%5d | s0=%d s1=%d | mouse %4d %4d %4d L%d R%d | key=0x%04X\r\n",
                     rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3],
                     rc_ctrl.rc.s[0], rc_ctrl.rc.s[1],
                     rc_ctrl.mouse.x, rc_ctrl.mouse.y, rc_ctrl.mouse.z,
                     rc_ctrl.mouse.press_l, rc_ctrl.mouse.press_r,
                     rc_ctrl.key.v);
      }
    }
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
