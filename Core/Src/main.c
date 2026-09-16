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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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

volatile int16_t initAngle;        // 机械角度（0~8191）
volatile int16_t fakeUseCurrent = 500;
volatile int16_t fakeUseSpeed   = 1500; 

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
  fakeUseCurrent = fakeUseSpeed - RxSpeed;

  if (fakeUseCurrent > 2000) 
  {
    fakeUseCurrent = 2000;
  }
  if (fakeUseCurrent < -2000)
  {
    fakeUseCurrent = -2000;
  }
  CAN_cmd_chassis(fakeUseCurrent, 0, 0, 0);
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
    if (RxStdId == 0x201)                           
      {
        for (int i = 0; i < 8; i++) RxData[i] = tmp[i];  // 拷贝 8 字节（原始）
        RxCount++;                                       // 帧数 +1
      
      /* ③ 心跳灯：每收到 500 帧就翻转一次绿灯
            电调回传频率是 1kHz → 约 2Hz 闪烁（肉眼明显）
            作用：不用调试器也能判断接收死活
                  绿灯稳定闪烁 = 接收正常 ✅
                  绿灯不动     = 接收断了 ❌                        */
      if ((RxCount % 500) == 1)
      {
        /* HAL函数：翻转引脚电平（当前是高就变低，是低就变高） */
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
      }

      /* 按电调反馈帧格式拼出物理量：高八位在前，低八位在后 */
      RxEcd     = (uint16_t)((RxData[0] << 8) | RxData[1]);  // 编码器角度 0~8191
      RxSpeed   = (int16_t) ((RxData[2] << 8) | RxData[3]);  // 转速 RPM
      RxCurrent = (int16_t) ((RxData[4] << 8) | RxData[5]);  // 实际电流
      RxTemp    = RxData[6];          
      Rotate_Fake(400000);                    
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
  /* USER CODE BEGIN 2 */
  CAN1_Init();                      /* CAN1 初始化：整个程序只做一次 */

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
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);       // 1kHz 控制周期

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
  huart3.Init.WordLength = UART_WORDLENGTH_9B;   /* DBUS = 8 数据位 + 偶校验 = 11 位帧
                                                    （M=1 才是"8 数据位+校验"，M=0 只有 7 数据位） */
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

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
