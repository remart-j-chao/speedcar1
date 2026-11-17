/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * 本文件内含BMI270陀螺仪的使用示例
 * 本文件内含无刷电桥的启动初始化的使用示例
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 陀螺仪原始六轴打印。调试用，会引入约 5.4 kB 浮点 printf，
   MDK-Lite 32 kB 链接上限容易爆，平时必须保持 0。 */
#define GYRO_DEBUG 0
/* 负压风扇。开发阶段保持 0：置 1 后启动流程会阻塞约 2 s 等负压建立。 */
#define IF_start_Brushless_motor 0

/* ==========================================================================
   循迹控制：方法一（三环串级）
   --------------------------------------------------------------------------
   第 1 环  转向环 PD      偏差 -> 目标角速度        单位 dps
   第 2 环  角速度环 PID   角速度误差 -> 差速 R      单位 m/s
   第 3 环  速度环 PI      每轮速度误差 -> CCR

       偏差 err ──►[PD]──► w_ref ──►[PID]──► R ──► 左轮目标 = V - R
                        ▲             ▲         右轮目标 = V + R
                        │             │                │
                     陀螺仪 Z 轴 ──────┘                ▼
                                                     [PI] ──► CCR
   注意符号约定：
     R > 0 时左轮慢、右轮快，车体**向左**转。
     err > 0 表示线在车体右侧，需要向右转，所以 w_ref 取负。
   参数全部来自实车实测，不是凭感觉给的（见各处注释）。
   ========================================================================== */

/* ---------------- 时基 ---------------- */
/* TIM2 = 1 kHz，再分频到 10 ms。10 ms 对 12 路光电管来说够快，
   也不会让串口打印把控制周期挤歪。 */
#define CTRL_PERIOD_MS          10U
#define CTRL_DT                 ((float)CTRL_PERIOD_MS / 1000.0f)

/* ---------------- 光电管 ---------------- */
/* 判定"压线"的电平。实测模块压白线时输出高电平。 */
#define LINE_ACTIVE_LEVEL       1U
/* 偏差内部用"千分之一权重"，方便整数打印。 */
#define LINE_ERROR_SCALE        1000.0f
/* 采样周期 */
#define LINE_SAMPLE_MS          10U
/* 光电管一档（相邻两路切换）对应的量化步长，约 1.33 个权重。
   微分用它做窗口平均，避免单拍差商打出尖峰。 */
#define LINE_D_WINDOW           3U      /* 3 × 10 ms = 30 ms 窗口 */
#define LINE_D_LPF_ALPHA        0.7f

/* ---------------- 编码器（实测标定） ---------------- */
#define ENC_COUNTS_PER_METER    37385.0f  /* 推 1 m 读到的计数 */
#define ENC_L_SIGN              1
#define ENC_R_SIGN              (-1)
#define ENC_AVG_SAMPLES         2U        /* 20 ms 滑窗，兼顾平滑与延迟 */

/* ---------------- 电机（实测两点拟合） ---------------- */
/* TIM1 的 ARR = 7199，占空比 = CCR / 7200。
   实测转速对 CCR 是仿射关系：CCR = MOTOR_CCR_ZERO + MOTOR_CCR_PER_MPS × v
     cz = 170（0.4 m/s 时 ccrL≈294，0.8 m/s 时 ccrL≈424，两点拟合得到） */
#define MOTOR_CCR_ZERO          170.0f
#define MOTOR_CCR_PER_MPS       325.0f
#define MOTOR_CCR_MAX           1000U     /* 约 14% 占空比，留足余量 */
/* 实测 CCR 288 时反转轮完全没有扭矩，被地面拖着正转。
   按转向量成比例补 0~340 的扭矩，直道不补，急弯补满。 */
#define MOTOR_CCR_HOLD          340.0f
#define MOTOR_HOLD_REF_MPS      0.35f
/* 正/反转切换迟滞，避免目标在 0 附近时 DIR 每 10 ms 翻一次造成抖动 */
#define MOTOR_REV_ENTER_MPS     0.030f
#define MOTOR_REV_LEAVE_MPS     0.010f
/* 实测 L 轮正转 = DIR 低，R 轮正转 = DIR 高 */
#define L_DIR_FORWARD           GPIO_PIN_RESET
#define L_DIR_REVERSE           GPIO_PIN_SET
#define R_DIR_FORWARD           GPIO_PIN_SET
#define R_DIR_REVERSE           GPIO_PIN_RESET

/* ---------------- 车速 ---------------- */
#define SPEED_TARGET_MPS        0.50f
#define SPEED_LIMIT_MPS         1.20f
#define SPEED_ACCEL_MPS2        1.5f

/* ---------------- 第 1 环：转向环 PD ---------------- */
/* 偏差范围 ±6，且光电管在最外侧饱和在 6。
   要让饱和（err=6）时角速度指令真的吃到上限 400 dps，增益必须 >= 400/6=66.7，
   否则急弯永远差一口气（环岛入口进不去就是这个原因）。取 70。 */
#define W_KP                    70.0f    /* dps / 权重 */
#define W_KD                    0.60f    /* dps / (权重/s) */
#define W_LIMIT_DPS             400.0f   /* 目标角速度上限 */

/* ---------------- 第 2 环：角速度环 PID ---------------- */
/* 实测对象增益 K ≈ 650 dps 每 1 m/s 差速（等价轮距 ≈ 0.177 m）。
   前馈取 1/K 就能让实测角速度紧跟目标 */
#define YAW_FF                  0.00154f /* m/s 每 dps */
/* 闭环增益 b = K × Kp。b 取 0.6 左右：既跟得上又不振荡 */
#define YAW_KP                  0.0009f  /* m/s 每 dps */
#define YAW_KI                  0.0015f  /* m/s 每 (dps·s) */
#define YAW_KD                  0.0f     /* 默认不用，需要时再给 */
#define YAW_I_LIMIT             0.15f    /* 积分输出限幅，m/s */
#define YAW_D_LPF_ALPHA         0.5f
/* 角速度死区：静止时 gyro 仍有 ±0.5 dps 的噪声，不滤掉会被前馈放大 */
#define YAW_DEADBAND_DPS        1.2f
#define YAW_LPF_ALPHA           0.4f     /* 角速度一阶低通 */

/* ---------------- 差速限幅 ---------------- */
/* 实测内侧轮最低到 -0.25 m/s 还能保持扭矩，再低就纯拖拽。
   配合这个差值，最小转弯半径约 6 cm。 */
#define R_LIMIT_MPS             0.60f
#define R_INNER_REVERSE_MPS     0.25f

/* ---------------- 第 3 环：速度环 PI（CCR 单位） ---------------- */
#define SPEED_KP_CCR            300.0f
/* KI 不能太大：积分穿越频率 ≈ KI / MOTOR_CCR_PER_MPS，
   超过约 10 rad/s 就会被电机动态 + 测速延迟推过 180°，形成加减速抖动。 */
#define SPEED_KI_CCR            2500.0f
#define SPEED_I_OUT_LIMIT       600.0f

/* ---------------- 安全 ---------------- */
/* 短暂丢线（十字路口、虚线、折角）保持上次偏差继续找线；
   超过这个时间还找不到才停车。 */
#define LINE_LOST_STOP_MS       500U
#define LINE_LOST_STOP_TICKS    (LINE_LOST_STOP_MS / CTRL_PERIOD_MS)
#define LINE_SEARCH_SPEED_SCALE 0.35f   /* 找线时降到 35% 车速，原地扫转 */
/* 找线专用转向增益（dps 每权重）：偏差 0.5 就给满角速度上限 */
#define LINE_SEARCH_W_KP        800.0f
/* 找线时累积扫过的角度上限。这一条专门治"直角弯转过头、绕回去把原赛道
   又走一遍"：原来丢线后一直保持满舵转，500 ms × 400 dps 能转过 200 度，
   早就转过头了。现在转过 110 度就停止转向、改为直行去找线。 */
#define LINE_SEARCH_MAX_SWEEP_DEG  110.0f

/* ---------------- 环岛 / 分叉 ---------------- */
/* 压线 >= 4 路就开始做"宽区"判定：环岛入口的分叉常常只压到 4 路，
   门槛设 5 会漏掉。 */
#define LINE_WIDE_COUNT         4U
/* 压线 >= 6 路才算真正的黑区，这时才掐掉 D 项。
   正常循迹常有 4 路同时压线，那种情况 D 必须保留（弯道阻尼靠它），
   否则门槛降到 4 之后弯道反而更抖。 */
#define LINE_D_OFF_COUNT        6U
/* 识别到分叉后强制满舵转多久（300 ms，400 dps 约转过 120 度） */
#define BRANCH_HOLD_TICKS       30U
/* 必须连续看到"干净单线"（压线 <= 3 路）这么久才允许再次触发，
   这样方框环岛每个角之间都能重新触发，而卡在路口里不会反复触发。 */
#define LINE_CLEAN_MAX_COUNT    3U
#define LINE_CLEAN_REARM_TICKS  10U
/* 堵转保护 */
#define STALL_MS                800U
#define STALL_TICKS             (STALL_MS / CTRL_PERIOD_MS)
#define STALL_CCR_MIN           300.0f
#define STALL_SPEED_MPS         0.03f
#define RUN_GRACE_MS            1000U
#define RUN_GRACE_TICKS         (RUN_GRACE_MS / CTRL_PERIOD_MS)
/* 陀螺仪超时：主循环里每轮都会刷新 gyro_tick，超过这个时间没刷新
   说明 IMU 挂了。此时角速度环拿的是旧值，必须停机。 */
#define IMU_TIMEOUT_MS          100U

/* ---------------- 陀螺仪零偏与方向自检 ---------------- */
/* 上电静止这段时间内累积 Z 轴角速度求均值作为零偏。
   BMI270 静止时 gyro_z 不是 0，不扣掉的话角速度环会以为车在一直转。 */
#define GYRO_BIAS_SETTLE_MS     500U
#define GYRO_BIAS_MIN_SAMPLES   200U
/* 方向自检：让左轮正转、右轮反转（车体顺时针转），测到的角速度应该是负的。
   测出来是正的就说明陀螺仪装反了，自动把 p_yaw_sign 取反。
   这样"陀螺仪方向"这个唯一没法从数据推算的未知量就不用靠人工试了。 */
#define GYRO_SELFTEST_SPIN_MS   200U
#define GYRO_SELFTEST_WARMUP_MS 80U
#define GYRO_SELFTEST_CCR       320U
#define GYRO_SELFTEST_MIN_DPS   8.0f

/* ---------------- 串口调参 ---------------- */
#define TUNE_RX_BUF             64U
#define TUNE_IDLE_MS            120U
#define PRINT_MS                100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_tim1_up;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//负压启动相关宏
//负压启动流程：900us脉宽50Hzpwm信号输出2秒后，改为持续2000us脉宽50Hzpwm信号输出
#define ESC_PULSE_SLOTS       200U       /* 20 ms at the 10 kHz TIM1 update rate */
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms */
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms */
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s */

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;

/* Fill a whole 20 ms frame.  Only used once, before the DMA is started. */
static void ESC_BuildDmaTable(uint32_t high_slots)
{
  uint32_t i;

  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}

/* Rewrite only the slots whose level really changes, in increasing index
   order: the circular DMA consumes the table in that same order, and slot 9
   is not transferred before ~0.9 ms after this interrupt, so the frame that
   is already running is never disturbed. */
static void ESC_SetPulseWidth(uint32_t high_slots)
{
  uint32_t first;
  uint32_t last;
  uint32_t i;

  if (high_slots == esc_high_slots)
  {
    return;
  }

  first = (high_slots < esc_high_slots) ? high_slots : esc_high_slots;
  last  = (high_slots > esc_high_slots) ? high_slots : esc_high_slots;

  for (i = first; i < last; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}
static void ESC_fullPulseWidth(void){
  uint32_t i;
  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = GPIO_PIN_11;
  }
  esc_high_slots = ESC_PULSE_SLOTS;
}
/* One interrupt per 20 ms frame: DMA transfer complete in circular mode.
   The 50 Hz waveform itself is produced by TIM1 + DMA in hardware; this
   handler only counts frames and switches 0.9 ms -> 2.0 ms after 2 s. */
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma)
{
  if (esc_period_count < ESC_START_PERIODS)
  {
    ++esc_period_count;
    if (esc_period_count == ESC_START_PERIODS)
    {
      /* The frame that has just been completed was the last 0.9 ms frame:
         100 x 20 ms = 2 s have elapsed, 2.0 ms is now held forever. */
      if (!IF_start_Brushless_motor)
        ESC_SetPulseWidth(10U);
      else
        ESC_SetPulseWidth(ESC_RUN_PULSE_SLOTS);
      /* Nothing left to time, so stop the 50 Hz interrupt. */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void LineSensor_Update(void);
static void Encoder_UpdateSpeed(void);
static void GyroBias_Update(float raw);
static void Control_Update(void);
static void RunButton_Update(void);
static void Tune_Service(uint32_t now);
static void Startup_SelfTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/* ==========================================================================
   在线可整定的参数：上电取宏里的默认值，运行中由串口改写。
   注意 erase 后（重新上电）会恢复成宏里的值。
   ========================================================================== */
static volatile float p_w_kp     = W_KP;
static volatile float p_w_kd     = W_KD;
static volatile float p_w_limit  = W_LIMIT_DPS;
static volatile float p_yaw_ff   = YAW_FF;
static volatile float p_yaw_kp   = YAW_KP;
static volatile float p_yaw_ki   = YAW_KI;
static volatile float p_yaw_kd   = YAW_KD;
static volatile float p_r_limit  = R_LIMIT_MPS;
static volatile float p_v_target = SPEED_TARGET_MPS;
static volatile float p_sp_kp    = SPEED_KP_CCR;
static volatile float p_sp_ki    = SPEED_KI_CCR;
static volatile float p_ccr_zero = MOTOR_CCR_ZERO;
static volatile float p_ccr_pmps = MOTOR_CCR_PER_MPS;
static volatile float p_hold_ccr = MOTOR_CCR_HOLD;
static volatile float p_line_sign = -1.0f;  /* 偏差 -> 角速度 的方向 */
static volatile float p_yaw_sign  = 1.0f;   /* 陀螺仪安装方向 */

/* ---------------- 光电管 ---------------- */
static volatile uint16_t line_bits;         /* 12 路原始位图 */
static volatile int16_t  line_err_milli;    /* 偏差，千分之一权重 */
static volatile uint8_t  line_ok;           /* 至少一路压线 */

/* ---------------- 编码器 / 车速 ---------------- */
static volatile float speed_l, speed_r;     /* m/s */
static int16_t enc_hist_l[ENC_AVG_SAMPLES];
static int16_t enc_hist_r[ENC_AVG_SAMPLES];
static uint8_t enc_hist_idx;
static int32_t enc_sum_l, enc_sum_r;

/* ---------------- 陀螺仪 ---------------- */
static volatile float gyro_w;               /* 已扣零偏、已按安装方向修正，dps，左转为正 */
static volatile uint32_t gyro_tick;
static float    gyro_bias;
static float    gyro_bias_sum;
static uint16_t gyro_bias_cnt;
static uint32_t gyro_bias_tick;
static uint8_t  gyro_bias_ok;

/* ---------------- 控制器状态 ---------------- */
static float line_prev_err;
static float line_err_hist[LINE_D_WINDOW];
static uint8_t line_err_idx;
static float line_d_lpf;
static float w_meas_lpf;                    /* 角速度低通状态 */
static float yaw_i;                         /* 角速度环积分输出，m/s */
static float yaw_prev_e;                    /* 角速度误差，用于 D */
static float yaw_d_lpf;
static float speed_i_l, speed_i_r;          /* 速度环积分输出，CCR */
static float v_ramp;
static float dir_l = 1.0f, dir_r = 1.0f;    /* 每轮当前驱动方向（带迟滞） */
static float sweep_deg;                     /* 丢线找线时累积扫过的角度 */
static uint16_t br_ticks;                   /* 分叉强制满舵剩余拍数 */
static uint8_t  br_armed = 1U;              /* 1 = 允许识别分叉 */
static uint16_t clean_ticks;                /* 连续"干净单线"拍数 */

/* ---------------- 遥测 ---------------- */
static volatile float dbg_err;
static volatile float dbg_w_ref;
static volatile float dbg_w_meas;
static volatile float dbg_r;
static volatile int16_t dbg_ccr_l, dbg_ccr_r;

/* ---------------- 运行状态 ---------------- */
static volatile uint8_t run_on;
static uint16_t run_ticks;
static uint16_t lost_ticks;
static uint16_t stall_ticks;
static volatile uint8_t fault_code;         /* 0 正常 / 1 堵转或编码器反接 */

/* ---------------- 串口 ---------------- */
static char tune_rx[TUNE_RX_BUF];
static uint8_t tune_rx_len;
static uint32_t tune_last_tick;

/* ---------------- 小工具 ---------------- */
static float Limit(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float AbsF(float v)
{
  return (v < 0.0f) ? -v : v;
}

/* 多路复用器切换通道后要等 1 us，multiplexer_delay_us() 用 DWT->CYCCNT 计时。
   DWT 不使能时 CYCCNT 恒为 0，那个 while 会死等，整个程序卡死。 */
static void DwtCyccnt_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void LineErrHist_Clear(void)
{
  uint8_t i;

  for (i = 0U; i < LINE_D_WINDOW; ++i) line_err_hist[i] = 0.0f;
  line_err_idx = 0U;
}

/* 某通道的权重：ch0 在最左 = -6，ch11 在最右 = +6，中心在 ch5/ch6 之间。 */
static int32_t Line_Weight(uint16_t ch)
{
  return (ch < 6U) ? ((int32_t)ch - 6L) : ((int32_t)ch - 5L);
}

static uint16_t Line_ActiveCount(uint16_t bits)
{
  uint16_t n = 0U, ch;

  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    if (MUX_GET_CHANNEL(bits, ch) == LINE_ACTIVE_LEVEL) ++n;
  }
  return n;
}

/* 读 12 路光电管，算加权平均偏差。
   偏差为正 = 线在车体右侧，为负 = 线在左侧。 */
static void LineSensor_Update(void)
{
  uint16_t bits;
  int32_t sum = 0;
  uint16_t n = 0U, ch;

  MUX_get_value(&bits);
  line_bits = bits;

  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    if (MUX_GET_CHANNEL(bits, ch) == LINE_ACTIVE_LEVEL)
    {
      sum += Line_Weight(ch);
      ++n;
    }
  }

  if (n == 0U)
  {
    line_err_milli = 0;
    line_ok = 0U;
    return;
  }

  line_err_milli = (int16_t)((float)sum * LINE_ERROR_SCALE / (float)n);
  line_ok = 1U;
}

/* 宽黑区（十字路口/环岛入口/分叉）时加权平均会被拉向中间，
   直接用会"看见分叉当没看见"。按压线通道是否"分成两段"区分：

   1) 两段之间断开 >= 2 路 = 主线 + 支线同时被看到（环岛入口、岔路）。
      取"离中心更远的那一段"的加权平均，返回 1。
   2) 连续一段（宽线/十字黑区）：取最左最右中点，返回 0。

   *out 单位与 line_err_milli 一致（千分之一权重）。 */
static uint8_t Line_WideError(int16_t *out)
{
  int32_t run_sum[4];
  uint16_t run_cnt[4];
  int16_t run_first[4], run_last[4];
  uint8_t runs = 0U, best = 0U, i, has_gap = 0U;
  uint8_t prev = 0U;
  uint16_t ch;

  *out = 0;

  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    uint8_t act = (MUX_GET_CHANNEL(line_bits, ch) == LINE_ACTIVE_LEVEL) ? 1U : 0U;

    if (act)
    {
      if ((prev == 0U) && (runs < 4U))
      {
        run_sum[runs] = 0;
        run_cnt[runs] = 0U;
        run_first[runs] = (int16_t)ch;
        ++runs;
      }
      if (runs > 0U)
      {
        run_sum[runs - 1U] += Line_Weight(ch);
        ++run_cnt[runs - 1U];
        run_last[runs - 1U] = (int16_t)ch;
      }
    }
    prev = act;
  }

  if (runs == 0U) return 0U;

  for (i = 1U; i < runs; ++i)
  {
    if (((int16_t)(run_first[i] - run_last[i - 1U]) - 1) >= 2) has_gap = 1U;
  }

  if ((runs == 1U) || (has_gap == 0U))
  {
    int32_t wf = Line_Weight((uint16_t)run_first[0]);
    int32_t wl = Line_Weight((uint16_t)run_last[runs - 1U]);

    *out = (int16_t)((float)(wf + wl) * LINE_ERROR_SCALE * 0.5f);
    return 0U;
  }

  /* 确实断开：取平均位置离中心最远的那一段 */
  for (i = 1U; i < runs; ++i)
  {
    int32_t a = (run_sum[i] < 0) ? -run_sum[i] : run_sum[i];
    int32_t b = (run_sum[best] < 0) ? -run_sum[best] : run_sum[best];

    if (a > b) best = i;
  }
  if (run_cnt[best] == 0U) return 0U;

  *out = (int16_t)((float)run_sum[best] * LINE_ERROR_SCALE / (float)run_cnt[best]);
  return 1U;
}

/* 10 ms 读一次编码器，20 ms 滑窗求平均车速。 */
static void Encoder_UpdateSpeed(void)
{
  int16_t raw_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t raw_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  raw_l = (int16_t)(ENC_L_SIGN * raw_l);
  raw_r = (int16_t)(ENC_R_SIGN * raw_r);

  enc_sum_l += (int32_t)raw_l - enc_hist_l[enc_hist_idx];
  enc_sum_r += (int32_t)raw_r - enc_hist_r[enc_hist_idx];
  enc_hist_l[enc_hist_idx] = raw_l;
  enc_hist_r[enc_hist_idx] = raw_r;
  if (++enc_hist_idx >= ENC_AVG_SAMPLES) enc_hist_idx = 0U;

  speed_l = ((float)enc_sum_l / ENC_COUNTS_PER_METER) /
            ((float)(ENC_AVG_SAMPLES * CTRL_PERIOD_MS) / 1000.0f);
  speed_r = ((float)enc_sum_r / ENC_COUNTS_PER_METER) /
            ((float)(ENC_AVG_SAMPLES * CTRL_PERIOD_MS) / 1000.0f);
}

/* 上电后整车静止，累积 Z 轴角速度求均值作为零偏。 */
static void GyroBias_Update(float raw)
{
  uint32_t now = HAL_GetTick();

  if (gyro_bias_ok != 0U) return;

  if (gyro_bias_tick == 0U) gyro_bias_tick = (now != 0U) ? now : 1U;
  if ((now - gyro_bias_tick) < GYRO_BIAS_SETTLE_MS) return;

  gyro_bias_sum += raw;
  if (gyro_bias_cnt < 0xFFFFU) ++gyro_bias_cnt;

  /* 样本够了就锁存，之后不再更新（发车后再采会把转动量当成零偏） */
  if (gyro_bias_cnt >= GYRO_BIAS_MIN_SAMPLES)
  {
    gyro_bias = gyro_bias_sum / (float)gyro_bias_cnt;
    gyro_bias_ok = 1U;
    printf("gyro,bias,%ld,%u\r\n", (long)(gyro_bias * 100.0f), (unsigned)gyro_bias_cnt);
  }
}

/* 上电自检：先测陀螺仪零偏，再用"左轮正转 + 右轮反转"让车体顺时针转一下，
   由测到的角速度符号确定陀螺仪安装方向。

   这一步很关键：陀螺仪正装/反装用肉眼看不出，符号反了会让角速度环变成
   **正反馈**——弯道里一直以为"转得不够"而持续满舵，结果转过头、
   绕回去又把原赛道走一遍，直道上也会左右摆导致频繁丢线（车速看起来变慢）。
   自检把它变成自动的，不用再靠人工试 n=±1。 */
static void Startup_SelfTest(void)
{
  uint16_t i;
  float sum = 0.0f;
  float test = 0.0f;

  /* 1) 零偏：静止取 200 次平均 */
  HAL_Delay(GYRO_BIAS_SETTLE_MS);
  for (i = 0U; i < 200U; ++i)
  {
    dodo_BMI270_get_data();
    sum += BMI270_gyro_transition(BMI270_gyro_z);
    HAL_Delay(2U);
  }
  gyro_bias = sum / 200.0f;
  gyro_bias_ok = 1U;

  /* 2) 方向：左轮正转 + 右轮反转 = 顺时针（右转），期望测到负角速度 */
  HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FORWARD);
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_REVERSE);
  TIM1->CCR1 = GYRO_SELFTEST_CCR;
  TIM1->CCR2 = GYRO_SELFTEST_CCR;
  HAL_Delay(GYRO_SELFTEST_WARMUP_MS);

  for (i = 0U; i < 50U; ++i)
  {
    dodo_BMI270_get_data();
    test += BMI270_gyro_transition(BMI270_gyro_z) - gyro_bias;
    HAL_Delay(4U);
  }

  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FORWARD);
  HAL_Delay(40U);

  test /= 50.0f;
  if (test > GYRO_SELFTEST_MIN_DPS)       p_yaw_sign = -1.0f;
  else if (test < -GYRO_SELFTEST_MIN_DPS) p_yaw_sign =  1.0f;
  /* 否则（几乎测不到转动）保持默认值，多半是轮子没转或被按住 */

  printf("gyro,bias,%ld,test,%ld,n=%ld\r\n",
         (long)(gyro_bias * 100.0f), (long)test, (long)p_yaw_sign);
}

/* 发车键：PB14 低电平按下，每按一次在"待发车 / 运行中"之间切换。 */
static void RunButton_Update(void)
{
  static uint8_t stable = 1U;
  static uint8_t candidate = 1U;
  static uint32_t since;
  uint32_t now = HAL_GetTick();
  uint8_t raw = (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) ? 0U : 1U;

  if (raw != candidate)
  {
    candidate = raw;
    since = now;
  }
  else if ((candidate != stable) && ((now - since) >= 20U))
  {
    stable = candidate;
    if (stable == 0U)
    {
      run_on = (run_on != 0U) ? 0U : 1U;
      fault_code = 0U;
      run_ticks = 0U;
      lost_ticks = 0U;
      stall_ticks = 0U;
      line_prev_err = 0.0f;
      line_d_lpf = 0.0f;
      LineErrHist_Clear();
      w_meas_lpf = 0.0f;
      yaw_i = 0.0f;
      yaw_prev_e = 0.0f;
      yaw_d_lpf = 0.0f;
      speed_i_l = 0.0f;
      speed_i_r = 0.0f;
      v_ramp = 0.0f;
      dir_l = 1.0f;
      dir_r = 1.0f;
      sweep_deg = 0.0f;
      br_ticks = 0U;
      br_armed = 1U;
      clean_ticks = 0U;
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
    }
  }
}

/* 每 10 ms 调用一次：三环串级全部算完后写 CCR。 */
static void Control_Update(void)
{
  const float dt = CTRL_DT;
  float err = 0.0f;
  float deriv;
  float w_ref;
  float w_meas;
  float e_w;
  float r;
  float v_target;
  float t_l, t_r;
  float ccr_l, ccr_r;
  float search_scale = 1.0f;
  uint8_t searching = 0U;
  uint8_t d_ok = 1U;

  /* ---- 未发车 / 故障 / 陀螺仪超时：停机 ---- */
  if ((run_on == 0U) || (fault_code != 0U) ||
      ((HAL_GetTick() - gyro_tick) > IMU_TIMEOUT_MS))
  {
    stall_ticks = 0U;
    lost_ticks = 0U;
    speed_i_l = 0.0f;
    speed_i_r = 0.0f;
    line_prev_err = 0.0f;
    line_d_lpf = 0.0f;
    LineErrHist_Clear();
    w_meas_lpf = 0.0f;
    yaw_i = 0.0f;
    yaw_prev_e = 0.0f;
    yaw_d_lpf = 0.0f;
    v_ramp = 0.0f;
    dir_l = 1.0f;
    dir_r = 1.0f;
    sweep_deg = 0.0f;
    br_ticks = 0U;
    br_armed = 1U;
    clean_ticks = 0U;
    dbg_err = 0.0f;
    dbg_w_ref = 0.0f;
    dbg_w_meas = 0.0f;
    dbg_r = 0.0f;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    return;
  }

  if (run_ticks < 0xFFFFU) ++run_ticks;

  /* ================= 偏差（第 1 环的输入） ================= */
  if (line_ok == 0U)
  {
    if (lost_ticks < LINE_LOST_STOP_TICKS) ++lost_ticks;
    if (lost_ticks >= LINE_LOST_STOP_TICKS)
    {
      /* 丢线超时：停车等待，不盲目往前冲 */
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
      speed_i_l = 0.0f;
      speed_i_r = 0.0f;
      LineErrHist_Clear();
      v_ramp = 0.0f;
      dbg_err = 0.0f;
      dbg_r = 0.0f;
      return;
    }
    /* 短暂丢线：保持上次偏差，同时降速原地扫转找线 */
    err = line_prev_err;
    search_scale = LINE_SEARCH_SPEED_SCALE;
    searching = 1U;
    d_ok = 0U;
    clean_ticks = 0U;
  }
  else
  {
    int16_t milli = line_err_milli;
    uint16_t n_active = Line_ActiveCount(line_bits);
    uint8_t branch = 0U;

    if (n_active >= LINE_WIDE_COUNT)
    {
      branch = Line_WideError(&milli);
    }
    if (n_active >= LINE_D_OFF_COUNT)
    {
      d_ok = 0U;   /* 真正的黑区：偏差本来就会突变，D 只会打出反向脉冲 */
    }

    /* 连续看到"干净单线"才重新允许认分叉：
       方框环岛每个角之间都有一段正常单线，所以每个角都能触发；
       而卡在路口里时压线一直很多，不会反复触发（否则会在环岛里绕不出去）。 */
    if (n_active <= LINE_CLEAN_MAX_COUNT)
    {
      if (clean_ticks < 0xFFFFU) ++clean_ticks;
      if (clean_ticks >= LINE_CLEAN_REARM_TICKS) br_armed = 1U;
    }
    else
    {
      clean_ticks = 0U;
    }

    err = (float)milli / LINE_ERROR_SCALE;   /* 正 = 线在车体右侧 */
    lost_ticks = 0U;
    sweep_deg = 0.0f;
    line_prev_err = err;

    /* 见到"主线 + 支线"（环岛入口/岔路）：短促强制满舵拐进去。
       这里直接把偏差置成 ±(上限/增益)，复用后面第 1 环的符号逻辑，
       不用另外写一套方向判断。 */
    if ((branch != 0U) && (br_armed != 0U) && (br_ticks == 0U))
    {
      br_ticks = BRANCH_HOLD_TICKS;
      br_armed = 0U;
      clean_ticks = 0U;
    }
    if (br_ticks > 0U)
    {
      /* 用编译期常量算"满舵对应的偏差"，避免用户把 wkp 设成 0 时除零 */
      const float full = W_LIMIT_DPS / W_KP;

      err = (milli >= 0) ? full : -full;
      line_prev_err = err;
      d_ok = 0U;
    }
  }
  if (br_ticks > 0U) --br_ticks;

  /* 偏差变化率用 30 ms 窗口差分：光电管一档 1.33 个权重，
     单拍 10 ms 差商能到 ±133 权重/s，乘 KD 就是一个明显的差速脉冲。 */
  if (d_ok != 0U)
  {
    deriv = (err - line_err_hist[line_err_idx]) / ((float)LINE_D_WINDOW * dt);
  }
  else
  {
    deriv = 0.0f;
  }
  line_err_hist[line_err_idx] = err;
  if (++line_err_idx >= LINE_D_WINDOW) line_err_idx = 0U;

  line_d_lpf = (LINE_D_LPF_ALPHA * line_d_lpf) +
               ((1.0f - LINE_D_LPF_ALPHA) * deriv);
  dbg_err = err;

  /* ================= 第 2 环的量测（提前算，找线扫转角要用） ================= */
  /* 扣零偏（已在 gyro_w 里做过）+ 按安装方向修正 + 低通 + 死区 */
  w_meas = p_yaw_sign * gyro_w;
  w_meas_lpf = (YAW_LPF_ALPHA * w_meas_lpf) +
               ((1.0f - YAW_LPF_ALPHA) * w_meas);
  w_meas = w_meas_lpf;
  if (AbsF(w_meas) < YAW_DEADBAND_DPS) w_meas = 0.0f;
  dbg_w_meas = w_meas;

  /* 丢线找线时累积"已经扫过多少度"。
     原来丢线后一直保持满舵，500 ms × 400 dps 能转过 200 度，
     直角弯早就转过头了，于是车沿着转回来的方向又压上原赛道——
     表现就是"直角转弯后反向回到赛道"。转过上限就停手、改直行找线。 */
  if (searching != 0U) sweep_deg += w_meas * dt;

  /* ================= 第 1 环：转向环 PD -> 目标角速度 ================= */
  /* 符号：err > 0 线在右侧，需要向右转；而"正角速度"定义为向左转，
     所以整条转向支路乘 p_line_sign（默认 -1）。 */
  if (searching != 0U)
  {
    /* 找线阶段用大增益：偏差 0.5 就给满角速度上限，车原地扫转 */
    w_ref = Limit(p_line_sign * err * LINE_SEARCH_W_KP, -p_w_limit, p_w_limit);

    /* 已经转过足够的角度还找不到线：别再转了，直行靠车速去碰线。
       继续转只会绕回原赛道。 */
    if (AbsF(sweep_deg) >= LINE_SEARCH_MAX_SWEEP_DEG) w_ref = 0.0f;
  }
  else
  {
    float d_term = p_w_kd * line_d_lpf;

    d_term = Limit(d_term, -p_w_limit * 0.5f, p_w_limit * 0.5f);
    w_ref = Limit(p_line_sign * ((p_w_kp * err) + d_term), -p_w_limit, p_w_limit);
  }
  dbg_w_ref = w_ref;

  /* ================= 第 2 环：角速度环 PID -> 差速 R ================= */
  e_w = w_ref - w_meas;

  /* 积分：积的是误差，输出直接是 m/s，单独限幅防止绕圈 */
  yaw_i = Limit(yaw_i + (p_yaw_ki * e_w * dt), -YAW_I_LIMIT, YAW_I_LIMIT);

  /* 微分：对误差做，低通一次 */
  {
    float de = (e_w - yaw_prev_e) / dt;

    yaw_prev_e = e_w;
    yaw_d_lpf = (YAW_D_LPF_ALPHA * yaw_d_lpf) +
                ((1.0f - YAW_D_LPF_ALPHA) * de);
  }

  r = (p_yaw_ff * w_ref) + (p_yaw_kp * e_w) + yaw_i + (p_yaw_kd * yaw_d_lpf);
  r = Limit(r, -p_r_limit, p_r_limit);
  dbg_r = r;

  /* ================= 速度环的目标 ================= */
  if (v_ramp < p_v_target)
  {
    v_ramp += SPEED_ACCEL_MPS2 * dt;
    if (v_ramp > p_v_target) v_ramp = p_v_target;
  }
  else if (v_ramp > p_v_target)
  {
    v_ramp -= SPEED_ACCEL_MPS2 * dt;
    if (v_ramp < p_v_target) v_ramp = p_v_target;
  }
  v_target = v_ramp * search_scale;

  /* R > 0：左轮慢、右轮快，车向左转 */
  t_l = Limit(v_target - r, -R_INNER_REVERSE_MPS, SPEED_LIMIT_MPS);
  t_r = Limit(v_target + r, -R_INNER_REVERSE_MPS, SPEED_LIMIT_MPS);

  /* ================= 第 3 环：速度环 PI（每轮独立，输出 CCR） ================= */
  {
    /* 反转拖拽补偿：按差速大小成比例，直道不补，急弯补满 */
    float hold = p_hold_ccr * Limit(AbsF(r) / MOTOR_HOLD_REF_MPS, 0.0f, 1.0f);
    float mag_l, mag_r, e_l, e_r;

    /* 方向迟滞：目标在 0 附近时不要来回切正反转 */
    if (dir_l > 0.0f) { if (t_l < -MOTOR_REV_ENTER_MPS) dir_l = -1.0f; }
    else              { if (t_l > -MOTOR_REV_LEAVE_MPS) dir_l = 1.0f; }
    if (dir_r > 0.0f) { if (t_r < -MOTOR_REV_ENTER_MPS) dir_r = -1.0f; }
    else              { if (t_r > -MOTOR_REV_LEAVE_MPS) dir_r = 1.0f; }

    mag_l = (t_l * dir_l > 0.0f) ? AbsF(t_l) : 0.0f;
    mag_r = (t_r * dir_r > 0.0f) ? AbsF(t_r) : 0.0f;
    e_l = mag_l - (speed_l * dir_l);
    e_r = mag_r - (speed_r * dir_r);

    if (mag_l <= 0.0f) { ccr_l = 0.0f; speed_i_l = 0.0f; }
    else
    {
      speed_i_l = Limit(speed_i_l + (p_sp_ki * e_l * dt),
                        -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);
      ccr_l = p_ccr_zero + (p_ccr_pmps * mag_l) +
              (p_sp_kp * e_l) + speed_i_l + ((dir_l < 0.0f) ? hold : 0.0f);
    }

    if (mag_r <= 0.0f) { ccr_r = 0.0f; speed_i_r = 0.0f; }
    else
    {
      speed_i_r = Limit(speed_i_r + (p_sp_ki * e_r * dt),
                        -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);
      ccr_r = p_ccr_zero + (p_ccr_pmps * mag_r) +
              (p_sp_kp * e_r) + speed_i_r + ((dir_r < 0.0f) ? hold : 0.0f);
    }

    ccr_l = Limit(ccr_l, 0.0f, (float)MOTOR_CCR_MAX);
    ccr_r = Limit(ccr_r, 0.0f, (float)MOTOR_CCR_MAX);

    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin,
                      (dir_l > 0.0f) ? L_DIR_FORWARD : L_DIR_REVERSE);
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin,
                      (dir_r > 0.0f) ? R_DIR_FORWARD : R_DIR_REVERSE);
    TIM1->CCR1 = (uint16_t)ccr_l;
    TIM1->CCR2 = (uint16_t)ccr_r;
    dbg_ccr_l = (int16_t)ccr_l;
    dbg_ccr_r = (int16_t)ccr_r;

    /* ---- 堵转 / 编码器反接保护 ---- */
    /* 命令正转却测到明显负速度（或反之）= 编码器符号或 DIR 脚接反，
       速度环会正反馈越跑越快，必须立刻停。 */
    {
      float amax = AbsF(speed_l);
      float atmp = AbsF(speed_r);
      uint8_t bad = 0U;

      if (atmp > amax) amax = atmp;
      if ((mag_l > 0.05f) && (ccr_l >= STALL_CCR_MIN) &&
          ((speed_l * dir_l) < -STALL_SPEED_MPS)) bad = 1U;
      if ((mag_r > 0.05f) && (ccr_r >= STALL_CCR_MIN) &&
          ((speed_r * dir_r) < -STALL_SPEED_MPS)) bad = 1U;

      if ((run_ticks > RUN_GRACE_TICKS) &&
          (bad || (((ccr_l >= STALL_CCR_MIN) || (ccr_r >= STALL_CCR_MIN)) &&
                   (amax < STALL_SPEED_MPS))))
      {
        if (stall_ticks < 0xFFFFU) ++stall_ticks;
        if (stall_ticks >= STALL_TICKS)
        {
          fault_code = 1U;
          TIM1->CCR1 = 0U;
          TIM1->CCR2 = 0U;
        }
      }
      else
      {
        stall_ticks = 0U;
      }
    }
  }
}

/* ---- 串口在线调参：形如 wkp=60 / rkp=0.0009 / spd=0.45 / show ----
   串口助手不发换行也可以，按 120 ms 空闲收尾。 */
static void Tune_Apply(const char *name, uint8_t len, float v)
{
  char nm[8];
  uint8_t i;

  if (len >= sizeof(nm)) { printf("err,name\r\n"); return; }
  for (i = 0U; i < len; ++i) nm[i] = name[i];
  nm[len] = '\0';

  if      ((len==3U)&&(nm[0]=='w')&&(nm[1]=='k')&&(nm[2]=='p')) p_w_kp      = v;
  else if ((len==3U)&&(nm[0]=='w')&&(nm[1]=='k')&&(nm[2]=='d')) p_w_kd      = v;
  else if ((len==3U)&&(nm[0]=='w')&&(nm[1]=='l')&&(nm[2]=='m')) p_w_limit   = v;
  else if ((len==3U)&&(nm[0]=='y')&&(nm[1]=='f')&&(nm[2]=='f')) p_yaw_ff    = v;
  else if ((len==3U)&&(nm[0]=='y')&&(nm[1]=='k')&&(nm[2]=='p')) p_yaw_kp    = v;
  else if ((len==3U)&&(nm[0]=='y')&&(nm[1]=='k')&&(nm[2]=='i')) p_yaw_ki    = v;
  else if ((len==3U)&&(nm[0]=='y')&&(nm[1]=='k')&&(nm[2]=='d')) p_yaw_kd    = v;
  else if ((len==3U)&&(nm[0]=='r')&&(nm[1]=='l')&&(nm[2]=='m')) p_r_limit   = v;
  else if ((len==3U)&&(nm[0]=='s')&&(nm[1]=='p')&&(nm[2]=='d')) p_v_target  = v;
  else if ((len==3U)&&(nm[0]=='s')&&(nm[1]=='k')&&(nm[2]=='p')) p_sp_kp     = v;
  else if ((len==3U)&&(nm[0]=='s')&&(nm[1]=='k')&&(nm[2]=='i')) p_sp_ki     = v;
  else if ((len==2U)&&(nm[0]=='c')&&(nm[1]=='z')) p_ccr_zero  = v;
  else if ((len==2U)&&(nm[0]=='c')&&(nm[1]=='p')) p_ccr_pmps  = v;
  else if ((len==2U)&&(nm[0]=='h')&&(nm[1]=='c')) p_hold_ccr  = v;
  else if ((len==1U)&&(nm[0]=='l')) p_line_sign = (v < 0.0f) ? -1.0f : 1.0f;
  else if ((len==1U)&&(nm[0]=='n')) p_yaw_sign  = (v < 0.0f) ? -1.0f : 1.0f;
  else { printf("err,name\r\n"); return; }

  /* 兜一层限幅，防止手误把参数输成离谱值后车子失控 */
  p_w_kp      = Limit(p_w_kp, 0.0f, 300.0f);
  p_w_kd      = Limit(p_w_kd, 0.0f, 20.0f);
  p_w_limit   = Limit(p_w_limit, 0.0f, 1500.0f);
  p_yaw_ff    = Limit(p_yaw_ff, 0.0f, 0.02f);
  p_yaw_kp    = Limit(p_yaw_kp, 0.0f, 0.02f);
  p_yaw_ki    = Limit(p_yaw_ki, 0.0f, 0.05f);
  p_yaw_kd    = Limit(p_yaw_kd, 0.0f, 0.02f);
  p_r_limit   = Limit(p_r_limit, 0.0f, SPEED_LIMIT_MPS);
  p_v_target  = Limit(p_v_target, 0.0f, SPEED_LIMIT_MPS);
  p_sp_kp     = Limit(p_sp_kp, 0.0f, 2000.0f);
  p_sp_ki     = Limit(p_sp_ki, 0.0f, 10000.0f);
  p_ccr_zero  = Limit(p_ccr_zero, 0.0f, 7199.0f);
  p_ccr_pmps  = Limit(p_ccr_pmps, 0.0f, 20000.0f);
  p_hold_ccr  = Limit(p_hold_ccr, 0.0f, 7199.0f);

  printf("set,%s,%ld\r\n", nm, (long)(v * 1000.0f));
}

static void Tune_Print(void)
{
  printf("cfg,wkp=%ld,wkd=%ld,wlm=%ld,yff=%ld,ykp=%ld,yki=%ld,ykd=%ld\r\n",
         (long)p_w_kp, (long)(p_w_kd * 100.0f), (long)p_w_limit,
         (long)(p_yaw_ff * 1000000.0f), (long)(p_yaw_kp * 1000000.0f),
         (long)(p_yaw_ki * 1000000.0f), (long)(p_yaw_kd * 1000000.0f));
  printf("cfg2,rlm=%ld,spd=%ld,skp=%ld,ski=%ld,cz=%ld,cp=%ld,hc=%ld,l=%ld,n=%ld\r\n",
         (long)(p_r_limit * 1000.0f), (long)(p_v_target * 1000.0f),
         (long)p_sp_kp, (long)p_sp_ki,
         (long)p_ccr_zero, (long)p_ccr_pmps, (long)p_hold_ccr,
         (long)p_line_sign, (long)p_yaw_sign);
}

/* 解析 "名字=数值"。数值支持负数和小数。 */
static void Tune_Execute(char *s, uint8_t len)
{
  uint8_t i = 0U, nl = 0U, k;
  float ipart = 0.0f, fpart = 0.0f, fdiv = 0.1f;
  uint8_t neg = 0U, digits = 0U, dot = 0U;

  while ((i < len) && ((s[i] == ' ') || (s[i] == '\t'))) ++i;
  while ((len > i) && ((s[len - 1U] == ' ') || (s[len - 1U] == '\t') ||
                       (s[len - 1U] == '\r'))) --len;
  if (len <= i) return;

  if (((len - i) == 4U) && (s[i] == 's') && (s[i + 1U] == 'h') &&
      (s[i + 2U] == 'o') && (s[i + 3U] == 'w'))
  {
    Tune_Print();
    return;
  }

  while (((i + nl) < len) && (s[i + nl] != '=') && (s[i + nl] != ' ')) ++nl;
  if (nl == 0U) { printf("err,name\r\n"); return; }

  k = (uint8_t)(i + nl);
  while ((k < len) && ((s[k] == '=') || (s[k] == ' '))) ++k;

  for (; k < len; ++k)
  {
    char c = s[k];

    if (c == '-') neg = 1U;
    else if (c == '.') dot = 1U;
    else if ((c >= '0') && (c <= '9'))
    {
      if (dot == 0U) ipart = (ipart * 10.0f) + (float)(c - '0');
      else { fpart += fdiv * (float)(c - '0'); fdiv *= 0.1f; }
      ++digits;
    }
  }
  if (digits == 0U) { printf("err,value\r\n"); return; }

  {
    float v = ipart + fpart;

    if (neg) v = -v;
    Tune_Apply(&s[i], nl, v);
  }
}

static void Tune_Service(uint32_t now)
{
  if (tune_rx_len == 0U) return;
  if ((now - tune_last_tick) < TUNE_IDLE_MS) return;

  tune_rx[tune_rx_len] = '\0';
  Tune_Execute(tune_rx, tune_rx_len);
  tune_rx_len = 0U;
}

static uint8_t uart_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    char c = (char)uart_rx_byte;

    if ((c == '\n') || (c == '\r'))
    {
      if (tune_rx_len > 0U)
      {
        tune_rx[tune_rx_len] = '\0';
        Tune_Execute(tune_rx, tune_rx_len);
        tune_rx_len = 0U;
      }
    }
    else if (tune_rx_len < (TUNE_RX_BUF - 1U))
    {
      tune_rx[tune_rx_len++] = c;
      tune_last_tick = HAL_GetTick();
    }
    (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
  }
}

/* TIM2 = 1 kHz，分频到 10 ms 作为控制周期。
   注意：printf 绝对不能放在这里，一个字符约 87 us，会把控制周期挤歪。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t divider;

  if (htim->Instance != TIM2) return;

  if (++divider < CTRL_PERIOD_MS) return;
  divider = 0U;

  Encoder_UpdateSpeed();
  Control_Update();
}

/* USER CODE END 0 */
void Init_brushless_motor(void)
{
  /* PA11 idles low: the first high level then also lasts exactly 0.9 ms,
     because the DMA writes the first slot 0.1 ms after TIM1 is started. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_START_PULSE_SLOTS);
  esc_period_count = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (IF_start_Brushless_motor)
    while (esc_period_count < ESC_START_PERIODS)
      HAL_Delay(200);//等待无刷电桥负压启动完成
  /* Disabled fan: return immediately; DMA completes the existing
     0.9 ms -> 1.0 ms idle-signal sequence in the background. */
}
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Init_brushless_motor(); //等待无刷电调负压启动完成
  /* USER CODE BEGIN 2 */
  /* 编码器：TIM3 = 左轮，TIM4 = 右轮，硬件正交计数 */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();

  /* 光电管读取依赖 DWT->CYCCNT 计时，必须先打开，否则会死等卡死 */
  DwtCyccnt_Init();

  /* 陀螺仪：上电自检（测零偏 + 自动判定安装方向） */
  dodo_BMI270_init();
  gyro_tick = HAL_GetTick();
  gyro_bias_tick = HAL_GetTick();

  /* 电机先确保停止，自检要自己控制方向 */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FORWARD);
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FORWARD);

  Startup_SelfTest();
  gyro_tick = HAL_GetTick();

  /* 1 kHz 时基 -> 10 ms 控制周期 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();

  /* 串口接收中断（在线调参） */
  (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);

  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;

  printf("ready,method1_cascade\r\n");
  Tune_Print();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ---- 陀螺仪：读出原始值，扣零偏得到角速度 ---- */
    dodo_BMI270_get_data();
    {
      float raw = BMI270_gyro_transition(BMI270_gyro_z);

      GyroBias_Update(raw);
      /* 零偏没锁存前输出 0，避免把"累积和"当成角速度送进环里 */
      gyro_w = (gyro_bias_ok != 0U) ? (raw - gyro_bias) : 0.0f;
      gyro_tick = now;
    }

    /* ---- 光电管：10 ms 采一次 ---- */
    {
      static uint32_t line_tick;

      if ((now - line_tick) >= LINE_SAMPLE_MS)
      {
        line_tick = now;
        LineSensor_Update();
      }
    }

    RunButton_Update();
    Tune_Service(now);

    /* ---- 遥测：100 ms 一帧 ----
       st,run,line_ok,fault,bits,err, w_ref,w_meas, R, L_mm/s, R_mm/s, ccrL, ccrR */
    {
      static uint32_t print_tick;

      if ((now - print_tick) >= PRINT_MS)
      {
        print_tick = now;
        printf("st,%u,%u,%u,%04X,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d\r\n",
               (unsigned)run_on,
               (unsigned)line_ok,
               (unsigned)fault_code,
               (unsigned)line_bits,
               (long)(dbg_err * 1000.0f),
               (long)(dbg_w_ref * 10.0f),
               (long)(dbg_w_meas * 10.0f),
               (long)(dbg_r * 1000.0f),
               (long)(speed_l * 1000.0f),
               (long)(speed_r * 1000.0f),
               (int)dbg_ccr_l,
               (int)dbg_ccr_r);
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* 使能 TIM2 更新中断。CubeMX 没勾 NVIC，必须在这里补上，
     否则 HAL_TIM_PeriodElapsedCallback 永远不触发，控制环等于不存在。 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* 使能 USART3 中断：串口调参依赖 HAL_UART_Receive_IT / 接收中断。
     CubeMX 同样没勾 NVIC，必须补。 */
  HAL_NVIC_SetPriority(USART3_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | R_DIR_Pin | GPIO_PIN_11 | MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin | MUX_1_Pin | MUX_2_Pin | MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RUN_Pin */
  GPIO_InitStruct.Pin = GPIO_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin | MUX_2_Pin | MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
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
