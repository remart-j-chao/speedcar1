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
/* 负压风扇：开发阶段保持 0（不启动无刷电调），
   只有明确要测试风扇时才改成 1。
   注意：改成 1 后启动流程会阻塞约 2 s（等待负压建立），上电时不要用车。 */
#define IF_start_Brushless_motor 0

/* ==========================================================================
   循迹控制参数（转向控制方法三：转向环 PD - 速度环 PI，不使用陀螺仪）
   --------------------------------------------------------------------------
   转向环：偏差 -> 转向量（左右轮速度差），用 PD
       转向量 = LINE_KP × 偏差 + LINE_KD × 偏差变化率
       转向量为正 = 右转 = 左轮加速、右轮减速
   速度环：每轮独立 PI，把左右轮实际车速拉到目标值
       目标左轮 = 基本车速 + 转向量，目标右轮 = 基本车速 - 转向量
   转向环不用积分：积分会在弯道里累积、出弯时反向过冲（画龙）。
   速度环必须用积分：直流电机低速段有死区、摩擦力不一致，
   纯 P 存在稳态误差，左右轮对不上。
   ========================================================================== */

/* 控制周期：TIM2 配成 1 kHz，再分频到 10 ms。 */
#define CTRL_PERIOD_MS          10U
#define CTRL_DT                 ((float)CTRL_PERIOD_MS / 1000.0f)

/* ---------------- 光电管 ---------------- */
/* 判定为"压线"的电平。若实测模块输出相反，把 1U 改成 0U。 */
#define LINE_ACTIVE_LEVEL       1U
/* 偏差的内部单位是"千分之一权重"，便于整数打印与在线调参。 */
#define LINE_ERROR_SCALE        1000L
/* 偏差方向：通道 0 在最左、权重 -6；通道 11 在最右、权重 +6，
   所以偏差为正 = 线在车体右侧。转向约定：转向量为正 = 右转
   （左轮加速、右轮减速）。因此这里默认 +1。 */
#define LINE_ERROR_SIGN         1.0f
/* 如果落地实测发现"线在右边却往左打"，把串口命令 l 设成 -1 即可，不用改代码。 */
#define LINE_SAMPLE_MS          10U
/* 同时压线的路数达到这个值就认为是十字/宽黑区，沿用上次偏差。 */
#define LINE_WIDE_ACTIVE_COUNT  5U

/* ---------------- 编码器 ---------------- */
/* 每米计数：按实测标定（推 1 m 读 odom 累计值）。 */
#define ENC_COUNTS_PER_METER    37385.0f
#define ENC_L_SIGN              1
#define ENC_R_SIGN              (-1)
/* 速度滑动平均窗长（样本数），3 × 10 ms = 30 ms。 */
#define ENC_AVG_SAMPLES         3U

/* ---------------- 电机输出 ---------------- */
/* TIM1 的 ARR 是 7199，占空比 = CCR / 7200。 */
#define L_DIR_FORWARD           GPIO_PIN_RESET
#define L_DIR_REVERSE           GPIO_PIN_SET
#define R_DIR_FORWARD           GPIO_PIN_SET
#define R_DIR_REVERSE           GPIO_PIN_RESET

/* ---------------- 车速 ---------------- */
#define SPEED_TARGET_MPS        0.80f    /* 基本速 */
#define SPEED_LIMIT_MPS         1.20f    /* 单轮目标速度上限 */
#define SPEED_ACCEL_MPS2        1.5f     /* 起步加速度限制 */

/* ---------------- 转向环：PD ---------------- */
/* 偏差单位是"权重"：12 路光电管，加权平均后范围约 ±6。
   转向输出直接就是左右轮的速度差（m/s），不再经过陀螺仪内环。
   LINE_KP = 0.10 时，偏差到 4.5 个权重就吃满转向上限，最外侧还能有富余。 */
#define LINE_KP                 0.10f    /* (m/s) / 权重 */
#define LINE_KD                 0.0025f  /* (m/s) / (权重/s) */
#define LINE_D_LPF_ALPHA        0.8f     /* 偏差变化率低通，抑制量化跳变尖峰 */
/* 转向（差速）上限，m/s。轮距约 0.186 m，角速度 = 2×差速/轮距。
   0.60 配上内侧轮反转，急弯最小半径可以到 6 cm 左右。 */
#define TURN_LIMIT_MPS          0.60f
/* 允许内侧轮反转多少（m/s）。急弯只靠"内侧轮停转"不够，
   必须让内侧轮倒转才能把车头硬拉过来。0 = 不允许反转。 */
#define TURN_INNER_REVERSE_MPS  0.25f

/* ---------------- 速度环（每轮独立 PI，直接以 CCR 为单位） ---------------- */
/* TIM1 的 ARR 是 7199，占空比 = CCR / 7200。
   直流电机转速对 CCR 是仿射关系（有一段不转的死区，之后近似线性）：
       需要的 CCR ≈ MOTOR_CCR_ZERO + MOTOR_CCR_PER_MPS × 车速
   这两个常数按实测两点拟合（静止不转的点 + 已知车速的点），
   前馈先把它算掉，PI 只补残差，这样即使标定不准也不会慢慢吞吞地爬。
   单位：CCR 每 (m/s)；SPEED_KP/ KI 也是 CCR 单位。 */
#define MOTOR_CCR_MAX           1000U    /* CCR 上限（约 14% 占空比，留有余量） */
#define MOTOR_CCR_ZERO          27.0f
#define MOTOR_CCR_PER_MPS       765.0f
#define SPEED_KP_CCR            300.0f
#define SPEED_KI_CCR            2000.0f
#define SPEED_I_LIMIT_CCR       700.0f

/* ---------------- 安全 ---------------- */
#define LINE_LOST_STOP_MS       300U     /* 丢线超过此时间就停机 */
#define LINE_LOST_STOP_TICKS    (LINE_LOST_STOP_MS / CTRL_PERIOD_MS)
#define STALL_MS                800U
#define STALL_TICKS             (STALL_MS / CTRL_PERIOD_MS)
#define STALL_CCR_MIN           300.0f   /* 左右至少一轮给到这么多 CCR 才算"在驱动" */
#define STALL_SPEED_MPS         0.03f
#define RUN_GRACE_MS            1000U
#define RUN_GRACE_TICKS         (RUN_GRACE_MS / CTRL_PERIOD_MS)

/* ---------------- 串口在线调参 ---------------- */
#define TUNE_RX_BUF             64U
#define TUNE_LINE_IDLE_MS       120U
#define TUNE_PRINT_MS           100U
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
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma){
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
static void DwtCyccnt_Init(void);
static void Control_Update(void);
static void RunButton_Update(void);
static void Tune_Service(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/* ==========================================================================
   循迹控制：转向环 PD + 速度环 PI（转向控制方法三，不使用陀螺仪）
   ========================================================================== */

/* ---- 可在线整定的参数：上电取宏，运行中由串口改写 ---- */
static volatile float tune_line_kp   = LINE_KP;
static volatile float tune_line_kd   = LINE_KD;
static volatile float tune_line_sign = LINE_ERROR_SIGN;
static volatile float tune_turn_lim  = TURN_LIMIT_MPS;
static volatile float tune_inner_rev = TURN_INNER_REVERSE_MPS;
static volatile float tune_target    = SPEED_TARGET_MPS;
static volatile float tune_speed_kp  = SPEED_KP_CCR;
static volatile float tune_speed_ki  = SPEED_KI_CCR;
static volatile float tune_ccr_zero  = MOTOR_CCR_ZERO;
static volatile float tune_ccr_per_mp = MOTOR_CCR_PER_MPS;
static volatile float tune_ccr_max   = (float)MOTOR_CCR_MAX;

/* ---- 循迹传感器 ---- */
static volatile uint16_t line_bits;         /* 12 路原始位图 */
static volatile int16_t  line_error_milli;  /* 偏差，千分之一权重，左负右正 */
static volatile uint8_t  line_valid;        /* 至少一路压线时为 1 */

/* ---- 编码器与车速 ---- */
static volatile int32_t enc_delta_l, enc_delta_r;
static volatile float   speed_l, speed_r;   /* m/s */
static int16_t  enc_hist_l[ENC_AVG_SAMPLES];
static int16_t  enc_hist_r[ENC_AVG_SAMPLES];
static uint8_t  enc_hist_idx;
static int32_t  enc_sum_l, enc_sum_r;

/* ---- 控制器状态 ---- */
static float line_prev_err;    /* 上一拍偏差，用于差分 */
static float line_d_lpf;       /* 偏差变化率低通状态 */
static float speed_i_l, speed_i_r;  /* 每轮速度环积分（CCR 单位） */
static float spd_ramp;         /* 目标速度斜坡状态 */
static volatile float dbg_turn_mps;
static volatile float dbg_err;
static volatile int16_t dbg_ccr_l, dbg_ccr_r;

/* ---- 运行状态 ---- */
static volatile uint8_t run_on;       /* 1 = 发车 */
static uint16_t run_ticks;
static uint16_t lost_ticks;
static uint16_t stall_ticks;
static volatile uint8_t fault_code;   /* 0 无 / 1 堵转 */

/* ---- 串口调参 ---- */
static char tune_rx[TUNE_RX_BUF];
static uint8_t tune_rx_len;
static uint32_t tune_last_byte_tick;
static uint8_t tune_enabled = 1U;

/* ---- 小工具 ---- */
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

/* 多路复用器切换通道后要等 1 us，multiplexer_delay_us() 是用 DWT->CYCCNT 计时的。
   DWT 不使能时 CYCCNT 恒为 0，那个 while 会死等，整个程序卡死。
   原来是在 BMI270 的端口初始化里顺手开的，现在不用陀螺仪了，必须自己开。 */
static void DwtCyccnt_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 按当前极性统计"压线"的路数。 */
static uint16_t Line_ActiveCount(uint16_t bits)
{
  uint16_t n = 0U, ch;
  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    if (MUX_GET_CHANNEL(bits, ch) == LINE_ACTIVE_LEVEL) ++n;
  }
  return n;
}

/* 压线过多时（十字、环岛入口、宽黑区）加权平均会被拉向中间，
   直接用会"看见分叉却当没看见"。这里改用最左和最右压线通道的中点判方向：
     - 对称的十字（最左和最右都在）-> 0，直行
     - 一侧有分叉（环岛入口、岔路）-> 偏向那一侧，跟着拐进去
   返回单位与 line_error_milli 一致（千分之一权重）。 */
static int16_t Line_EdgeError(void)
{
  int16_t first = -1, last = -1;
  uint16_t ch;

  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    if (MUX_GET_CHANNEL(line_bits, ch) == LINE_ACTIVE_LEVEL)
    {
      if (first < 0) first = (int16_t)ch;
      last = (int16_t)ch;
    }
  }
  if ((first < 0) || (last < 0)) return 0;

  {
    int32_t w_first = (first < 6) ? ((int32_t)first - 6L) : ((int32_t)first - 5L);
    int32_t w_last  = (last  < 6) ? ((int32_t)last  - 6L) : ((int32_t)last  - 5L);
    return (int16_t)(((w_first + w_last) * LINE_ERROR_SCALE) / 2L);
  }
}

/* 读 12 路光电管，算出加权平均偏差。
   通道 0 在最左、权重 -6；通道 11 在最右、权重 +6；
   所以偏差为负表示线在车体左侧。 */
static void LineSensor_Update(void)
{
  uint16_t bits;
  int32_t weighted = 0;
  uint16_t n = 0U, ch;

  MUX_get_value(&bits);
  line_bits = bits;

  for (ch = 0U; ch < MULTIPLEXER_CHANNEL_NUM; ++ch)
  {
    if (MUX_GET_CHANNEL(bits, ch) == LINE_ACTIVE_LEVEL)
    {
      int32_t w = (ch < 6U) ? ((int32_t)ch - 6L) : ((int32_t)ch - 5L);
      weighted += w;
      ++n;
    }
  }

  if (n == 0U)
  {
    line_error_milli = 0;
    line_valid = 0U;
    return;
  }

  line_error_milli = (int16_t)((weighted * LINE_ERROR_SCALE) / (int32_t)n);
  line_valid = 1U;
}

/* 10 ms 读一次编码器，30 ms 滑窗求平均车速。 */
static void Encoder_UpdateSpeed(void)
{
  int16_t raw_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t raw_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  raw_l = (int16_t)(ENC_L_SIGN * raw_l);
  raw_r = (int16_t)(ENC_R_SIGN * raw_r);

  enc_delta_l = raw_l;
  enc_delta_r = raw_r;

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

/* 发车键：PB14 低电平按下，每按一次在"待发车 / 运行中"之间切换。 */
static void RunButton_Update(void)
{
  static uint8_t stable = 1U;
  static uint8_t candidate = 1U;
  static uint32_t candidate_since;
  uint32_t now = HAL_GetTick();
  uint8_t raw = (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) == GPIO_PIN_RESET) ? 0U : 1U;

  if (raw != candidate)
  {
    candidate = raw;
    candidate_since = now;
  }
  else if ((candidate != stable) && ((now - candidate_since) >= 20U))
  {
    stable = candidate;
    if (stable == 0U) /* 按下 */
    {
      run_on = (run_on != 0U) ? 0U : 1U;
      fault_code = 0U;
      run_ticks = 0U;
      lost_ticks = 0U;
      stall_ticks = 0U;
      line_prev_err = 0.0f;
      line_d_lpf = 0.0f;
      speed_i_l = 0.0f;
      speed_i_r = 0.0f;
      spd_ramp = 0.0f;
      dbg_turn_mps = 0.0f;
      dbg_err = 0.0f;
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
    }
  }
}

/* 每 10 ms 调用一次：转向环 PD 算左右轮速度差，速度环 PI 把每轮拉到目标值。 */
static void Control_Update(void)
{
  const float dt = CTRL_DT;
  float err = 0.0f;         /* 偏差，正 = 线在车体右侧 */
  float deriv;
  float turn;               /* 转向量，m/s，正 = 右转 */
  float v_target, t_l, t_r;
  float ccr_l, ccr_r;

  /* 未发车 / 故障：停机 */
  if ((run_on == 0U) || (fault_code != 0U))
  {
    stall_ticks = 0U;
    lost_ticks = 0U;
    speed_i_l = 0.0f;
    speed_i_r = 0.0f;
    line_prev_err = 0.0f;
    line_d_lpf = 0.0f;
    spd_ramp = 0.0f;
    dbg_turn_mps = 0.0f;
    dbg_err = 0.0f;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    return;
  }

  if (run_ticks < 0xFFFFU) ++run_ticks;

  /* ---------- 偏差 ---------- */
  if (line_valid == 0U)
  {
    if (lost_ticks < LINE_LOST_STOP_TICKS) ++lost_ticks;
    if (lost_ticks >= LINE_LOST_STOP_TICKS)
    {
      /* 丢线超时：停车等待（不盲目往前冲） */
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
      speed_i_l = 0.0f;
      speed_i_r = 0.0f;
      line_prev_err = 0.0f;
      line_d_lpf = 0.0f;
      spd_ramp = 0.0f;
      dbg_turn_mps = 0.0f;
      return;
    }
    /* 短暂丢线（十字路口 / 虚线 / 折角）：保持上一次偏差继续走几十毫秒 */
    err = line_prev_err;
    deriv = 0.0f;
    line_d_lpf *= LINE_D_LPF_ALPHA;
  }
  else
  {
    int16_t e_milli = line_error_milli;
    uint8_t wide = (Line_ActiveCount(line_bits) >= LINE_WIDE_ACTIVE_COUNT) ? 1U : 0U;

    if (wide) e_milli = Line_EdgeError();

    err = tune_line_sign * ((float)e_milli / (float)LINE_ERROR_SCALE);
    /* 宽黑区里偏差本来就会突变，D 项会给出吓人的反向脉冲，直接掐掉 */
    deriv = wide ? 0.0f : ((err - line_prev_err) / dt);
    lost_ticks = 0U;
    line_prev_err = err;

    /* 光电管是离散的，单步差分会有很大量化尖峰，这里再低通一次 */
    line_d_lpf = (LINE_D_LPF_ALPHA * line_d_lpf) +
                 ((1.0f - LINE_D_LPF_ALPHA) * deriv);
  }
  dbg_err = err;

  /* ---------- 转向环：PD -> 左右轮速度差 ---------- */
  turn = (tune_line_kp * err) + (tune_line_kd * line_d_lpf);
  turn = Limit(turn, -tune_turn_lim, tune_turn_lim);
  dbg_turn_mps = turn;

  /* ---------- 目标速度走斜坡，避免发车瞬间满油门打滑 ---------- */
  if (spd_ramp < tune_target)
  {
    spd_ramp += SPEED_ACCEL_MPS2 * dt;
    if (spd_ramp > tune_target) spd_ramp = tune_target;
  }
  else if (spd_ramp > tune_target)
  {
    spd_ramp -= SPEED_ACCEL_MPS2 * dt;
    if (spd_ramp < tune_target) spd_ramp = tune_target;
  }

  v_target = spd_ramp;

  /* turn > 0 表示右转：左轮加速、右轮减速（可以减到反转为负） */
  t_l = Limit(v_target + turn, -tune_inner_rev, SPEED_LIMIT_MPS);
  t_r = Limit(v_target - turn, -tune_inner_rev, SPEED_LIMIT_MPS);

  /* ---------- 速度环：每轮独立 PI，输出 CCR + 方向 ---------- */
  {
    float dir_l = (t_l < 0.0f) ? -1.0f : 1.0f;
    float dir_r = (t_r < 0.0f) ? -1.0f : 1.0f;
    float mag_t_l = AbsF(t_l);
    float mag_t_r = AbsF(t_r);
    /* 实测速度投影到"命令方向"上，反转时也是正的，PI 就能统一处理 */
    float e_l = mag_t_l - (speed_l * dir_l);
    float e_r = mag_t_r - (speed_r * dir_r);

    if (mag_t_l <= 0.0f)
    {
      ccr_l = 0.0f;
      speed_i_l = 0.0f;
    }
    else
    {
      speed_i_l = Limit(speed_i_l + (e_l * dt), -SPEED_I_LIMIT_CCR, SPEED_I_LIMIT_CCR);
      ccr_l = tune_ccr_zero + (tune_ccr_per_mp * mag_t_l) +
              (tune_speed_kp * e_l) + (tune_speed_ki * speed_i_l);
    }

    if (mag_t_r <= 0.0f)
    {
      ccr_r = 0.0f;
      speed_i_r = 0.0f;
    }
    else
    {
      speed_i_r = Limit(speed_i_r + (e_r * dt), -SPEED_I_LIMIT_CCR, SPEED_I_LIMIT_CCR);
      ccr_r = tune_ccr_zero + (tune_ccr_per_mp * mag_t_r) +
              (tune_speed_kp * e_r) + (tune_speed_ki * speed_i_r);
    }

    ccr_l = Limit(ccr_l, 0.0f, tune_ccr_max);
    ccr_r = Limit(ccr_r, 0.0f, tune_ccr_max);

    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin,
                      (dir_l > 0.0f) ? L_DIR_FORWARD : L_DIR_REVERSE);
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin,
                      (dir_r > 0.0f) ? R_DIR_FORWARD : R_DIR_REVERSE);

    TIM1->CCR1 = (uint16_t)ccr_l;
    TIM1->CCR2 = (uint16_t)ccr_r;
    dbg_ccr_l = (int16_t)ccr_l;
    dbg_ccr_r = (int16_t)ccr_r;

    /* ---------- 堵转 / 编码器方向反接保护 ---------- */
    /* 命令正转却测到明显负速度（或反之），说明编码器符号或方向脚接反了。
       这种情况速度环会正反馈、越跑越快，必须立刻停机。 */
    {
      float amax = AbsF(speed_l);
      float atmp = AbsF(speed_r);
      uint8_t bad_sign = 0U;

      if (atmp > amax) amax = atmp;
      if ((mag_t_l > 0.05f) && (ccr_l >= STALL_CCR_MIN) &&
          ((speed_l * dir_l) < -STALL_SPEED_MPS)) bad_sign = 1U;
      if ((mag_t_r > 0.05f) && (ccr_r >= STALL_CCR_MIN) &&
          ((speed_r * dir_r) < -STALL_SPEED_MPS)) bad_sign = 1U;

      if ((run_ticks > RUN_GRACE_TICKS) &&
          (bad_sign ||
           (((ccr_l >= STALL_CCR_MIN) || (ccr_r >= STALL_CCR_MIN)) &&
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

/* ---- 串口在线调参：形如 lkp=0.1 / spd=0.4 / l=-1 / show ----
   串口助手不发换行也可以，按 120 ms 空闲收尾。 */
static void Tune_Apply(const char *name, uint8_t len, float v)
{
  char nm[8];
  uint8_t i;

  if (len >= sizeof(nm)) { printf("err,name\r\n"); return; }
  for (i = 0U; i < len; ++i) nm[i] = name[i];
  nm[len] = '\0';

  if      ((len == 3U) && (nm[0]=='l') && (nm[1]=='k') && (nm[2]=='p')) tune_line_kp   = v;
  else if ((len == 3U) && (nm[0]=='l') && (nm[1]=='k') && (nm[2]=='d')) tune_line_kd   = v;
  else if ((len == 3U) && (nm[0]=='s') && (nm[1]=='k') && (nm[2]=='p')) tune_speed_kp   = v;
  else if ((len == 3U) && (nm[0]=='s') && (nm[1]=='k') && (nm[2]=='i')) tune_speed_ki   = v;
  else if ((len == 3U) && (nm[0]=='s') && (nm[1]=='p') && (nm[2]=='d')) tune_target     = v;
  else if ((len == 3U) && (nm[0]=='t') && (nm[1]=='l') && (nm[2]=='m')) tune_turn_lim   = v;
  else if ((len == 2U) && (nm[0]=='r') && (nm[1]=='v')) tune_inner_rev  = v;
  else if ((len == 2U) && (nm[0]=='c') && (nm[1]=='z')) tune_ccr_zero    = v;
  else if ((len == 2U) && (nm[0]=='c') && (nm[1]=='p')) tune_ccr_per_mp  = v;
  else if ((len == 2U) && (nm[0]=='c') && (nm[1]=='x')) tune_ccr_max     = v;
  else if ((len == 1U) && (nm[0]=='l')) tune_line_sign = (v < 0.0f) ? -1.0f : 1.0f;
  else { printf("err,name\r\n"); return; }

  /* 兜一层限幅，防止手误把参数输成离谱的值后车子失控 */
  tune_line_kp   = Limit(tune_line_kp, 0.0f, 1.0f);
  tune_line_kd   = Limit(tune_line_kd, 0.0f, 0.05f);
  tune_speed_kp  = Limit(tune_speed_kp, 0.0f, 2000.0f);
  tune_speed_ki  = Limit(tune_speed_ki, 0.0f, 10000.0f);
  tune_target    = Limit(tune_target, 0.0f, SPEED_LIMIT_MPS);
  tune_turn_lim  = Limit(tune_turn_lim, 0.0f, SPEED_LIMIT_MPS);
  tune_inner_rev = Limit(tune_inner_rev, 0.0f, SPEED_LIMIT_MPS);
  tune_ccr_zero  = Limit(tune_ccr_zero, 0.0f, 7199.0f);
  tune_ccr_per_mp = Limit(tune_ccr_per_mp, 0.0f, 20000.0f);
  tune_ccr_max   = Limit(tune_ccr_max, 0.0f, 7199.0f);

  printf("set,%s,%ld\r\n", nm, (long)(v * 1000.0f));
}

static void Tune_Print(void)
{
  printf("cfg,lkp=%ld,lkd=%ld,spd=%ld,tlm=%ld,rv=%ld,skp=%ld,ski=%ld,cz=%ld,cp=%ld,cx=%ld,l=%ld\r\n",
         (long)(tune_line_kp * 1000.0f),
         (long)(tune_line_kd * 10000.0f),
         (long)(tune_target * 1000.0f),
         (long)(tune_turn_lim * 1000.0f),
         (long)(tune_inner_rev * 1000.0f),
         (long)tune_speed_kp,
         (long)tune_speed_ki,
         (long)tune_ccr_zero,
         (long)tune_ccr_per_mp,
         (long)tune_ccr_max,
         (long)tune_line_sign);
}

static void Tune_Execute(char *s, uint8_t len)
{
  uint8_t i = 0U, nl = 0U, k;
  float ipart = 0.0f, fpart = 0.0f, fdiv = 0.1f;
  uint8_t neg = 0U, digits = 0U, dot = 0U;

  while ((i < len) && ((s[i] == ' ') || (s[i] == '\t'))) ++i;
  while ((len > i) && ((s[len-1] == ' ') || (s[len-1] == '\t') || (s[len-1] == '\r'))) --len;
  if (len <= i) return;

  if (((len - i) == 4U) && (s[i]=='s') && (s[i+1]=='h') && (s[i+2]=='o') && (s[i+3]=='w'))
  {
    Tune_Print();
    return;
  }

  while (((i + nl) < len) && (s[i+nl] != '=') && (s[i+nl] != ' ')) ++nl;
  if (nl == 0U) { printf("err,name\r\n"); return; }

  k = (uint8_t)(i + nl);
  while ((k < len) && ((s[k] == '=') || (s[k] == ' '))) ++k;

  for (; k < len; ++k)
  {
    char c = s[k];
    if (c == '-') { neg = 1U; }
    else if (c == '.') { dot = 1U; }
    else if ((c >= '0') && (c <= '9'))
    {
      if (dot == 0U)
      {
        ipart = (ipart * 10.0f) + (float)(c - '0');
      }
      else
      {
        fpart += fdiv * (float)(c - '0');
        fdiv *= 0.1f;
      }
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
  if ((now - tune_last_byte_tick) < TUNE_LINE_IDLE_MS) return;

  tune_rx[tune_rx_len] = '\0';
  Tune_Execute(tune_rx, tune_rx_len);
  tune_rx_len = 0U;
}

/* 串口收到的每个字节先进缓冲区，按 120 ms 空闲判定一行结束。 */
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
      tune_last_byte_tick = HAL_GetTick();
    }
    (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
  }
}

/* TIM2 是 1 kHz 时基，分频到 10 ms 作为控制周期。 */
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
  /* 编码器：TIM3 = 左轮，TIM4 = 右轮，正交计数 */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();

  /* 光电管读取依赖 DWT 计时，必须最先打开 */
  DwtCyccnt_Init();

  /* 1 kHz 时基：控制周期由 HAL_TIM_PeriodElapsedCallback 分频到 10 ms */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();

  /* 串口接收中断，用于在线调参 */
  if (tune_enabled) (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);

  /* 电机先确保停止 */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FORWARD);
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FORWARD);

  printf("ready,line_follow\r\n");
  Tune_Print();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 光电管与发车键 */
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

    /* 周期性打印，便于用串口助手观察 */
    {
      static uint32_t print_tick;
      if ((now - print_tick) >= TUNE_PRINT_MS)
      {
        print_tick = now;
        printf("st,%u,%u,%u,%04X,%d,%ld,%ld,%ld,%ld,%d,%d\r\n",
               (unsigned int)run_on,
               (unsigned int)line_valid,
               (unsigned int)fault_code,
               (unsigned int)line_bits,
               (int)line_error_milli,               /* 偏差，千分之一权重 */
               (long)(dbg_err * 1000.0f),           /* 参与控制的偏差 ×1000 */
               (long)(dbg_turn_mps * 1000.0f),      /* 转向量 mm/s，正 = 右转 */
               (long)(speed_l * 1000.0f),           /* 左轮实测 mm/s */
               (long)(speed_r * 1000.0f),           /* 右轮实测 mm/s */
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
  /* 使能 TIM2 更新中断：控制周期就是在这个中断里产生的。
     CubeMX 没有勾选 NVIC，必须在这里补上，否则 HAL_TIM_PeriodElapsedCallback
     永远不会被调用，控制环等于不存在。 */
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
  /* 使能 USART3 中断：串口调参依赖 HAL_UART_Receive_IT。
     CubeMX 未勾选 NVIC，同样必须在这里补上。 */
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
