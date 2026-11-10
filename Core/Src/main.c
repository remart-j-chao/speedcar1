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
/* 陀螺仪调试打印开关。置 1 会引入约 5.4 kB 的浮点 printf 代码，
   MDK-Lite 的 32 kB 链接上限会因此报 L6047U 而链接失败。
   平时保持 0；要串口看数据时改成 1（并注意镜像大小）。 */
#define GYRO_DEBUG 1
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动（首次试车先置0）

/* 整车试跑测试：上电后两轮按 MOTOR_TEST_CCR 转动 MOTOR_TEST_RUN_MS 后自动停车。
   置 0 则不加这段测试逻辑。
   MOTOR_TEST_CCR 即 TIM1 通道比较值，占空比 = CCR / (Period + 1) = CCR / 7200，
   800 约占 11.1%。 */
#define MOTOR_TEST             0
#define MOTOR_TEST_CCR        700U
#define MOTOR_TEST_R_CCR       600U
#define MOTOR_TEST_RUN_MS      3000U
/* 前进时的方向脚电平。若某个轮子反转，把对应的一行改成 GPIO_PIN_RESET。 */
#define L_DIR_FORWARD          GPIO_PIN_RESET
#define R_DIR_FORWARD          GPIO_PIN_SET

/* ---------------- 编码器测速参数（按实际硬件标定） ----------------
   车轮每转一圈计数 = ENCODER_LINES × 4（TI12 四倍频）× ENCODER_GEAR_RATIO
   手推车轮转一圈，累计计数应接近这个值，否则说明线数/传动比填错。
   13 线霍尔直接装车轮轴时一圈只有 52 个计数，10 ms 内只有个位数的计数，
   所以速度用 SPEED_AVG_SAMPLES 个采样做滑动平均，窗口 10 × 10 ms = 100 ms。 */
#define ENCODER_LINES         13.0f     /* 编码器单相每转脉冲数 */
#define ENCODER_GEAR_RATIO    1.0f      /* 编码器轴到车轮的传动比：直接装在车轮轴上取 1.0 */
#define WHEEL_DIAMETER_M      0.065f    /* 车轮直径，单位 m */
#define SPEED_SAMPLE_MS       10U       /* 采样周期，单位 ms（TIM2 为 1 kHz 时基） */
#define SPEED_AVG_SAMPLES     10U       /* 滑动平均窗长（样本数），10 × 10 ms = 100 ms */
#define SPEED_WINDOW_MS       (SPEED_SAMPLE_MS * SPEED_AVG_SAMPLES)
#define ENC_L_SIGN            1         /* 左轮计数方向修正，前进计数为正时保持 1 */
#define ENC_R_SIGN            (-1)      /* 右轮方向与左轮相反，取负统一为前进为正 */

/* 光电管判定为白色的电平；如果实测模块输出相反，将 1U 改为 0U。 */
#define LINE_SENSOR_ACTIVE_LEVEL 1U
#define LINE_SAMPLE_MS          10U
#define LINE_SENSOR_DEBUG       1
#define LINE_PRINT_MS           100U
#define LINE_ERROR_SCALE        1000L
#define RUN_BUTTON_DEBOUNCE_MS  20U
#define RUN_BUTTON_PRESSED      GPIO_PIN_RESET

/* 控制器只在完成实车标定后启用；0 表示不向电机 PWM 写入控制量。 */
#define MOTOR_CONTROL_ENABLE       0U
#define CONTROL_UPDATE_MS          SPEED_SAMPLE_MS
#define CONTROL_TARGET_SPEED_MPS   0.0f
#define CONTROL_MAX_PWM_CCR        700.0f
#define CONTROL_MAX_YAW_RATE_DPS   180.0f

/* 以下增益均为待标定值，默认置零，避免未经调参产生电机输出。 */
#define LINE_PD_KP                 0.0f  /* 目标角速度 / 循迹误差 */
#define LINE_PD_KD                 0.0f  /* 目标角速度 / 循迹误差变化率 */
#define YAW_PID_KP                 0.0f  /* 差速 PWM / 角速度误差 */
#define YAW_PID_KI                 0.0f  /* 差速 PWM / 角速度误差积分 */
#define YAW_PID_KD                 0.0f  /* 差速 PWM / 角速度误差变化率 */
#define SPEED_PI_KP                0.0f  /* 基础 PWM / 速度误差 */
#define SPEED_PI_KI                0.0f  /* 基础 PWM / 速度误差积分 */
#define YAW_INTEGRAL_LIMIT         500.0f
#define SPEED_INTEGRAL_LIMIT       2.0f

/* 一个滑动窗口内的计数之和对应的车轮线速度，单位 m/s，编译期常量 */
#define ENC_COUNTS_TO_MPS \
  ((3.14159265f * WHEEL_DIAMETER_M) / \
   (ENCODER_LINES * 4.0f * ENCODER_GEAR_RATIO) * (1000.0f / (float)SPEED_WINDOW_MS))

/* 测速结果打印开关。以整数 mm/s 输出，不引入额外的浮点 printf 代码。 */
#define SPEED_DEBUG            1
#define SPEED_PRINT_MS         100U
#define GYRO_PRINT_MS          100U
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

/* 编码器测速结果：TIM2 回调写入，主循环与后续速度环读取 */
static volatile int32_t enc_left_delta;    /* 最近一次采样的脉冲增量（10 ms） */
static volatile int32_t enc_right_delta;
static volatile float   speed_left_mps;    /* 车轮线速度（100 ms 滑窗平均），单位 m/s */
static volatile float   speed_right_mps;
static int16_t  enc_left_hist[SPEED_AVG_SAMPLES];
static int16_t  enc_right_hist[SPEED_AVG_SAMPLES];
static uint16_t enc_hist_index;
static int32_t  enc_left_sum;
static int32_t  enc_right_sum;

/* 循迹状态：通道 0 在最左侧，通道 11 在最右侧。 */
static volatile uint16_t line_sensor_bits;
/* 偏差单位为千分之一传感器权重，左偏为负，右偏为正。 */
static volatile int16_t line_error_milli;
/* 至少有一路传感器检测到白线时为 1；全无信号时为 0。 */
static volatile uint8_t line_sensor_valid;

/* 发车键稳定按下后锁存为 1；当前仅记录状态，不触发电机或风扇。 */
static volatile uint8_t run_button_started;
/* IMU 初始化状态：失败时禁止后续控制计算。 */
static volatile uint8_t imu_ready;

/* 控制器中间量与限幅后的左右轮 PWM，便于调试器查看。 */
static volatile float control_target_yaw_rate_dps;
static volatile float control_turn_pwm;
static volatile float control_base_pwm;
static volatile uint16_t control_left_pwm;
static volatile uint16_t control_right_pwm;

typedef struct
{
  float integral;
  float previous_error;
  uint8_t initialized;
} ControlLoopState;

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
static void Encoder_UpdateSpeed(void);
static void LineSensor_Update(void);
static void RunButton_Update(void);
static void VehicleControl_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* 主循环更新、TIM2 控制回调读取的 IMU 物理量。 */
volatile float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
/* USER CODE END 0 */

/* 读取 12 路光电管并计算白线相对车体中心的位置，不控制电机。 */
static void LineSensor_Update(void)
{
  uint16_t sensor_bits;
  uint16_t active_count = 0U;
  int32_t weighted_position = 0;
  uint16_t channel;

  MUX_get_value(&sensor_bits);
  line_sensor_bits = sensor_bits;

  for (channel = 0U; channel < MULTIPLEXER_CHANNEL_NUM; ++channel)
  {
    if (MUX_GET_CHANNEL(sensor_bits, channel) == LINE_SENSOR_ACTIVE_LEVEL)
    {
      /* 按教程采用左侧 -6 至 -1、右侧 +1 至 +6 的循迹权重。 */
      int32_t weight = (channel < 6U) ? ((int32_t)channel - 6L) : ((int32_t)channel - 5L);
      weighted_position += weight;
      ++active_count;
    }
  }

  if (active_count == 0U)
  {
    line_error_milli = 0;
    line_sensor_valid = 0U;
    return;
  }

  /* 对有效传感器的权重取平均，避免多路同时压线时偏差被放大。 */
  line_error_milli = (int16_t)((weighted_position * LINE_ERROR_SCALE) / active_count);
  line_sensor_valid = 1U;
}

/* 将控制器状态清零，防止停机或丢线期间积分继续累积。 */
static void VehicleControl_Reset(ControlLoopState *line_state,
                                 ControlLoopState *yaw_state,
                                 ControlLoopState *speed_state)
{
  line_state->integral = 0.0f;
  line_state->previous_error = 0.0f;
  line_state->initialized = 0U;
  yaw_state->integral = 0.0f;
  yaw_state->previous_error = 0.0f;
  yaw_state->initialized = 0U;
  speed_state->integral = 0.0f;
  speed_state->previous_error = 0.0f;
  speed_state->initialized = 0U;

  control_target_yaw_rate_dps = 0.0f;
  control_turn_pwm = 0.0f;
  control_base_pwm = 0.0f;
  control_left_pwm = 0U;
  control_right_pwm = 0U;

  /* 只有显式启用控制器后，安全条件失效才会将电机 PWM 清零。 */
  if (MOTOR_CONTROL_ENABLE != 0U)
  {
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
  }
}

/* 限制控制量，避免积分饱和或请求超出允许的 PWM / 角速度范围。 */
static float Control_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
    return minimum;
  if (value > maximum)
    return maximum;
  return value;
}

/*
 * 三级控制计算（每 10 ms 调用一次）：
 * 光电管位置 PD 生成目标偏航角速度，陀螺仪 PID 生成左右轮差速，
 * 编码器速度 PI 生成基础油门。默认仅计算调试量，不写电机 PWM。
 */
static void VehicleControl_Update(void)
{
  static ControlLoopState line_state;
  static ControlLoopState yaw_state;
  static ControlLoopState speed_state;
  const float dt = (float)CONTROL_UPDATE_MS / 1000.0f;
  float line_error;
  float line_derivative;
  float yaw_error;
  float yaw_derivative;
  float speed_error;
  float average_speed;
  float left_pwm;
  float right_pwm;

  /* 未按下发车键、光电管丢线或 IMU 未初始化时，不允许控制输出。 */
  if ((run_button_started == 0U) ||
      (line_sensor_valid == 0U) ||
      (imu_ready == 0U))
  {
    VehicleControl_Reset(&line_state, &yaw_state, &speed_state);
    return;
  }

  /* 将循迹误差从千分之一权重还原为传感器权重。 */
  line_error = (float)line_error_milli / (float)LINE_ERROR_SCALE;
  line_derivative = line_state.initialized ?
                    (line_error - line_state.previous_error) / dt : 0.0f;
  control_target_yaw_rate_dps = Control_Clamp(
      (LINE_PD_KP * line_error) + (LINE_PD_KD * line_derivative),
      -CONTROL_MAX_YAW_RATE_DPS, CONTROL_MAX_YAW_RATE_DPS);
  line_state.previous_error = line_error;
  line_state.initialized = 1U;

  /* 目标角速度与 BMI270 Z 轴角速度之差由 PID 转为左右轮差速量。 */
  yaw_error = control_target_yaw_rate_dps - gyro_z;
  yaw_derivative = yaw_state.initialized ?
                   (yaw_error - yaw_state.previous_error) / dt : 0.0f;
  yaw_state.integral = Control_Clamp(yaw_state.integral + yaw_error * dt,
                                     -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);
  control_turn_pwm = Control_Clamp(
      (YAW_PID_KP * yaw_error) +
      (YAW_PID_KI * yaw_state.integral) +
      (YAW_PID_KD * yaw_derivative),
      -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);
  yaw_state.previous_error = yaw_error;
  yaw_state.initialized = 1U;

  /* 平均左右轮速度与目标速度之差由 PI 转为公共基础 PWM。 */
  average_speed = (speed_left_mps + speed_right_mps) * 0.5f;
  speed_error = CONTROL_TARGET_SPEED_MPS - average_speed;
  speed_state.integral = Control_Clamp(speed_state.integral + speed_error * dt,
                                       -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
  control_base_pwm = Control_Clamp(
      (SPEED_PI_KP * speed_error) + (SPEED_PI_KI * speed_state.integral),
      0.0f, CONTROL_MAX_PWM_CCR);
  speed_state.previous_error = speed_error;
  speed_state.initialized = 1U;

  left_pwm = Control_Clamp(control_base_pwm - control_turn_pwm,
                           0.0f, CONTROL_MAX_PWM_CCR);
  right_pwm = Control_Clamp(control_base_pwm + control_turn_pwm,
                            0.0f, CONTROL_MAX_PWM_CCR);
  control_left_pwm = (uint16_t)left_pwm;
  control_right_pwm = (uint16_t)right_pwm;

  /* 电机输出需人工完成标定后才可启用；当前编译配置保持关闭。 */
  if (MOTOR_CONTROL_ENABLE != 0U)
  {
    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FORWARD);
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FORWARD);
    TIM1->CCR1 = control_left_pwm;
    TIM1->CCR2 = control_right_pwm;
  }
}

/* 非阻塞消抖：PB14 按下电平持续 20 ms 后锁存发车状态。 */
static void RunButton_Update(void)
{
  static GPIO_PinState candidate_state = GPIO_PIN_SET;
  static GPIO_PinState stable_state = GPIO_PIN_SET;
  static uint32_t candidate_since;
  GPIO_PinState raw_state = HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin);
  uint32_t now = HAL_GetTick();

  /* 原始电平发生变化时重新开始计时，等待机械触点稳定。 */
  if (raw_state != candidate_state)
  {
    candidate_state = raw_state;
    candidate_since = now;
    return;
  }

  if ((raw_state != stable_state) &&
      ((uint32_t)(now - candidate_since) >= RUN_BUTTON_DEBOUNCE_MS))
  {
    stable_state = raw_state;

    /* 本项目按低电平按下处理；按键只设置状态，不启动任何动力输出。 */
    if ((stable_state == RUN_BUTTON_PRESSED) && (run_button_started == 0U))
      run_button_started = 1U;
  }
}

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
  /* 启动编码器（TIM3 左轮 / TIM4 右轮）与 1 kHz 周期时基；
     计数器由 TIM2 中断每 SPEED_SAMPLE_MS 读取并清零。 */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    Error_Handler();
  {
    int8_t bmi_init_result = dodo_BMI270_init();
    imu_ready = (bmi_init_result == 0) ? 1U : 0U;
    if (imu_ready == 0U)
      printf("imu_init,fail,%d\r\n", (int)bmi_init_result);
  }
#if MOTOR_TEST
  uint32_t motor_test_start;
  HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FORWARD);
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FORWARD);
  TIM1->CCR1 = MOTOR_TEST_CCR;
  TIM1->CCR2 = MOTOR_TEST_R_CCR;
  motor_test_start = HAL_GetTick();
#endif
  //pwm示例
  //TIM1->CCR1 = 2000;
  //TIM1->CCR2 = 2000;
  // 延时5000秒，等待无刷电调初始化
  // TIM1->PSC = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RunButton_Update();

    /* 仅在首次确认按下后报告状态，供后续控制逻辑使用。 */
    if (run_button_started != 0U)
    {
      static uint8_t run_state_reported;
      if (run_state_reported == 0U)
      {
        printf("run,1\r\n");
        run_state_reported = 1U;
      }
    }

    /* 每 10 ms 更新一次循迹数据；此步骤只采样，不改变左右轮 PWM。 */
    {
      static uint32_t line_sample_tick;
      uint32_t now = HAL_GetTick();
      if ((now - line_sample_tick) >= LINE_SAMPLE_MS)
      {
        line_sample_tick = now;
        LineSensor_Update();
      }
    }
#if MOTOR_TEST
    if ((HAL_GetTick() - motor_test_start) >= MOTOR_TEST_RUN_MS)
    {
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
    }
#endif
#if LINE_SENSOR_DEBUG
    {
      static uint32_t line_print_tick;
      if ((HAL_GetTick() - line_print_tick) >= LINE_PRINT_MS)
      {
        line_print_tick = HAL_GetTick();
        printf("line,%u%u%u%u%u%u%u%u%u%u%u%u,%u,%d\r\n",
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 0U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 1U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 2U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 3U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 4U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 5U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 6U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 7U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 8U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 9U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 10U),
               (unsigned int)MUX_GET_CHANNEL(line_sensor_bits, 11U),
               (unsigned int)line_sensor_valid, (int)line_error_milli);
      }
    }
#endif
#if SPEED_DEBUG
    {
      static uint32_t speed_print_tick;
      if ((HAL_GetTick() - speed_print_tick) >= SPEED_PRINT_MS)
      {
        speed_print_tick = HAL_GetTick();
        printf("enc,%ld,%ld,%ld,%ld\r\n",
               (long)enc_left_delta, (long)enc_right_delta,
               (long)(speed_left_mps * 1000.0f), (long)(speed_right_mps * 1000.0f));
      }
    }
#endif
#if SPEED_DEBUG
    {
      static uint32_t control_print_tick;
      if ((HAL_GetTick() - control_print_tick) >= SPEED_PRINT_MS)
      {
        control_print_tick = HAL_GetTick();
        printf("ctrl,%u,%u,%u,%ld,%ld,%ld,%u,%u\r\n",
               (unsigned int)run_button_started,
               (unsigned int)line_sensor_valid,
               (unsigned int)imu_ready,
               (long)(control_target_yaw_rate_dps * 1000.0f),
               (long)(control_turn_pwm * 1000.0f),
               (long)(control_base_pwm * 1000.0f),
               (unsigned int)control_left_pwm,
               (unsigned int)control_right_pwm);
      }
    }
#endif
    // 以下为陀螺仪使用示例
    dodo_BMI270_get_data();                         // 调用此函数会更新陀螺仪数据
    gyro_x = BMI270_gyro_transition(BMI270_gyro_x); // 将原始陀螺仪数据转换为物理值，单位为度每秒
    gyro_y = BMI270_gyro_transition(BMI270_gyro_y);
    gyro_z = BMI270_gyro_transition(BMI270_gyro_z);
    accel_x = BMI270_acc_transition(BMI270_accel_x); // 将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
    accel_y = BMI270_acc_transition(BMI270_accel_y);
    accel_z = BMI270_acc_transition(BMI270_accel_z);
    if (GYRO_DEBUG)
    {
      static uint32_t gyro_print_tick;
      if ((HAL_GetTick() - gyro_print_tick) >= GYRO_PRINT_MS)
      {
        gyro_print_tick = HAL_GetTick();
        printf("%f,%f,%f,%f,%f,%f\r\n",gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z);
      }
    }
    // 以下为读取光电管的示例（从左到右编号0~11）
    //  uint16_t mux_value;
    //  MUX_get_value(&mux_value);
    //  for(int i=0;i<=11;i++){
    //    printf("%d,",MUX_GET_CHANNEL(mux_value,i));//获取第i个光电管的数值并输出
    //  }
    //  printf("\n");
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
/* 读一次两个编码器的计数增量。计数器是 16 位，必须按 int16_t 解读，
   否则反转时回绕出来的 65535 会被当成很大的正数。
   13 线霍尔一圈只有 52 个计数，单次 10 ms 的增量太小，速度用滑窗求和后换算；
   滑窗长度 SPEED_AVG_SAMPLES × SPEED_SAMPLE_MS = 100 ms，起步后 100 ms 内偏小。 */
static void Encoder_UpdateSpeed(void)
{
  int16_t left  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  left  = (int16_t)(ENC_L_SIGN * left);
  right = (int16_t)(ENC_R_SIGN * right);

  /* 滑窗：减去最老的一次采样，再写入本次采样 */
  enc_left_sum  += left  - enc_left_hist[enc_hist_index];
  enc_right_sum += right - enc_right_hist[enc_hist_index];
  enc_left_hist[enc_hist_index]  = left;
  enc_right_hist[enc_hist_index] = right;
  if (++enc_hist_index >= SPEED_AVG_SAMPLES)
    enc_hist_index = 0U;

  enc_left_delta  = left;
  enc_right_delta = right;
  speed_left_mps  = (float)enc_left_sum  * ENC_COUNTS_TO_MPS;
  speed_right_mps = (float)enc_right_sum * ENC_COUNTS_TO_MPS;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t speed_divider;

  if (htim->Instance != TIM2)
    return;

  if (++speed_divider < SPEED_SAMPLE_MS)
    return;
  speed_divider = 0U;

  Encoder_UpdateSpeed();
  VehicleControl_Update();
}

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
