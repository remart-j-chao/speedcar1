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
#define GYRO_DEBUG 0
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动（首次试车先置0）

/* 整车试跑测试：上电后两轮按 MOTOR_TEST_CCR 转动 MOTOR_TEST_RUN_MS 后自动停车。
   置 0 则不加这段测试逻辑。
   MOTOR_TEST_CCR 即 TIM1 通道比较值，占空比 = CCR / (Period + 1) = CCR / 7200，
   800 约占 11.1%。 */
#define MOTOR_TEST             0
#define MOTOR_TEST_CCR         310U
#define MOTOR_TEST_R_CCR       300U
#define MOTOR_TEST_RUN_MS      3000U
/* 前进时的方向脚电平。若某个轮子反转，把对应的一行改成 GPIO_PIN_RESET。 */
#define L_DIR_FORWARD          GPIO_PIN_RESET
#define R_DIR_FORWARD          GPIO_PIN_SET
/* 反向电平由正向电平推导，改上面两行时不用再动这里。 */
#define L_DIR_REVERSE          ((L_DIR_FORWARD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define R_DIR_REVERSE          ((R_DIR_FORWARD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)

/* ---------------- 编码器标定（按实际硬件实测） ----------------
   速度换算只依赖“每米计数”这一个标定值，不要再用
   线数 × 4 × 传动比 ÷ 轮周长 去推算：三个参数任一估错，结果会差好几倍。
   本项目实测约 3.77 万计数/米，比“编码器直连车轮”的假设大 148 倍。

   标定方法：
   1) 上电后按一次发车键（odom 清零）
   2) 沿直线推行恰好 1 m
   3) 读串口最后一行 odom 的两个数值，取平均即为每米计数
   4) 把结果填入 ENCODER_COUNTS_PER_METER，重新编译 */
#define ENCODER_COUNTS_PER_METER  37385.0f
#define SPEED_SAMPLE_MS       10U       /* 采样周期，单位 ms（TIM2 为 1 kHz 时基） */
#define SPEED_AVG_SAMPLES     3U        /* 滑动平均窗长（样本数），3 × 10 ms = 30 ms。
                                           窗口越长速度越平滑但滞后越大：100 ms 窗口配 10 ms
                                           控制周期会让速度环"拿旧数据做快速决策"，表现为一顿一顿。
                                           3.7 万计数/米下 30 ms 已有 400+ 计数，精度足够。 */
#define SPEED_WINDOW_MS       (SPEED_SAMPLE_MS * SPEED_AVG_SAMPLES)
#define ENC_L_SIGN            1         /* 左轮计数方向修正，前进计数为正时保持 1 */
#define ENC_R_SIGN            (-1)      /* 右轮方向与左轮相反，取负统一为前进为正 */

/* 光电管判定为白色的电平；如果实测模块输出相反，将 1U 改为 0U。 */
#define LINE_SENSOR_ACTIVE_LEVEL 1U
/* 同时压线的路数达到这个值就判为十字路口 / 横向宽线：保持上一次偏差直穿，
   而不是用加权平均算出一个假偏差（实测 011111100000 会得出 -2333，
   车会朝空地打舵）。正常赛道线宽只覆盖 1~3 路。 */
#define LINE_JUNCTION_ACTIVE_MIN 5U
#define LINE_SAMPLE_MS          10U
#define LINE_SENSOR_DEBUG       1
#define LINE_PRINT_MS           100U
#define LINE_ERROR_SCALE        1000L
#define RUN_BUTTON_DEBOUNCE_MS  20U
#define RUN_BUTTON_PRESSED      GPIO_PIN_RESET

/* 控制器只在完成实车标定后启用；0 表示不向电机 PWM 写入控制量。 */
#define MOTOR_CONTROL_ENABLE       1U
#define CONTROL_UPDATE_MS          SPEED_SAMPLE_MS
/* 速度环目标：0.30 m/s —— 这已经是本车能命令到的最低速度。
   原因：死区补偿把任何非零 PWM 都映射到至少 CCR 310，而实测 CCR 310~317
   对应的轮速就是 0.29~0.31 m/s。所以 0.28 以下的车速在硬件上不存在，
   设成 0.2 只会让速度环在 CCR 0 和 310 之间开开关，车一顿一顿反而更糟。
   想要真正更慢只有三条路：提高 PWM 频率、换更大减速比、或做 PWM 抖动
   （在 CCR 310 与 0 之间快速切换取平均）。 */
#define CONTROL_TARGET_SPEED_MPS   0.30f
#define CONTROL_MAX_PWM_CCR        700.0f
/* 目标角速度上限。这条是最容易被忽略的"隐形天花板"：
   lkp=35、误差 6 时目标是 210 dps，超过 180 就被削平，
   于是把 lkp 从 35 加到 55（最高 330 dps）完全没有区别 —— 全被削掉。
   急弯需要 200~300 dps，所以放到 300。
   真正的物理上限由差速和外侧轮最高转速决定，由 YAW_DIFF_LIMIT_MPS 兜底。 */
#define CONTROL_MAX_YAW_RATE_DPS   300.0f

/* 陀螺仪安装方向修正：让“左转（俯视逆时针回正）”对应正角速度。
   判据不靠手转手感，而看闭环符号：yt 阶跃测试里 ykp=0 时 diff≈+0.22 m/s，
   若串口 yaw 报的是负值，说明 plant 增益为负、角速度环是正反馈，必须取反。
   2026-11-12 实测：diff=+0.216 时 yaw 报 -77 dps，故取 +1.0f。 */
#define YAW_RATE_SIGN             1.0f
/* 扣除零偏后的静止死区，单位 dps；残余漂移小于该值时按 0 处理。
   1.0 dps 可以挡掉台架/地面静止时的大部分噪声，又不会明显损失转向灵敏度。 */
#define GYRO_DEADBAND_DPS          1.0f
/* 零偏微调的默认值（dps）：自动校准后若仍有残余，把它写在这里固化。
   运行中也可用 ofs=±x 在线调整；save 命令会把它写入 Flash。 */
#define GYRO_TRIM_DPS              0.0f
/* 上电时是否从 Flash 加载已保存的参数（1=加载，0=只用宏默认值）。 */
#define TUNE_LOAD_ON_BOOT          1
/* 上电后先静置一段时间等滤波器稳定，再用静止期样本求零偏均值。
   累积样本数设上限，避免长时间等待后浮点累加和过大而损失精度。 */
#define GYRO_BIAS_SETTLE_MS        500U
#define GYRO_BIAS_MIN_SAMPLES      100U
#define GYRO_BIAS_MAX_SAMPLES      20000U

/* 短暂丢线时保持最后一次偏差继续盲开的时间，超时仍无信号才停机。 */
#define LINE_LOST_HOLD_MS          300U
#define LINE_LOST_HOLD_TICKS       (LINE_LOST_HOLD_MS / CONTROL_UPDATE_MS)
/* 丢线期间保持的偏差按此系数逐周期衰减（指数衰减，每 10 ms 乘一次）。
   0.93 时：100 ms 后剩 48%，200 ms 剩 23%，300 ms 只剩 11%。
   这样做的好处是两头都对：
     - 十字路口那种 50~100 ms 的短暂丢线，转向量基本保留，能直穿过去；
     - 冲出弯道后的长丢线，转向量迅速衰减到接近 0，车会自己摆正去重新找线，
       而不是像固定保持那样「以满舵转 300 ms」把车甩到赛道外面。
   实测日志里丢线后 target 一直是 300、diff 0.70，紧接着就彻底丢线并停机，
   正是这个固定保持造成的。 */
#define LINE_LOST_DECAY            0.93f
/* 衰减期结束后进入「直行搜索」，再给这么长时间找回线才真正停机。
   停车是最后手段：比赛里车一旦停住，这一圈就废了，直行搜索远比停车好。 */
#define LINE_LOST_SEARCH_MS        1200U
#define LINE_LOST_SEARCH_TICKS     ((LINE_LOST_HOLD_MS + LINE_LOST_SEARCH_MS) / CONTROL_UPDATE_MS)
/* 直行搜索时的车速比例：直接用全速，低速时轮子会掉进死区开始抖。 */
#define LINE_SEARCH_SPEED_SCALE    1.00f

/* 发车后目标速度的爬升率，避免目标速度阶跃造成打滑和电流冲击。 */
#define CONTROL_ACCEL_LIMIT_MPS2   1.0f

/* 实测起转 CCR（死区）：低于该占空比轮子不转。填 0 表示不做死区补偿。
   标定方法：逐级加大 CCR，记录轮子刚开始转动的最小值，左右轮分别填。 */
#define MOTOR_DEADZONE_L           310.0f
#define MOTOR_DEADZONE_R           300.0f

/* 堵转保护：指令 PWM 已超过该值、两轮速度都接近 0 并持续一段时间，判定卡死停机。 */
#define MOTOR_STALL_PWM_MIN        200U
#define MOTOR_STALL_SPEED_MPS      0.02f
#define MOTOR_STALL_MS             800U
#define MOTOR_STALL_TICKS          (MOTOR_STALL_MS / CONTROL_UPDATE_MS)
#define MOTOR_RUN_GRACE_MS         1000U
#define MOTOR_GRACE_TICKS          (MOTOR_RUN_GRACE_MS / CONTROL_UPDATE_MS)

/* 角速度数据超时：主循环被阻塞或 SPI 卡死时不允许控制环继续使用旧值。 */
#define GYRO_SAMPLE_TIMEOUT_MS     100U

/* 故障码：非 0 时控制环停机，并由主循环打印一次。 */
#define FAULT_NONE                 0U
#define FAULT_STALL                1U
#define FAULT_IMU                  2U
#define FAULT_DIR                  3U

/* 故障瞬间的快照，供主循环打印，避免在中断里直接 printf。 */
static volatile int32_t fault_speed_l_mmps;
static volatile int32_t fault_speed_r_mmps;
static volatile uint16_t fault_ccr_l;
static volatile uint16_t fault_ccr_r;

/* 编码器方向接反时每个轮子的速度环会正反馈失控（指令正向、测到反向 → 输出越给越大），
   所以一旦在正向目标下测到持续反向速度就立即停机。 */
#define MOTOR_REVERSE_SPEED_MPS    (-0.05f)

/* 独立看门狗：1 表示启用（LSI 约 40 kHz、超时约 0.8 s）。
   一旦启动无法关闭，调试器停在断点会触发复位，调试阶段保持 0。 */
#define WATCHDOG_ENABLE            0

/* 以下增益均为实车标定值；可在运行中用在线整定命令改写。
   角速度 Kp 注意尺度：turn→差速换算为 YAW_DIFF_PWM_PER_MPS=5500，
   kp=10 时 10 dps 误差只产生 0.018 m/s 差速（闭环增益约 0.8，稳定区）。
   历史上按旧尺度(550)调出的 80 在本尺度下相当于 800，会让车身持续振荡。 */
#define LINE_PD_KP                 35.0f /* 目标角速度 / 塑形后循迹误差。
                                            塑形公式 err*|err|/3 后，各误差点的输出为：
                                              err=1 -> 12 dps   （直道微调，不画龙）
                                              err=2 -> 47 dps
                                              err=3 -> 105 dps  （0.3 m 弯所需）
                                              err=4 -> 187 dps
                                              err=6 -> 300 dps  （顶到上限，急弯全力）
                                            线性映射（除以 3 拉平后）在 err=1 时给 35 dps，
                                            直道太抖；err=6 时只给 210 dps，急弯又不够。
                                            这才是"直道画龙 + 弯道冲出去"同时出现的根源。 */
#define LINE_PD_KD                 0.5f  /* 目标角速度 / 循迹误差变化率（提前量）。
                                           单位是 (dps)/(权重/秒)：偏差每 30 ms 变化 1 个权重
                                           就是 33 单位/秒，所以 0.5 约给 17 dps —— 温和的提前量。
                                           串联双环滞后 150 ms 以上，D 项负责在误差还没长大时
                                           就先打一点方向；但给太多会和量化噪声一起变成抖动源。 */
/* 微分窗口（采样次数）：用 30 ms 差商代替 10 ms 单步差分，压掉整数跳变尖峰。 */
#define LINE_DERIV_SPAN            3U
/* 差商限幅（权重/秒）：防止 Z 字弯换向瞬间差商把 D 项打爆。
   100 对应"30 ms 内变化 3 个权重"，正常过弯达不到，只有换向才有。 */
#define LINE_DERIV_LIMIT           100.0f
/* ---------------- Z 字弯 / 连续 S 弯 ----------------
   光电管视野有限，高速下车头刚摆过来，线已经跑出传感器。
   偏差一大就主动降速，让车有时间把姿态换过去，线回到中间后再提速。 */
/* 偏差超过该值开始降速，到第二个值降到最低比例。 */
#define CORNER_SLOW_ERR_START      2.0f
#define CORNER_SLOW_ERR_FULL       4.5f
/* 降速后的车速比例：车基本速已经是 0.30 的下限，再乘就掉进死区，
   所以这里只留很小的降速余地。 */
#define CORNER_SLOW_MIN_SCALE      0.90f
/* 目标速度的绝对下限（m/s）。实测死区 CCR 310 对应 0.29~0.31 m/s，
   低于这个值速度环只能在 CCR 0 与 310 之间开关，车会一顿一顿。 */
#define CONTROL_SPEED_MIN_MPS      0.28f
/* 减速比加速更急：加速 1 m/s²，减速 4 m/s²，否则进弯前来不及降下来。 */
#define CONTROL_DECEL_LIMIT_MPS2   4.0f
/* 循迹误差方向：1 = 误差为正时向左转，-1 = 向右转。
   必须与角速度环的方向一致：
   差速为正 → 左轮慢右轮快 → 车向左（逆时针）转；
   角速度环能稳定跟踪正目标就意味着"正目标 = 左转"，
   而线偏右（err>0）时必须右转，所以这里取 -1。
   符号错的表现：直道几秒就甩出线，弯道直接冲出赛道（越偏越往外打）。 */
#define LINE_ERROR_SIGN            (-1.0f)
/* 循迹误差塑形除数：输出 = err × |err| / 该值。越小越激进，3 是实测折中值。
   设为 0 或负数即退回线性映射（便于对比）。 */
#define LINE_ERROR_SHAPE           3.0f
#define YAW_PID_KP                 5.0f  /* 差速 PWM / 角速度误差。
                                            P 只补前馈的残差。实测对象增益 K≈615 dps per m/s，
                                            闭环增益 b=K×ykp/5500：ykp=10 时 b≈1.12（临界，实测
                                            车身角速度在 94~118 dps 之间来回摆），
                                            ykp=5 时 b≈0.56，配合前馈 a=K×yff≈1.0 更稳。
                                            这个值设成 0 会让角速度环只剩开环前馈，
                                            "能前进但不打舵"多半就是它。 */
#define YAW_PID_KI                 0.0f  /* 差速 PWM / 角速度误差积分 */
#define YAW_PID_KD                 0.0f  /* 差速 PWM / 角速度误差变化率（未用） */
/* 陀螺仪阻尼（差速 PWM 每 (dps/s)）：对实测角速度求微分后反向叠加。
   Z 字弯里车身刚转过一个弯、惯性还在往原方向甩时，这一项会立刻给出反向差速
   把车头拽回来，作用相当于给转向轴加一个阻尼器。
   实测角速度 100 ms 内变化 300 dps 时（3000 dps/s），0.05 会给 150 PWM 的阻尼，
   相当于 ykp 项在 30 dps 误差下的输出，量级合适。 */
#define YAW_RATE_DAMP              0.05f
/* 角速度微分的限幅（dps/s），防止噪声或跳变把阻尼项打爆。 */
#define YAW_RATE_DERIV_LIMIT       6000.0f
#define SPEED_PI_KP                100.0f  /* 基础 PWM / 速度误差。
                                              实测 CCR 317~330 对应 0.35~0.42 m/s，
                                              即约 0.0054 m/s 每 CCR、0.0097 m/s 每 PWM。
                                              Kp=200 时速度环增益约 1.94，配合 30 ms 滑窗
                                              会在低目标速度下打成 0↔330 的开关，
                                              日志里搜索阶段两轮交替滑行就是这么来的。
                                              降到 100 后增益约 0.97，能稳住。 */
#define SPEED_PI_KI                60.0f   /* 基础 PWM / 速度误差积分。
                                              为 0 时靠 P 撑不住稳态必抖；过大则撞墙时积分瞬间触限，
                                              脱困瞬间猛窜（实测 0.8 s 从 120 涨到 520，故取 60）。 */
/* 速度前馈：base 先按目标速度给一个开环油门，PI 只负责修正残差。
   实测标定：0.6 m/s 时实际只需要 base ≈ 77，故 77/0.6 ≈ 130。
   填过大会让 PI 用负积分去压，积分一旦触限就失去调节能力。 */
#define SPEED_FF_PWM_PER_MPS       130.0f  /* 基础 PWM / (m/s)，按实测标定 */
/* 差速换算（固定值，与 SPEED_FF 解耦）：turn 的 PWM 量折算成左右轮速度偏置。
   5500 意味着满 turn(700) 只对应 0.127 m/s 差速：
   实测 550 时 10 dps 误差就会给出 0.73 m/s 差速（≈320 dps 转向），开环增益约 30 倍，
   必然振荡。改成 5500 后同样误差只给 0.073 m/s，增益降到约 3 倍，可以调。 */
#define YAW_DIFF_PWM_PER_MPS       5500.0f
/* 角速度前馈：按目标角速度直接给差速（m/s 每 dps）。
   实测标定（2026-11-12）：由 odom 反推实际差速 0.169 m/s 时陀螺仪测到 104 dps，
   即对象增益 K≈615 dps per m/s（等价轮距 2×57.3/615≈0.186 m）。
   取 yff = 1/K ≈ 0.0016 是"稳态刚好跟上"的理论值；
   但速度环追不满指令差速（右轮目标接近 0.85 m/s 时实测只有 0.74），
   所以取 0.0025 略高于理论值，用一点超前来抵消执行机构的滞后。 */
#define YAW_DIFF_FF_MPS_PER_DPS    0.0025f
/* ---------------- 直角弯（陀螺仪角度闭环） ----------------
   直角弯的死穴：车头刚转过 90°，新的赛道还没进入光电管的视野，
   此时 P 项没有任何依据，车只能靠惯性冲出去。
   解决办法是不靠光电管，直接用陀螺仪积分出来的航向角闭环转过固定角度。 */
/* 目标转角（度）。85~90 之间；设 0 可整体关闭该功能。 */
#define CORNER_TURN_DEG            88.0f
/* 触发条件：偏差绝对值超过该值（千分之一权重）且同时压线路数不超过 3 路，
   即"最外侧的两三路压线、内侧全脱"——这是直角弯的典型特征。
   4500 对应重心落在最外侧两路之间。 */
#define CORNER_DETECT_ERR          4500
#define CORNER_DETECT_MAX_ACTIVE   3U
/* 连续满足多少个控制周期才认账，避免单帧误触发。 */
#define CORNER_DETECT_TICKS        2U
/* 角度到位判据（度）与超时保护。 */
#define CORNER_EXIT_DEG            6.0f
#define CORNER_EXIT_LINE_ERR       2000
#define CORNER_TIMEOUT_MS          1500U
#define CORNER_TIMEOUT_TICKS       (CORNER_TIMEOUT_MS / CONTROL_UPDATE_MS)
/* 角度环 P 增益（dps 每度）。90° 偏差时给 720 dps，会被限幅到 300，
   接近目标时自然减速；配合内层角速度环属于串级，不会有明显超调。 */
#define CORNER_ANGLE_KP            8.0f
/* 退出后冷却时间，防止刚上线又被同一个弯重复触发。
   Z 字弯的两个折角间隔很短，冷却太长会漏掉第二个折角，故取 150 ms。 */
#define CORNER_COOLDOWN_MS         150U
#define CORNER_COOLDOWN_TICKS      (CORNER_COOLDOWN_MS / CONTROL_UPDATE_MS)
/* 反转自检窗口：目标为负、实测仍向前超过 400 ms 即判定方向脚没生效。 */
#define REV_CHECK_MS               400U
#define REV_CHECK_TICKS            (REV_CHECK_MS / CONTROL_UPDATE_MS)

/* 单次差速偏置上限，防止饱和打转。
   允许内轮反转后差速可以做到 0.70：spd=0.45、差速 0.61 时
   内轮 -0.16、外轮 0.88，角速度约 320 dps、半径约 0.06 m。 */
#define YAW_DIFF_LIMIT_MPS         0.70f
/* 内轮允许反转的最大速度（m/s）。差速 = (外轮 - 内轮)/2，
   内轮能到负值之后同样车速下差速几乎翻倍，急弯才真正转得进去。
   只在内轮目标被压到负数时才启用，直道和缓弯完全不受影响。
   取 0。25 是折中：再大差速收益变小，而反向电流冲击和轮胎拖滑明显增加。 */
#define YAW_INNER_REVERSE_MPS      0.25f
/* 差速指令的最大变化率（m/s²）。0 表示不限制。
   防止前馈阶跃直接砸到速度环上造成过冲。 */
#define YAW_DIFF_SLEW_MPS2         12.0f
/* 左右轮目标速度的上限：差速偏置叠加后不能超过这个值，防止请求超出物理能力。 */
#define CONTROL_SPEED_TARGET_LIMIT_MPS  1.5f
#define YAW_INTEGRAL_LIMIT         500.0f
#define SPEED_INTEGRAL_LIMIT       2.0f

/* ---------------- 在线整定（逐飞助手 / 无线串口） ----------------
   上面这些增益与死区在运行时可被串口改写，断电后恢复为宏里的值。
   改到满意的数值后，把它写回对应宏并重新编译即可固化。 */
#define TUNE_ENABLE                1
#define TUNE_RX_BUFFER_SIZE        256U
#define TUNE_LINE_MAX              40U
/* 部分串口工具默认不发送换行符，此时按“空闲超时”收尾，
   避免整行一直等不到 \r\n 而毫无反应。 */
#define TUNE_LINE_IDLE_MS          150U

/* 一个滑动窗口内的计数之和对应的车轮线速度，单位 m/s，编译期常量 */
#define ENC_COUNTS_TO_MPS \
  ((1000.0f / (float)SPEED_WINDOW_MS) / ENCODER_COUNTS_PER_METER)

/* 测速结果打印开关。以整数 mm/s 输出，不引入额外的浮点 printf 代码。 */
#define SPEED_DEBUG            1
#define SPEED_PRINT_MS         100U
#define GYRO_PRINT_MS          100U
/* 累计计数打印周期，标定每米计数时用来读取总计数。 */
#define ODOM_PRINT_MS          200U
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

/* 累计计数，用于标定“每米计数”。按一次发车键清零，
   直接对每次 10 ms 增量累加，不受调试打印降采样（100 ms）影响。 */
static volatile int32_t odom_left;
static volatile int32_t odom_right;

/* 循迹状态：通道 0 在最左侧，通道 11 在最右侧。 */
static volatile uint16_t line_sensor_bits;
/* 偏差单位为千分之一传感器权重，左偏为负，右偏为正。 */
static volatile int16_t line_error_milli;
/* 至少有一路传感器检测到白线时为 1；全无信号时为 0。 */
static volatile uint8_t line_sensor_valid;
/* 光电管极性自动判定结果：1 表示实际输出与 LINE_SENSOR_ACTIVE_LEVEL 相反。
   上电后、发车前自动判定一次，避免"极性反了导致车往线的反方向跑"。 */
static uint8_t line_polarity_inverted;
static uint8_t line_polarity_locked;
static uint16_t line_polarity_votes;
static uint16_t line_polarity_ok_votes;
static uint16_t line_polarity_total;
/* 循迹方向修正：lsg=-1 可在线翻转，用于排查"越偏越往外打"的情况。 */
static volatile float tune_line_sign = LINE_ERROR_SIGN;

/* 发车键每按一次在"待发车"和"运行中"之间切换：1 表示运行中。 */
static volatile uint8_t run_button_started;
/* IMU 初始化状态：失败时禁止后续控制计算。 */
static volatile uint8_t imu_ready;

/* 故障码，非 0 时控制环停机；由主循环打印一次。 */
static volatile uint8_t control_fault;
/* 最近一次 IMU 采样时刻，用于检测角速度数据是否过期。 */
static volatile uint32_t gyro_sample_tick;
/* 速度环斜坡后的目标速度，单位 m/s。 */
static float control_speed_target_mps;

/* 陀螺仪 Z 轴零偏校准：发车键按下前车静止，用这段时间累积样本求均值，
   按键锁存那一刻固化为 offset。offset 保存的是未做安装方向修正的原始值。 */
static uint32_t gyro_bias_tick;
static uint32_t gyro_bias_count;
static float    gyro_bias_sum;
static float    gyro_z_offset;
static uint8_t  gyro_bias_ready;
static uint8_t  gyro_bias_latched;

/* 控制器中间量与限幅后的左右轮 PWM，便于调试器查看。 */
static volatile float control_target_yaw_rate_dps;
static volatile float control_measured_yaw_rate_dps;
static volatile float control_yaw_diff_mps;
static volatile float control_turn_pwm;
static volatile float control_base_pwm;
static volatile uint16_t control_left_pwm;
static volatile uint16_t control_right_pwm;

/* 可在线整定的参数：上电取自宏，运行中由串口命令 / 逐飞助手改写。
   控制环在 TIM2 中断里读取，所以用 volatile；32 位对齐写天然原子，无需加锁。 */
static volatile float tune_line_kp      = LINE_PD_KP;
static volatile float tune_line_kd      = LINE_PD_KD;
static volatile float tune_yaw_kp       = YAW_PID_KP;
static volatile float tune_yaw_ki       = YAW_PID_KI;
static volatile float tune_yaw_kd       = YAW_PID_KD;
static volatile float tune_speed_kp     = SPEED_PI_KP;
static volatile float tune_speed_ki     = SPEED_PI_KI;
static volatile float tune_speed_ff     = SPEED_FF_PWM_PER_MPS;
/* 角速度方向修正，可在运行中翻转：ysg=-1 / ysg=1。
   符号错会让角速度环变成正反馈（原地打转），必须先确认方向再调增益。 */
static volatile float tune_yaw_sign     = YAW_RATE_SIGN;
/* 角速度一阶低通系数（0~1，越大越平滑、相位滞后越大）。
   电机振动会让 gyro_z 出现 ±2 dps 的高频噪声，滤波后角速度环可以用更大增益。 */
static const float yaw_rate_filter_alpha = 0.5f;
/* 零偏手动微调（dps）：自动校准后若仍有残余，用 ofs=±x 在线拧到静止读数为 0。 */
static volatile float tune_gyro_trim = GYRO_TRIM_DPS;
/* 角速度前馈系数（m/s 每 dps）：直接决定"能不能转过急弯"，可在线调。 */
static volatile float tune_yaw_ff = YAW_DIFF_FF_MPS_PER_DPS;
/* 角速度环阶跃测试（yt 命令，dps）：非 0 时忽略循迹、直接把该值当作目标角速度。
   用于阶段 7 单独标定角速度环：车会按 半径 = 速度/(目标角速度) 画圆，
   串口 ctrl 帧里能直接对比 target 与实测 yaw，判断增益是否合适。
   |yt| < 1 表示退出测试、回到正常循迹。 */
static volatile float tune_yaw_test_dps = 0.0f;

/* 角速度环阶跃测试是否生效（yt 非 0）。测试期间会自动静音循迹与编码器打印，
   只留 ctrl 帧，避免几十行/秒的滚动把角速度读数冲掉。 */
static uint8_t YawTest_Active(void)
{
  return ((tune_yaw_test_dps > -1.0f) && (tune_yaw_test_dps < 1.0f)) ? 0U : 1U;
}
/* 内轮允许反转的最大速度（m/s），可在运行中用 rev 命令改写。
   设为 0 即退回"内轮最低停转"的旧行为，便于一键对比。 */
static volatile float tune_inner_reverse_mps = YAW_INNER_REVERSE_MPS;
/* 差速指令的斜坡状态（跨周期保留，见 VehicleControl_Update）。 */
static float yaw_diff_slewed;
/* 陀螺仪积分出来的航向角（度，正 = 左转）。直角弯状态机用它做绝对角度闭环。 */
static volatile float control_yaw_angle;
static volatile float tune_corner_deg = CORNER_TURN_DEG;
static uint8_t  corner_state;            /* 0 = 循迹，1 = 直角弯转角中 */
static float    corner_target_angle;
static uint16_t corner_ticks;
static uint16_t corner_detect_ticks;
static uint16_t corner_cooldown_ticks;
/* 直角弯事件由中断里登记、主循环里打印：printf 是阻塞 UART，
   在 10 ms 控制中断里直接打印会拖垮控制周期。 */
static volatile uint8_t corner_event;        /* 0 无 / 1 进入 / 2 退出 */
static volatile float   corner_event_angle;
/* 角速度环的陀螺仪阻尼：对实测角速度求微分，系数与状态在这里。 */
static volatile float tune_yaw_rate_damp = YAW_RATE_DAMP;
static float yaw_prev_rate;
static float yaw_rate_derivative;
/* Z 字弯检测用的上一拍偏差（判断是否发生了左右翻转）。 */
static int16_t prev_line_error_milli;
/* 是否处于丢线后的直行搜索阶段（主循环用来缩放车速）。 */
static volatile uint8_t line_searching;
/* 丢线后是否继续直行搜索：0 = 直接停机（默认），1 = 继续开 1.2 s 找线。
   可在运行中用 srch 命令切换。 */
static volatile float tune_search_enable = 0.0f;
/* 内轮反转自检：目标为负但实测仍向前跑满 REV_CHECK_MS 就打一次标记。 */
static volatile uint8_t rev_fail_flag;
static uint16_t rev_check_ticks;
static volatile float tune_target_speed = CONTROL_TARGET_SPEED_MPS;
static volatile float tune_deadzone_l   = MOTOR_DEADZONE_L;
static volatile float tune_deadzone_r   = MOTOR_DEADZONE_R;
/* 堵转保护开关：台架/调参时可用 stl=0 关闭，避免车轮不转就自动停机。 */
static volatile uint8_t tune_stall_enable = 1U;

/* 周期性调试打印的运行时开关：串口发 q 静音、发 v 恢复。
   静音时仍保留 run / fault / set / show 等事件性输出，
   这样调参回复不会被 line / enc / ctrl / gyro 每 100 ms 的刷屏冲掉。 */
static volatile uint8_t debug_print_on = 1U;

#if TUNE_ENABLE
/* USART3 接收环形缓冲：中断里只做入队，解析放在主循环，避免阻塞。 */
static uint8_t tune_rx_buffer[TUNE_RX_BUFFER_SIZE];
static volatile uint16_t tune_rx_head;
static volatile uint16_t tune_rx_tail;
static uint8_t tune_rx_byte;
#endif

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
static void GyroBias_Update(float raw_gyro_z);
static void VehicleControl_Update(void);
#if TUNE_ENABLE
static void Tune_Update(void);
static void Tune_PrintAll(void);
#endif
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
  uint8_t active_level = LINE_SENSOR_ACTIVE_LEVEL;

  MUX_get_value(&sensor_bits);
  line_sensor_bits = sensor_bits;

  /* 极性自动判定（上电后、发车前判定一次，之后锁定）：
     悬空或压在线上时，12 路里正常最多只有 1~3 路处于"压线"电平；
     若超过一半通道都落在 probe_level 上，说明 LINE_SENSOR_ACTIVE_LEVEL 与实际电平相反。
     判定结果会打印 line,pol,0/1，可用 pol 命令（通道 17）强制改写或重新判定。 */
  if ((run_button_started == 0U) && (line_polarity_locked == 0U))
  {
    uint16_t raw_active = 0U;
    uint8_t probe_level = LINE_SENSOR_ACTIVE_LEVEL;

    if (line_polarity_inverted != 0U)
      probe_level = (probe_level == 0U) ? 1U : 0U;

    for (channel = 0U; channel < MULTIPLEXER_CHANNEL_NUM; ++channel)
    {
      if (MUX_GET_CHANNEL(sensor_bits, channel) == probe_level)
        ++raw_active;
    }

    if (raw_active > (MULTIPLEXER_CHANNEL_NUM / 2U))
    {
      line_polarity_ok_votes = 0U;
      if (line_polarity_votes < 30U)
        ++line_polarity_votes;
    }
    else
    {
      line_polarity_votes = 0U;
      if (line_polarity_ok_votes < 50U)
        ++line_polarity_ok_votes;
    }

    if (line_polarity_votes >= 30U)
    {
      line_polarity_inverted = (line_polarity_inverted != 0U) ? 0U : 1U;
      line_polarity_locked = 1U;
      printf("line,pol,%u\r\n", (unsigned int)line_polarity_inverted);
    }
    else if (line_polarity_ok_votes >= 50U)
    {
      line_polarity_locked = 1U;
      printf("line,pol,%u\r\n", (unsigned int)line_polarity_inverted);
    }
    else
    {
      /* 3 秒内读不到稳定状态（例如传感器对空或背景反光不均）就按当前极性锁定 */
      if (line_polarity_total < 400U)
        ++line_polarity_total;
      if (line_polarity_total >= 300U)
      {
        line_polarity_locked = 1U;
        printf("line,pol,%u\r\n", (unsigned int)line_polarity_inverted);
      }
      else
      {
        /* 判定期间不输出循迹量，避免误用未定极性的数据 */
        line_error_milli = 0;
        line_sensor_valid = 0U;
        return;
      }
    }
  }

  if (line_polarity_inverted != 0U)
    active_level = (active_level == 0U) ? 1U : 0U;

  for (channel = 0U; channel < MULTIPLEXER_CHANNEL_NUM; ++channel)
  {
    if (MUX_GET_CHANNEL(sensor_bits, channel) == active_level)
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

  /* 十字路口 / 宽线：12 路里 5 路以上同时压线，说明横向还有一条线穿过去，
     此时加权平均会算出一个"假偏差"（例如 011111100000 得 -2333），
     车会朝空地打舵。正确做法是保持上一次有效偏差直着穿过去。
     正常赛道线宽只会覆盖 1~3 路。 */
  if (active_count >= LINE_JUNCTION_ACTIVE_MIN)
  {
    line_sensor_valid = 1U;
    return;
  }

  /* 对有效传感器的权重取平均，避免多路同时压线时偏差被放大。 */
  line_error_milli = (int16_t)((weighted_position * LINE_ERROR_SCALE) / active_count);
  line_sensor_valid = 1U;
}

/* 陀螺仪零偏校准：发车键按下前整车静止，用这段时间累积 Z 轴角速度求均值，
   按键锁存瞬间固化为 offset，之后控制环使用（原始值 - offset）消除恒定漂移。
   必须在主循环读取 IMU 之后调用，raw_gyro_z 是未扣除零偏的物理值。 */
static void GyroBias_Update(float raw_gyro_z)
{
  if ((imu_ready == 0U) || (gyro_bias_latched != 0U))
    return;

  if (gyro_bias_ready == 0U)
  {
    /* 0 用作"尚未开始计时"的标记，避免上电瞬间 HAL_GetTick() 返回 0 时反复重置 */
    if (gyro_bias_tick == 0U)
    {
      gyro_bias_tick = HAL_GetTick();
      if (gyro_bias_tick == 0U)
        gyro_bias_tick = 1U;
      return;
    }
    if ((HAL_GetTick() - gyro_bias_tick) < GYRO_BIAS_SETTLE_MS)
      return;
    gyro_bias_ready = 1U;
  }

  if (run_button_started == 0U)
  {
    if (gyro_bias_count < GYRO_BIAS_MAX_SAMPLES)
    {
      gyro_bias_sum += raw_gyro_z;
      ++gyro_bias_count;
    }
    return;
  }

  /* 按键按下：样本足够才采信，否则保留 0，并在串口报告实际样本数，
     便于发现"上电后立刻发车、零偏没校准"的情况。 */
  if (gyro_bias_count >= GYRO_BIAS_MIN_SAMPLES)
    gyro_z_offset = gyro_bias_sum / (float)gyro_bias_count;

  gyro_bias_latched = 1U;
  printf("gyro,ofs,%ld,%lu\r\n",
         (long)(gyro_z_offset * 1000.0f), (unsigned long)gyro_bias_count);
}

/* 停车后重新打开零偏校准窗口：等 GYRO_BIAS_SETTLE_MS 后再重新累积，
   这样下次发车会用最新的静止读数求零偏，不必断电重启。 */
static void GyroBias_Restart(void)
{
  gyro_bias_sum = 0.0f;
  gyro_bias_count = 0U;
  gyro_bias_ready = 0U;
  gyro_bias_latched = 0U;
  gyro_bias_tick = HAL_GetTick();
  if (gyro_bias_tick == 0U)
    gyro_bias_tick = 1U;
}

#if TUNE_ENABLE
/* ---------------- 在线整定：串口协议 ----------------
   两条路都走 USART3（有线调试线与无线串口模块共用这组收发脚）：

   A. 逐飞助手「参数设置」面板
      下发帧：0x55 0x20 通道 校验 4 字节 float（小端），通道 1~10。
      校验字节只接收不比对，因此不依赖上位机的具体校验算法。

   B. 纯文本命令（逐飞助手「串口助手」或任意串口工具，末尾回车换行）
        lkp=10    循迹 Kp        lkd=0    循迹 Kd
        ykp=5     角速度 Kp      yki=0    角速度 Ki      ykd=0  角速度 Kd
        skp=500   速度 Kp        ski=60   速度 Ki        sff=130  速度前馈
        spd=0.6   目标速度 m/s
        dzl=310   dzr=300       左右轮死区 CCR
        ysg=-1    角速度方向      ofs=0.6   零偏微调（dps）
        stl=0     关闭堵转保护
        show      打印当前全部参数      ?  同上
        q / v     关闭 / 打开周期打印
        save      把当前参数写入 Flash（断电不丢）
        load      重新从 Flash 读取
        default   恢复宏默认值并清除存档

   参数默认来自下面的宏；用 save 保存后，每次上电会自动加载存档。 */

static const char *const tune_names[] =
{
  "lkp", "lkd", "ykp", "yki", "ykd", "skp", "ski", "spd", "dzl", "dzr", "sff", "stl", "ysg", "ofs", "yff", "lsg", "pol", "yt", "rev", "c90", "ydm", "srch"
};

/* 通道号与宏一一对应：1=lKp 2=lKd 3=yKp 4=yKi 5=yKd 6=sKp 7=sKi 8=目标速度 9/10=左右死区 */
static void Tune_ApplyParameter(uint8_t index, float value)
{
  /* 挡住串口噪声造成的 NaN/INF 与离谱数值 */
  if (!((value == value) && (value > -1000000.0f) && (value < 1000000.0f)))
    return;

  switch (index)
  {
    case 1U:  tune_line_kp      = value; break;
    case 2U:  tune_line_kd      = value; break;
    case 3U:  tune_yaw_kp       = value; break;
    case 4U:  tune_yaw_ki       = value; break;
    case 5U:  tune_yaw_kd       = value; break;
    case 6U:  tune_speed_kp     = value; break;
    case 7U:  tune_speed_ki     = value; break;
    case 8U:  tune_target_speed = value; break;
    case 9U:  tune_deadzone_l   = value; break;
    case 10U: tune_deadzone_r   = value; break;
    case 11U: tune_speed_ff     = value; break;
    case 12U: tune_stall_enable = (value != 0.0f) ? 1U : 0U; break;
    case 13U: tune_yaw_sign   = (value < 0.0f) ? -1.0f : 1.0f; break;
    case 14U: tune_gyro_trim  = value; break;
    case 15U: tune_yaw_ff     = value; break;
    case 16U: tune_line_sign  = (value < 0.0f) ? -1.0f : 1.0f; break;
    case 17U: /* 光电管极性：pol=0/1 强制指定，pol=-1 清除锁定、重新自动判定 */
      line_polarity_votes = 0U;
      line_polarity_ok_votes = 0U;
      line_polarity_total = 0U;
      if (value < 0.0f)
      {
        line_polarity_locked = 0U;
      }
      else
      {
        line_polarity_inverted = (value < 0.5f) ? 0U : 1U;
        line_polarity_locked = 1U;
      }
      break;
    case 18U: /* 角速度环阶跃测试：yt=60 表示强制目标 +60 dps，yt=0 回到循迹 */
      if ((value > -1.0f) && (value < 1.0f))
      {
        tune_yaw_test_dps = 0.0f;
      }
      else if (value > CONTROL_MAX_YAW_RATE_DPS)
      {
        tune_yaw_test_dps = CONTROL_MAX_YAW_RATE_DPS;
      }
      else if (value < -CONTROL_MAX_YAW_RATE_DPS)
      {
        tune_yaw_test_dps = -CONTROL_MAX_YAW_RATE_DPS;
      }
      else
      {
        tune_yaw_test_dps = value;
      }
      break;
    case 19U: /* 内轮反转上限：rev=0 退回内轮最低停转，rev=0.25 允许反转 */
      if (value < 0.0f)
        tune_inner_reverse_mps = 0.0f;
      else if (value > 0.35f)
        tune_inner_reverse_mps = 0.35f;
      else
        tune_inner_reverse_mps = value;
      break;
    case 20U: /* 直角弯目标转角（度）：c90=88 正常，c90=0 关闭该功能 */
      if (value < 0.0f)
        tune_corner_deg = 0.0f;
      else if (value > 120.0f)
        tune_corner_deg = 120.0f;
      else
        tune_corner_deg = value;
      corner_detect_ticks = 0U;
      break;
    case 21U: /* 陀螺仪阻尼系数：ydm=0 关闭，ydm=0.05 默认 */
      if (value < 0.0f)
        tune_yaw_rate_damp = 0.0f;
      else if (value > 0.5f)
        tune_yaw_rate_damp = 0.5f;
      else
        tune_yaw_rate_damp = value;
      break;
    case 22U: /* 丢线后是否直行搜索：srch=0 停机（默认），srch=1 继续找线 */
      tune_search_enable = (value > 0.5f) ? 1.0f : 0.0f;
      if (tune_search_enable < 0.5f)
        line_searching = 0U;
      break;
    default: return;
  }
  printf("set,%u,%ld\r\n", (unsigned int)index, (long)(value * 1000.0f));
}

/* 数值解析：只支持简单的十进制，避免引入 strtof 的浮点库开销。 */
static float Tune_ParseValue(const char *s, uint8_t len, uint8_t *ok)
{
  float value = 0.0f;
  float scale = 0.1f;
  uint8_t i = 0U;
  uint8_t negative = 0U;
  uint8_t digits = 0U;
  uint8_t after_dot = 0U;

  *ok = 0U;
  while ((i < len) && ((s[i] == ' ') || (s[i] == '\t')))
    ++i;

  if ((i < len) && ((s[i] == '-') || (s[i] == '+')))
  {
    negative = (s[i] == '-') ? 1U : 0U;
    ++i;
  }

  for (; i < len; ++i)
  {
    if ((s[i] >= '0') && (s[i] <= '9'))
    {
      if (after_dot == 0U)
      {
        value = (value * 10.0f) + (float)(s[i] - '0');
      }
      else
      {
        value += (float)(s[i] - '0') * scale;
        scale *= 0.1f;
      }
      ++digits;
    }
    else if ((s[i] == '.') && (after_dot == 0U))
    {
      after_dot = 1U;
    }
    else if ((s[i] == ' ') || (s[i] == '\t'))
    {
      continue;
    }
    else
    {
      return 0.0f;
    }
  }

  *ok = (digits > 0U) ? 1U : 0U;
  return (negative != 0U) ? -value : value;
}

/* ---------------- 参数持久化：Flash 最后一页 ----------------
   STM32F103C8 为 64 KB Flash、1 KB/页，程序区当前约 44.9 kB，
   最后一页 0x0800FC00 起还剩约 18 kB 余量，用它存放整定参数，不触碰程序区。
   上电时若 magic / 版本 / 校验都匹配则自动加载，否则用宏默认值。 */
#define TUNE_FLASH_ADDR     0x0800FC00U
#define TUNE_MAGIC_VALUE    0x53434647U   /* 'SCFG' */
#define TUNE_VERSION_VALUE  2U

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float line_kp;
  float line_kd;
  float yaw_kp;
  float yaw_ki;
  float yaw_kd;
  float speed_kp;
  float speed_ki;
  float speed_ff;
  float target_speed;
  float deadzone_l;
  float deadzone_r;
  float yaw_sign;
  float gyro_trim;
  float yaw_ff;
  uint32_t stall_enable;
  uint32_t checksum;
} TuneConfig;

/* 按字写入 Flash 要求结构体大小是 4 的倍数，这里做编译期检查。 */
typedef char TuneConfigSizeIsWordAligned[(sizeof(TuneConfig) % 4U == 0U) ? 1 : -1];

/* 校验和只覆盖 checksum 之前的字段；乘法混合比简单求和更能发现位翻转。 */
static uint32_t Tune_Checksum(const TuneConfig *cfg)
{
  const uint32_t *word = (const uint32_t *)cfg;
  const uint32_t words = (uint32_t)((sizeof(TuneConfig) / 4U) - 1U);
  uint32_t sum = 0x5A5A5A5AU;
  uint32_t i;

  for (i = 0U; i < words; ++i)
    sum = (sum * 31U) + word[i];
  return sum;
}

static void Tune_SetDefaults(void)
{
  tune_line_kp      = LINE_PD_KP;
  tune_line_kd      = LINE_PD_KD;
  tune_yaw_kp       = YAW_PID_KP;
  tune_yaw_ki       = YAW_PID_KI;
  tune_yaw_kd       = YAW_PID_KD;
  tune_speed_kp     = SPEED_PI_KP;
  tune_speed_ki     = SPEED_PI_KI;
  tune_speed_ff     = SPEED_FF_PWM_PER_MPS;
  tune_target_speed = CONTROL_TARGET_SPEED_MPS;
  tune_deadzone_l   = MOTOR_DEADZONE_L;
  tune_deadzone_r   = MOTOR_DEADZONE_R;
  tune_yaw_sign     = YAW_RATE_SIGN;
  tune_gyro_trim    = GYRO_TRIM_DPS;
  tune_yaw_ff       = YAW_DIFF_FF_MPS_PER_DPS;
  tune_line_sign    = LINE_ERROR_SIGN;
  /* 阶跃测试不属于标定参数，恢复默认时一并退出，避免发车后车莫名画圈。 */
  tune_yaw_test_dps = 0.0f;
  tune_inner_reverse_mps = YAW_INNER_REVERSE_MPS;
  tune_corner_deg = CORNER_TURN_DEG;
  tune_yaw_rate_damp = YAW_RATE_DAMP;
  tune_search_enable = 0.0f;
  tune_stall_enable = 1U;
}

/* 返回 1 表示加载成功。 */
static uint8_t Tune_LoadFromFlash(void)
{
  const TuneConfig *cfg = (const TuneConfig *)TUNE_FLASH_ADDR;

  if ((cfg->magic != TUNE_MAGIC_VALUE) ||
      (cfg->version != TUNE_VERSION_VALUE) ||
      (cfg->checksum != Tune_Checksum(cfg)))
  {
    return 0U;
  }

  tune_line_kp      = cfg->line_kp;
  tune_line_kd      = cfg->line_kd;
  tune_yaw_kp       = cfg->yaw_kp;
  tune_yaw_ki       = cfg->yaw_ki;
  tune_yaw_kd       = cfg->yaw_kd;
  tune_speed_kp     = cfg->speed_kp;
  tune_speed_ki     = cfg->speed_ki;
  tune_speed_ff     = cfg->speed_ff;
  tune_target_speed = cfg->target_speed;
  tune_deadzone_l   = cfg->deadzone_l;
  tune_deadzone_r   = cfg->deadzone_r;
  tune_yaw_sign     = (cfg->yaw_sign < 0.0f) ? -1.0f : 1.0f;
  tune_gyro_trim    = cfg->gyro_trim;
  tune_yaw_ff       = cfg->yaw_ff;
  tune_stall_enable = (cfg->stall_enable != 0U) ? 1U : 0U;
  return 1U;
}

/* 先擦后写并回读校验。擦除期间 CPU 取指会停顿约几十毫秒，
   所以请在停车状态下保存。返回 1 表示成功。 */
static uint8_t Tune_SaveToFlash(void)
{
  TuneConfig cfg;
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;
  uint32_t addr = TUNE_FLASH_ADDR;
  const uint32_t *word;
  uint32_t i;

  cfg.magic         = TUNE_MAGIC_VALUE;
  cfg.version       = TUNE_VERSION_VALUE;
  cfg.reserved      = 0U;
  cfg.line_kp       = tune_line_kp;
  cfg.line_kd       = tune_line_kd;
  cfg.yaw_kp        = tune_yaw_kp;
  cfg.yaw_ki        = tune_yaw_ki;
  cfg.yaw_kd        = tune_yaw_kd;
  cfg.speed_kp      = tune_speed_kp;
  cfg.speed_ki      = tune_speed_ki;
  cfg.speed_ff      = tune_speed_ff;
  cfg.target_speed  = tune_target_speed;
  cfg.deadzone_l    = tune_deadzone_l;
  cfg.deadzone_r    = tune_deadzone_r;
  cfg.yaw_sign      = tune_yaw_sign;
  cfg.gyro_trim     = tune_gyro_trim;
  cfg.yaw_ff        = tune_yaw_ff;
  cfg.stall_enable  = (uint32_t)tune_stall_enable;
  cfg.checksum      = Tune_Checksum(&cfg);

  HAL_FLASH_Unlock();

  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.Banks       = FLASH_BANK_1;
  erase.PageAddress = TUNE_FLASH_ADDR;
  erase.NbPages     = 1U;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  word = (const uint32_t *)&cfg;
  for (i = 0U; i < (sizeof(TuneConfig) / 4U); ++i)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 0U;
    }
    addr += 4U;
  }

  HAL_FLASH_Lock();

  /* 回读验证：magic、校验和都必须匹配 */
  return (Tune_LoadFromFlash() != 0U) ? 1U : 0U;
}

/* 恢复宏默认值，并擦除 Flash 中的存档，避免下次上电又加载旧值。 */
static uint8_t Tune_ResetDefaults(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;

  Tune_SetDefaults();

  HAL_FLASH_Unlock();
  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.Banks       = FLASH_BANK_1;
  erase.PageAddress = TUNE_FLASH_ADDR;
  erase.NbPages     = 1U;
  (void)HAL_FLASHEx_Erase(&erase, &page_error);
  HAL_FLASH_Lock();
  return 1U;
}

static void Tune_PrintAll(void)
{
  /* 单行紧凑输出：便于在滚动日志里一眼看全，也方便整行复制。
     数值统一 ×1000，例如 spd 显示 700 表示 0.700。 */
  printf("cfg,lkp=%ld,lkd=%ld,ykp=%ld,yki=%ld,ykd=%ld,skp=%ld,ski=%ld,sff=%ld,spd=%ld,dzl=%ld,dzr=%ld,stl=%u,ysg=%ld,ofs=%ld,yff=%ld,lsg=%ld,pol=%u\r\n",
         (long)(tune_line_kp * 1000.0f),
         (long)(tune_line_kd * 1000.0f),
         (long)(tune_yaw_kp * 1000.0f),
         (long)(tune_yaw_ki * 1000.0f),
         (long)(tune_yaw_kd * 1000.0f),
         (long)(tune_speed_kp * 1000.0f),
         (long)(tune_speed_ki * 1000.0f),
         (long)(tune_speed_ff * 1000.0f),
         (long)(tune_target_speed * 1000.0f),
         (long)(tune_deadzone_l * 1000.0f),
         (long)(tune_deadzone_r * 1000.0f),
         (unsigned int)tune_stall_enable,
         (long)tune_yaw_sign,
         (long)(tune_gyro_trim * 1000.0f),
         (long)(tune_yaw_ff * 1000000.0f),
         (long)tune_line_sign,
         (unsigned int)line_polarity_inverted);
  printf("cfg2,yt=%ld,rev=%ld,c90=%ld,ydm=%ld,srch=%ld\r\n", (long)(tune_yaw_test_dps * 1000.0f),
         (long)(tune_inner_reverse_mps * 1000.0f),
         (long)(tune_corner_deg * 10.0f),
         (long)(tune_yaw_rate_damp * 1000.0f),
         (long)tune_search_enable);
}

/* 比较 line 的前 len 个字符是否恰好等于 name */
static uint8_t Tune_NameMatch(const char *line, uint8_t len, const char *name)
{
  uint8_t i;

  for (i = 0U; i < len; ++i)
  {
    if ((name[i] == '\0') || (line[i] != name[i]))
      return 0U;
  }
  return (name[len] == '\0') ? 1U : 0U;
}

static void Tune_ExecuteLine(const char *line, uint8_t len)
{
  uint8_t name_len = 0U;
  uint8_t offset;
  uint8_t ok = 0U;
  uint8_t k;
  float value;

  while ((len > 0U) && ((line[0] == ' ') || (line[0] == '\t')))
  {
    ++line;
    --len;
  }
  while ((len > 0U) && ((line[len - 1U] == ' ') || (line[len - 1U] == '\t')))
    --len;
  if (len == 0U)
    return;

  if ((Tune_NameMatch(line, len, "show") != 0U) ||
      (Tune_NameMatch(line, len, "?") != 0U))
  {
    Tune_PrintAll();
    return;
  }

  /* 静音 / 恢复周期性调试打印：调参时先 q，回显就只剩 set / cfg / err。 */
  if ((Tune_NameMatch(line, len, "q") != 0U) ||
      (Tune_NameMatch(line, len, "quiet") != 0U) ||
      (Tune_NameMatch(line, len, "off") != 0U))
  {
    debug_print_on = 0U;
    printf("dbg,off\r\n");
    return;
  }
  if ((Tune_NameMatch(line, len, "v") != 0U) ||
      (Tune_NameMatch(line, len, "verbose") != 0U) ||
      (Tune_NameMatch(line, len, "on") != 0U))
  {
    debug_print_on = 1U;
    printf("dbg,on\r\n");
    return;
  }

  /* 参数持久化：save 写 Flash，load 重新读取，default 恢复宏默认并清除存档。 */
  if (Tune_NameMatch(line, len, "save") != 0U)
  {
    printf("flash,save,%u\r\n", (unsigned int)Tune_SaveToFlash());
    return;
  }
  if (Tune_NameMatch(line, len, "load") != 0U)
  {
    printf("flash,load,%u\r\n", (unsigned int)Tune_LoadFromFlash());
    return;
  }
  if ((Tune_NameMatch(line, len, "default") != 0U) ||
      (Tune_NameMatch(line, len, "reset") != 0U))
  {
    printf("flash,default,%u\r\n", (unsigned int)Tune_ResetDefaults());
    return;
  }

  /* 名字部分到 '=' 或空格为止，例如 lkp=10 / lkp 10 */
  while ((name_len < len) && (line[name_len] != '=') && (line[name_len] != ' '))
    ++name_len;

  offset = name_len;
  while ((offset < len) && ((line[offset] == '=') || (line[offset] == ' ')))
    ++offset;

  value = Tune_ParseValue(&line[offset], (uint8_t)(len - offset), &ok);
  if (ok == 0U)
  {
    printf("err,value\r\n");
    return;
  }

  for (k = 0U; k < 22U; ++k)
  {
    if (Tune_NameMatch(line, name_len, tune_names[k]) != 0U)
    {
      Tune_ApplyParameter((uint8_t)(k + 1U), value);
      return;
    }
  }

  printf("err,name\r\n");
}

/* 主循环调用：把中断收进来的字节流分帧解析。空闲时开销只是一次队列判断。 */
static void Tune_Update(void)
{
  static char line[TUNE_LINE_MAX];
  static uint8_t line_len;
  static uint32_t line_tick;
  static uint8_t frame_state;      /* 0=文本 1=收到 0x55 2=等功能字 3=通道 4=校验 5..8=float */
  static uint8_t frame_index;
  static uint8_t frame_channel;
  static uint8_t frame_bytes[4];
  uint8_t byte;

  while (tune_rx_tail != tune_rx_head)
  {
    byte = tune_rx_buffer[tune_rx_tail];
    tune_rx_tail = (uint16_t)((tune_rx_tail + 1U) % TUNE_RX_BUFFER_SIZE);

    if (frame_state != 0U)
    {
      if (frame_state == 1U)
      {
        if (byte == 0x20U)
        {
          frame_state = 2U;
        }
        else if (byte != 0x55U)
        {
          frame_state = 0U;
        }
        continue;
      }
      if (frame_state == 2U)
      {
        frame_channel = byte;
        frame_state = 3U;
        continue;
      }
      if (frame_state == 3U)
      {
        frame_state = 4U;           /* 校验字节只接收不比对 */
        continue;
      }

      frame_bytes[frame_index] = byte;
      if (++frame_index >= 4U)
      {
        float value;
        uint8_t *raw = (uint8_t *)&value;

        raw[0] = frame_bytes[0];
        raw[1] = frame_bytes[1];
        raw[2] = frame_bytes[2];
        raw[3] = frame_bytes[3];
        Tune_ApplyParameter(frame_channel, value);

        frame_state = 0U;
        frame_index = 0U;
      }
      continue;
    }

    if (byte == 0x55U)
    {
      frame_state = 1U;
      continue;
    }
    if ((byte == '\r') || (byte == '\n'))
    {
      if (line_len > 0U)
      {
        Tune_ExecuteLine(line, line_len);
        line_len = 0U;
      }
      continue;
    }
    if ((byte >= 0x20U) && (byte < 0x7FU))
    {
      if (line_len < TUNE_LINE_MAX)
        line[line_len++] = (char)byte;
      line_tick = HAL_GetTick();
      continue;
    }
    line_len = 0U;                  /* 非法字节，丢弃当前行 */
  }

  /* 没有换行符时按空闲超时收尾：等效于“发送完整命令后稍等即执行”。 */
  if ((line_len > 0U) && ((HAL_GetTick() - line_tick) >= TUNE_LINE_IDLE_MS))
  {
    Tune_ExecuteLine(line, line_len);
    line_len = 0U;
  }
}
#endif /* TUNE_ENABLE */

/* 将控制器状态清零，防止停机或丢线期间积分继续累积。 */
static void VehicleControl_Reset(ControlLoopState *line_state,
                                 ControlLoopState *yaw_state,
                                 ControlLoopState *speed_left_state,
                                 ControlLoopState *speed_right_state)
{
  line_state->integral = 0.0f;
  line_state->previous_error = 0.0f;
  line_state->initialized = 0U;
  yaw_state->integral = 0.0f;
  yaw_state->previous_error = 0.0f;
  yaw_state->initialized = 0U;
  speed_left_state->integral = 0.0f;
  speed_left_state->previous_error = 0.0f;
  speed_left_state->initialized = 0U;
  speed_right_state->integral = 0.0f;
  speed_right_state->previous_error = 0.0f;
  speed_right_state->initialized = 0U;

  control_target_yaw_rate_dps = 0.0f;
  control_measured_yaw_rate_dps = 0.0f;
  control_yaw_diff_mps = 0.0f;
  yaw_diff_slewed = 0.0f;
  /* 停车/复位时把航向角、直角弯状态机全部归零，下次发车从 0 度重新计。 */
  control_yaw_angle = 0.0f;
  corner_state = 0U;
  corner_ticks = 0U;
  corner_detect_ticks = 0U;
  corner_cooldown_ticks = 0U;
  line_searching = 0U;
  control_turn_pwm = 0.0f;
  control_base_pwm = 0.0f;
  control_speed_target_mps = 0.0f;
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

/* 按当前极性统计"压线"的路数，供直角弯检测使用。 */
static uint16_t Line_ActiveCount(void)
{
  uint16_t bits = line_sensor_bits;
  uint8_t level = LINE_SENSOR_ACTIVE_LEVEL;
  uint16_t count = 0U;
  uint16_t channel;

  if (line_polarity_inverted != 0U)
    level = (level == 0U) ? 1U : 0U;

  for (channel = 0U; channel < MULTIPLEXER_CHANNEL_NUM; ++channel)
  {
    if (MUX_GET_CHANNEL(bits, channel) == level)
      ++count;
  }
  return count;
}

/* 循迹误差非线性塑形：output = err * |err| / SHAPE。
   传感器是离散的，正中心附近误差会在 0 和 ±0.67 之间来回跳，
   线性映射会让直道持续小幅打舵（画龙）；而进弯时误差涨到 3~6，
   又需要非常大的打舵量才能及时转进去。
   平方律正好同时满足两头：中心附近增益小 3 倍，大偏差时增益放大 3~6 倍。 */
static float Control_ShapeLineError(float error)
{
  float magnitude = (error < 0.0f) ? -error : error;

  if (LINE_ERROR_SHAPE <= 0.0f)
    return error;
  return error * magnitude / LINE_ERROR_SHAPE;
}

/* 死区补偿：把期望 PWM 线性映射到 [deadzone, CONTROL_MAX_PWM_CCR]。
   占空比很小时也能真正驱动轮子，同时 pwm = 0 时输出仍为 0（可停车）。
   5 PWM 以下直接输出 0：避免内轮在正反转切换点上让 DIR 脚来回抖动。 */
static uint16_t Motor_ApplyDeadzone(float pwm, float deadzone)
{
  float ccr;

  if (pwm <= 5.0f)
    return 0U;
  if (deadzone <= 0.0f)
    return (uint16_t)Control_Clamp(pwm, 0.0f, CONTROL_MAX_PWM_CCR);
  if (deadzone >= CONTROL_MAX_PWM_CCR)
    return (uint16_t)CONTROL_MAX_PWM_CCR;

  ccr = deadzone + (pwm * (CONTROL_MAX_PWM_CCR - deadzone) / CONTROL_MAX_PWM_CCR);
  return (uint16_t)Control_Clamp(ccr, 0.0f, CONTROL_MAX_PWM_CCR);
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
  static ControlLoopState speed_left_state;
  static ControlLoopState speed_right_state;
  static uint16_t line_lost_ticks;
  static uint16_t run_ticks;
  static uint16_t stall_ticks;
  static float yaw_rate_filtered;
  const float dt = (float)CONTROL_UPDATE_MS / 1000.0f;
  float line_error;
  /* 微分用 3 个采样间隔的差商：history 保存 30 ms 前的偏差。 */
  static float line_error_hist[LINE_DERIV_SPAN];
  static uint8_t line_hist_index;
  float line_derivative;
  float yaw_rate;
  float yaw_error;
  float yaw_derivative;
  float speed_error_left;
  float speed_error_right;
  float speed_target_left;
  float speed_target_right;
  float speed_ff_left_pwm;
  float speed_ff_right_pwm;
  float yaw_differential_mps;
  float base_left_pwm;
  float base_right_pwm;
  float average_speed;
  float left_pwm;
  float right_pwm;
  uint8_t left_forward;
  uint8_t right_forward;

#if MOTOR_TEST
  /* 试跑测试直接占用 CCR，若控制器继续运行会每 10 ms 把它清零，
     表现为“给多少占空比都不转”。测试期间完全让出控制权。 */
  return;
#endif

  /* 未按下发车键、IMU 未初始化或已记录故障时，不允许控制输出。 */
  if ((run_button_started == 0U) || (imu_ready == 0U) ||
      (control_fault != FAULT_NONE))
  {
    line_lost_ticks = 0U;
    run_ticks = 0U;
    stall_ticks = 0U;
    VehicleControl_Reset(&line_state, &yaw_state, &speed_left_state, &speed_right_state);
    return;
  }

  /* 角速度数据过期：主循环被长时间阻塞或 SPI 卡死，此时不能用旧值继续闭环。 */
  if ((uint32_t)(HAL_GetTick() - gyro_sample_tick) > GYRO_SAMPLE_TIMEOUT_MS)
  {
    control_fault = FAULT_IMU;
    run_button_started = 0U;
    VehicleControl_Reset(&line_state, &yaw_state, &speed_left_state, &speed_right_state);
    return;
  }

  if (run_ticks < 0xFFFFU)
    ++run_ticks;

  if ((tune_yaw_test_dps > -1.0f) && (tune_yaw_test_dps < 1.0f))
  {
    if (corner_state != 0U)
    {
      /* ---------- 直角弯：光电管已经没有新赛道的信息，改用航向角闭环 ----------
         目标角速度 = 角度环 P × （目标角 − 当前角），外层角度环 + 内层角速度环，
         两环都是我们已经标定好的，所以车头能准确转过设定角度。 */
      float angle_error = corner_target_angle - control_yaw_angle;

      line_error = 0.0f;
      line_derivative = 0.0f;
      line_state.previous_error = 0.0f;
      line_state.initialized = 0U;
      line_lost_ticks = 0U;
      control_target_yaw_rate_dps = Control_Clamp(CORNER_ANGLE_KP * angle_error,
                                                  -CONTROL_MAX_YAW_RATE_DPS,
                                                  CONTROL_MAX_YAW_RATE_DPS);

      if (corner_ticks < 0xFFFFU)
        ++corner_ticks;

      /* 三种退出：角度到位 / 重新压到线且偏差不大 / 超时保护 */
      if (((angle_error < CORNER_EXIT_DEG) && (angle_error > -CORNER_EXIT_DEG)) ||
          ((line_sensor_valid != 0U) &&
           (line_error_milli < CORNER_EXIT_LINE_ERR) &&
           (line_error_milli > -CORNER_EXIT_LINE_ERR)) ||
          (corner_ticks >= CORNER_TIMEOUT_TICKS))
      {
        corner_state = 0U;
        corner_cooldown_ticks = CORNER_COOLDOWN_TICKS;
        line_state.initialized = 0U;
        line_state.previous_error = 0.0f;
        corner_event = 2U;
        corner_event_angle = control_yaw_angle;
      }
    }
    else
    {
      if (corner_cooldown_ticks > 0U)
        --corner_cooldown_ticks;

      if (line_sensor_valid == 0U)
      {
      /* 十字路口、虚线或飞线造成的短暂丢线：保持最后一次偏差继续盲开，
         角速度环仍由陀螺仪闭环以维持航向；超时仍无信号才停机。 */
      if (line_lost_ticks < 0xFFFFU)
        ++line_lost_ticks;
      if (line_lost_ticks < LINE_LOST_HOLD_TICKS)
      {
        /* 保持期间偏差按指数衰减：短暂丢线仍能维持转向，长丢线则迅速摆正。
           微分必须置 0，否则误差会持续累积。 */
        line_error = line_state.previous_error * LINE_LOST_DECAY;
      }
      else if ((tune_search_enable > 0.5f) &&
               (line_lost_ticks < LINE_LOST_SEARCH_TICKS))
      {
        /* 可选：丢线后不停车，直行搜索（用 srch=1 打开，默认关闭）。
           差速清零、保持车速继续往前开，线一回到传感器视野 P 项立刻接管。
           代价是车会继续往赛道外跑，所以默认不开。 */
        line_error = 0.0f;
        line_state.previous_error = 0.0f;
        line_state.initialized = 0U;
        line_searching = 1U;
      }
      else
      {
        /* 默认行为：丢线超过保持窗口就停机。 */
        VehicleControl_Reset(&line_state, &yaw_state, &speed_left_state, &speed_right_state);
        return;
      }
      line_derivative = 0.0f;
    }
    else
    {
      uint8_t i;

      /* 将循迹误差从千分之一权重还原为传感器权重。 */
      line_error = tune_line_sign * ((float)line_error_milli / (float)LINE_ERROR_SCALE);
      /* 重新压线的第一个周期先填满历史，避免偏差跳变造成打舵尖峰 */
      if ((line_state.initialized != 0U) && (line_lost_ticks == 0U))
      {
        /* 微分用 3 个采样间隔（30 ms）的差商，而不是 10 ms 单步差分。
           光电管是整数跳变的，单步差分会带 ±150 单位/秒的量化尖峰，
           D 项一放大就变成抖动源；30 ms 窗口把尖峰压到 1/3，滞后只多 10 ms。 */
        line_derivative = (line_error - line_error_hist[line_hist_index]) /
                          (dt * (float)LINE_DERIV_SPAN);
        /* Z 字弯折角处偏差会瞬间从极负跳到极正，此时差商仍会给出巨大尖峰，
           表现为车身抽一下再反向抽一下（画龙）。两个措施：
           1) 偏差符号翻转的那一拍直接清零 —— 这不是"趋势"，是换向了；
           2) 其余情况给差商限幅。 */
        if (((line_error_milli >= 0) && (prev_line_error_milli < 0)) ||
            ((line_error_milli < 0) && (prev_line_error_milli > 0)))
        {
          line_derivative = 0.0f;
        }
        line_derivative = Control_Clamp(line_derivative,
                                        -LINE_DERIV_LIMIT, LINE_DERIV_LIMIT);
      }
      else
      {
        line_derivative = 0.0f;
      }
      line_error_hist[line_hist_index] = line_error;
      i = (uint8_t)(line_hist_index + 1U);
      line_hist_index = (i >= LINE_DERIV_SPAN) ? 0U : i;
      prev_line_error_milli = line_error_milli;
      line_lost_ticks = 0U;
      line_searching = 0U;
    }
    control_target_yaw_rate_dps = Control_Clamp(
        (tune_line_kp * Control_ShapeLineError(line_error)) + (tune_line_kd * line_derivative),
        -CONTROL_MAX_YAW_RATE_DPS, CONTROL_MAX_YAW_RATE_DPS);
      line_state.previous_error = line_error;
      line_state.initialized = 1U;

      /* 直角弯检测：线跑到最外侧（偏差绝对值很大）且只剩少数几路压线，
         说明车身已经把线甩到传感器边缘，再往前走光电管就看不到新赛道了。
         此时记录当前航向角，转入角度闭环强行转过设定角度。 */
      if ((tune_corner_deg > 1.0f) && (corner_cooldown_ticks == 0U) &&
          (line_sensor_valid != 0U) &&
          ((line_error_milli >= CORNER_DETECT_ERR) ||
           (line_error_milli <= -CORNER_DETECT_ERR)) &&
          (Line_ActiveCount() <= CORNER_DETECT_MAX_ACTIVE) &&
          /* Z 字弯的折角处偏差也会瞬间变到极值，但方向上和上一拍相反；
             要求与上一拍同号，避免把普通 S 弯误判成直角弯。 */
          (((line_error_milli >= 0) && (prev_line_error_milli >= 0)) ||
           ((line_error_milli < 0) && (prev_line_error_milli < 0))))
      {
        if (corner_detect_ticks < CORNER_DETECT_TICKS)
          ++corner_detect_ticks;
      }
      else
      {
        corner_detect_ticks = 0U;
      }

      if (corner_detect_ticks >= CORNER_DETECT_TICKS)
      {
        /* 偏差为负说明线在左侧，需要向左转；航向角正方向就是左转。 */
        corner_target_angle = control_yaw_angle +
            ((line_error_milli < 0) ? tune_corner_deg : -tune_corner_deg);
        corner_state = 1U;
        corner_ticks = 0U;
        corner_detect_ticks = 0U;
        corner_event = 1U;
        corner_event_angle = control_yaw_angle;
      }
    }
  }
  else
  {
    /* 角速度环阶跃测试：忽略循迹（也忽略丢线停机），直接给定目标角速度，
       这样可以在没有赛道的情况下单独验证角速度环的跟踪与稳定性。 */
    line_error = 0.0f;
    line_derivative = 0.0f;
    line_state.previous_error = 0.0f;
    line_state.initialized = 0U;
    line_lost_ticks = 0U;
    control_target_yaw_rate_dps = Control_Clamp(tune_yaw_test_dps,
                                               -CONTROL_MAX_YAW_RATE_DPS,
                                               CONTROL_MAX_YAW_RATE_DPS);
  }

  /* 目标角速度与扣除零偏后的 BMI270 Z 轴角速度之差由 PID 转为差速偏置。
     安装方向修正只作用在这里：调试打印的 gyro_z 始终是传感器原始方向。 */
  yaw_rate = tune_yaw_sign * gyro_z;
  /* 直角弯用的航向角积分：用未滤波的角速度累加，避免低通带来的误差。 */
  control_yaw_angle += yaw_rate * dt;
  /* 一阶低通抑制电机振动带来的高频噪声（只在控制环内使用）。 */
  yaw_rate_filtered = (yaw_rate_filter_alpha * yaw_rate_filtered) +
                      ((1.0f - yaw_rate_filter_alpha) * yaw_rate);
  yaw_rate = yaw_rate_filtered;
  if ((yaw_rate < GYRO_DEADBAND_DPS) && (yaw_rate > -GYRO_DEADBAND_DPS))
    yaw_rate = 0.0f;
  control_measured_yaw_rate_dps = yaw_rate;
  yaw_error = control_target_yaw_rate_dps - yaw_rate;
  yaw_derivative = yaw_state.initialized ?
                   (yaw_error - yaw_state.previous_error) / dt : 0.0f;
  /* 陀螺仪阻尼：对"实测角速度"求微分，而不是对误差求微分。
     对误差求微分会在目标突变时产生很大的尖峰（setpoint kick），
     对实测值求微分只反映车身真实的角加速度，Z 字弯切向时靠它抑制过冲。 */
  yaw_rate_derivative = ((yaw_state.initialized != 0U) &&
                         (run_ticks > MOTOR_GRACE_TICKS)) ?
                        (yaw_rate - yaw_prev_rate) / dt : 0.0f;
  yaw_rate_derivative = Control_Clamp(yaw_rate_derivative,
                                      -YAW_RATE_DERIV_LIMIT, YAW_RATE_DERIV_LIMIT);
  yaw_prev_rate = yaw_rate;
  yaw_state.integral = Control_Clamp(yaw_state.integral + yaw_error * dt,
                                     -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);
  control_turn_pwm = Control_Clamp(
      (tune_yaw_kp * yaw_error) +
      (tune_yaw_ki * yaw_state.integral) +
      (tune_yaw_kd * yaw_derivative) -
      (tune_yaw_rate_damp * yaw_rate_derivative),
      -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);
  yaw_state.previous_error = yaw_error;
  yaw_state.initialized = 1U;

  /* 左右轮各自一套速度 PI：两侧电机、减速箱、摩擦不可能完全一致，
     共用一个 base 时偏弱的一侧会一直慢，车就会持续往一侧偏。
     分开闭环后每个轮子都各自跟到目标速度，直道自然走正。 */
  average_speed = (speed_left_mps + speed_right_mps) * 0.5f;
  /* 方向保护：目标速度为正时两轮平均速度却持续为负，说明编码器方向接反。 */
  if ((run_ticks > MOTOR_GRACE_TICKS) && (tune_target_speed > 0.05f) &&
      (average_speed < MOTOR_REVERSE_SPEED_MPS))
  {
    control_fault = FAULT_DIR;
    run_button_started = 0U;
    VehicleControl_Reset(&line_state, &yaw_state, &speed_left_state, &speed_right_state);
    return;
  }

  /* 差速以“左右轮目标速度偏置”的形式进入速度环，而不是在 PWM 上加减。
     若在 PWM 上加减，每轮速度 PI 会把被减掉的速度补回来，yaw 环给出的差速
     几百毫秒内就被抵消掉（表现为“加大 ykp 车照样走直线”）。
     转成目标偏置后，差速由两个速度环各自实现，二者不再互相打架。 */
  /* 差速 = 前馈（按目标角速度给权限）+ P 项（按误差修正），整体限幅。
     前馈负责"转得动"（急弯需要 0.3 m/s 以上差速），
     P 项负责"转得准"，两者分开后增益与权限不再互相牵制。 */
  {
    float diff_cmd;

    diff_cmd = Control_Clamp(
        (tune_yaw_ff * control_target_yaw_rate_dps) +
        (control_turn_pwm / YAW_DIFF_PWM_PER_MPS),
        -YAW_DIFF_LIMIT_MPS, YAW_DIFF_LIMIT_MPS);

    /* 差速斜坡限制：误差一跳，前馈就会给出一个阶跃的差速指令，
       而轮速从 0.45 变到 −0.2 需要几十毫秒，速度环根本跟不上阶跃，
       结果就是"指令先到位、车身后到位"，必然过冲再摆回来。
       限制变化率后，指令和轮速同速跟上，过冲显著减小。
       12 m/s² 时从 0 到满差速 0.70 需要 58 ms，对 0.35 m/s 的车足够快。 */
    {
      const float max_step = YAW_DIFF_SLEW_MPS2 * dt;

      if (diff_cmd > yaw_diff_slewed + max_step)
        diff_cmd = yaw_diff_slewed + max_step;
      else if (diff_cmd < yaw_diff_slewed - max_step)
        diff_cmd = yaw_diff_slewed - max_step;
    }
    yaw_diff_slewed = diff_cmd;
    yaw_differential_mps = diff_cmd;
  }
  control_yaw_diff_mps = yaw_differential_mps;

  {
    /* 目标速度按加速度上限爬升，避免发车瞬间阶跃造成打滑。
       目标值可在运行中改（在线整定），这里取一次快照保证本轮一致。 */
    float speed_target = tune_target_speed;
    float magnitude;

    /* 动态降速：偏差越大说明正在急弯里，主动把车速压下来，
       让车有时间把姿态换过去；线回到中间后自动恢复。
       Z 字弯（连续反向急弯）靠这一条才不至于惯性累积冲出赛道。 */
    magnitude = (line_error < 0.0f) ? -line_error : line_error;
    if (corner_state != 0U)
    {
      /* 直角弯转角阶段同样降速，避免原地转时外侧轮顶到极限。 */
      speed_target *= 0.80f;
    }
    else if (line_searching != 0U)
    {
      /* 丢线直行搜索：慢一点跑，别一下子冲出很远。 */
      speed_target *= LINE_SEARCH_SPEED_SCALE;
    }
    else if (magnitude >= CORNER_SLOW_ERR_FULL)
    {
      speed_target *= CORNER_SLOW_MIN_SCALE;
    }
    else if (magnitude > CORNER_SLOW_ERR_START)
    {
      float t = (magnitude - CORNER_SLOW_ERR_START) /
                (CORNER_SLOW_ERR_FULL - CORNER_SLOW_ERR_START);
      speed_target *= 1.0f - (t * (1.0f - CORNER_SLOW_MIN_SCALE));
    }
    /* 绝对下限：轮子贴死区时速度环会抖，宁可少降一点也不降进死区。 */
    if (speed_target < CONTROL_SPEED_MIN_MPS)
      speed_target = CONTROL_SPEED_MIN_MPS;

    if (control_speed_target_mps < speed_target)
    {
      control_speed_target_mps += CONTROL_ACCEL_LIMIT_MPS2 * dt;
      if (control_speed_target_mps > speed_target)
        control_speed_target_mps = speed_target;
    }
    else if (control_speed_target_mps > speed_target)
    {
      /* 减速用更大的斜率，否则进弯前来不及降下来。 */
      control_speed_target_mps -= CONTROL_DECEL_LIMIT_MPS2 * dt;
      if (control_speed_target_mps < speed_target)
        control_speed_target_mps = speed_target;
    }
  }

  /* 左右轮各自的目标速度：基速 ± 差速偏置。
     内轮允许降到 YAW_INNER_REVERSE_MPS 的负值（反向转），
     这是把差速做大的唯一途径：差速 = (外轮 - 内轮)/2，
     内轮从 0 变成负值后同样的外轮转速能得到近两倍的差速。 */
  speed_target_left = Control_Clamp(control_speed_target_mps - yaw_differential_mps,
                                    -tune_inner_reverse_mps, CONTROL_SPEED_TARGET_LIMIT_MPS);
  speed_target_right = Control_Clamp(control_speed_target_mps + yaw_differential_mps,
                                     -tune_inner_reverse_mps, CONTROL_SPEED_TARGET_LIMIT_MPS);

  /* 零速（停车或尚未起步）时不允许任何差速：
     陀螺仪残余零偏会被角速度环放大成一个持续的差速，
     表现为“目标速度 0 但车自己在地上爬/打转”。 */
  if (control_speed_target_mps <= 0.0f)
  {
    speed_target_left = 0.0f;
    speed_target_right = 0.0f;
    yaw_differential_mps = 0.0f;
  }

  /* 前馈按各自的目标速度给开环油门，PI 只修正残差。
     目标速度可能为负（内轮反转），所以这里要保号，上下限对称。 */
  speed_ff_left_pwm = Control_Clamp(tune_speed_ff * speed_target_left,
                                    -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);
  speed_ff_right_pwm = Control_Clamp(tune_speed_ff * speed_target_right,
                                     -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);

  speed_error_left = speed_target_left - speed_left_mps;
  /* 反转自检：目标速度已经是负值（要求倒转），实测却仍在向前跑，
     说明 DIR 方向脚没有真正换向（接线/电平/驱动芯片问题）。
     此时"内轮反转"不但无效，还会让差速被吃掉一半，Z 字弯必然转不过。
     连续 400 ms 满足就报一次，主循环里打印。 */
  if ((speed_target_left < -0.05f) && (speed_left_mps > 0.08f))
  {
    if (rev_check_ticks < 0xFFFFU)
      ++rev_check_ticks;
    if (rev_check_ticks >= REV_CHECK_TICKS)
      rev_fail_flag = 1U;
  }
  else
  {
    rev_check_ticks = 0U;
  }

  speed_left_state.integral = Control_Clamp(speed_left_state.integral + speed_error_left * dt,
                                            -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
  base_left_pwm = Control_Clamp(
      speed_ff_left_pwm +
      (tune_speed_kp * speed_error_left) + (tune_speed_ki * speed_left_state.integral),
      -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);
  speed_left_state.previous_error = speed_error_left;
  speed_left_state.initialized = 1U;

  speed_error_right = speed_target_right - speed_right_mps;
  speed_right_state.integral = Control_Clamp(speed_right_state.integral + speed_error_right * dt,
                                             -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
  base_right_pwm = Control_Clamp(
      speed_ff_right_pwm +
      (tune_speed_kp * speed_error_right) + (tune_speed_ki * speed_right_state.integral),
      -CONTROL_MAX_PWM_CCR, CONTROL_MAX_PWM_CCR);
  speed_right_state.previous_error = speed_error_right;
  speed_right_state.initialized = 1U;

  /* 直行搜索：两轮强制用同一个 PWM，并把速度积分清零。
     实测日志里搜索阶段两轮交替掉到 CCR=0（滑行）而另一轮仍在出力，
     车就一边"直行搜索"一边自己转向，越搜越偏。
     搜索阶段根本不需要差速，两轮同油门才能走直。 */
  if (line_searching != 0U)
  {
    float mean_pwm = (base_left_pwm + base_right_pwm) * 0.5f;

    base_left_pwm = mean_pwm;
    base_right_pwm = mean_pwm;
    speed_left_state.integral = 0.0f;
    speed_right_state.integral = 0.0f;
  }

  /* 调试量：base 仍输出左右平均，便于和旧打印格式对照。 */
  control_base_pwm = (base_left_pwm + base_right_pwm) * 0.5f;

  left_pwm = base_left_pwm;
  right_pwm = base_right_pwm;
  /* 死区补偿只作用于最终写进 CCR 的值，控制环内部仍用未补偿的 PWM。
     PWM 为负表示该轮要反转：占空比取绝对值，方向交给 DIR 脚。 */
  left_forward = (left_pwm >= 0.0f) ? 1U : 0U;
  right_forward = (right_pwm >= 0.0f) ? 1U : 0U;
  control_left_pwm = Motor_ApplyDeadzone(left_forward ? left_pwm : -left_pwm, tune_deadzone_l);
  control_right_pwm = Motor_ApplyDeadzone(right_forward ? right_pwm : -right_pwm, tune_deadzone_r);

  /* 电机输出需人工完成标定后才可启用；当前编译配置保持关闭。 */
  if (MOTOR_CONTROL_ENABLE != 0U)
  {
    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin,
                      (left_forward != 0U) ? L_DIR_FORWARD : L_DIR_REVERSE);
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin,
                      (right_forward != 0U) ? R_DIR_FORWARD : R_DIR_REVERSE);
    TIM1->CCR1 = control_left_pwm;
    TIM1->CCR2 = control_right_pwm;
  }

  /* 堵转保护：指令已超过起转占空比，两轮却都几乎不转，判定卡死并停机。
     发车后留 MOTOR_RUN_GRACE_MS 的起步时间，避免把正常起步误判成卡死。
     台架调参时可用 stl=0 关闭（车轮架空或被人按住时不会误停）。 */
  if ((MOTOR_CONTROL_ENABLE != 0U) && (tune_stall_enable != 0U) &&
      (run_ticks > MOTOR_GRACE_TICKS) &&
      ((control_left_pwm >= MOTOR_STALL_PWM_MIN) ||
       (control_right_pwm >= MOTOR_STALL_PWM_MIN)) &&
      (average_speed < MOTOR_STALL_SPEED_MPS) &&
      (average_speed > -MOTOR_STALL_SPEED_MPS))
  {
    if (stall_ticks < 0xFFFFU)
      ++stall_ticks;
    if (stall_ticks >= MOTOR_STALL_TICKS)
    {
      /* 记录故障瞬间的速度与占空比：速度全 0 才是编码器没读数，
         有速度却停机说明是机械卡住或被人按住。 */
      fault_speed_l_mmps = (int32_t)(speed_left_mps * 1000.0f);
      fault_speed_r_mmps = (int32_t)(speed_right_mps * 1000.0f);
      fault_ccr_l = control_left_pwm;
      fault_ccr_r = control_right_pwm;
      control_fault = FAULT_STALL;
      run_button_started = 0U;
      VehicleControl_Reset(&line_state, &yaw_state, &speed_left_state, &speed_right_state);
    }
  }
  else
  {
    stall_ticks = 0U;
  }
}

/* 非阻塞消抖：PB14 按下电平持续 20 ms 后判定为一次按键，
   每次按下在"发车 / 停车"之间切换。 */
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

    /* 本项目按低电平按下处理。 */
    if (stable_state == RUN_BUTTON_PRESSED)
    {
      /* 每次按键都清零累计计数，便于“清零 → 推行 1 m → 读 odom”标定。 */
      odom_left = 0;
      odom_right = 0;

      if (run_button_started == 0U)
      {
        /* 重新发车：清故障码，零偏用停稳期间累积的样本重新校准。 */
        control_fault = FAULT_NONE;
        /* 若极性自动判定还没完成，就以当前结果锁定，避免发车后一直无有效循迹量 */
        if (line_polarity_locked == 0U)
        {
          line_polarity_locked = 1U;
          printf("line,pol,%u\r\n", (unsigned int)line_polarity_inverted);
        }
        run_button_started = 1U;
      }
      else
      {
        /* 停车：控制环下一个 10 ms 节拍会清零 PWM 与积分，
           同时重新打开零偏校准窗口，便于下次发车前重新测零偏。 */
        run_button_started = 0U;
        GyroBias_Restart();
      }
    }
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

/* 独立看门狗直接用寄存器配置，不依赖 HAL_IWDG 驱动和 CubeMX 生成配置。
   LSI 约 40 kHz，预分频 64 得到 625 Hz，重装值 499 约 0.8 s 超时。
   IWDG 一旦启动无法关闭，调试阶段务必将 WATCHDOG_ENABLE 保持 0。 */
static void Watchdog_Init(void)
{
#if WATCHDOG_ENABLE
  IWDG->KR = 0xCCCCU;   /* 启动 IWDG */
  IWDG->KR = 0x5555U;   /* 允许写 PR / RLR */
  IWDG->PR = 0x04U;     /* LSI / 64 */
  IWDG->RLR = 499U;     /* 约 0.8 s */
  while (IWDG->SR != 0U)
  {
  }
  IWDG->KR = 0xAAAAU;   /* 首次喂狗 */
#endif
}

static void Watchdog_Feed(void)
{
#if WATCHDOG_ENABLE
  IWDG->KR = 0xAAAAU;
#endif
}/**
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
    gyro_sample_tick = HAL_GetTick();
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

  /* 看门狗最后启动：前面的初始化（含负压启动 2 s 等待）不受其超时约束。 */
  Watchdog_Init();

#if TUNE_ENABLE
  /* 启动 USART3 单字节接收中断：在线整定的数据入口（有线/无线共用）。 */
  if (HAL_UART_Receive_IT(&huart3, &tune_rx_byte, 1U) != HAL_OK)
    printf("tune,rx_fail\r\n");
  else
    printf("tune,ready,%s %s\r\n", __DATE__, __TIME__);

#if TUNE_LOAD_ON_BOOT
  /* 上电优先使用 Flash 中已保存的整定参数；无有效存档时保持宏默认值。 */
  printf("flash,load,%u\r\n", (unsigned int)Tune_LoadFromFlash());
#endif
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Watchdog_Feed();
    RunButton_Update();
#if TUNE_ENABLE
    Tune_Update();
#endif

    /* 发车状态变化时报告一次：1 = 运行中，0 = 已停车。 */
    {
      static uint8_t run_state_reported;
      if (run_state_reported != run_button_started)
      {
        run_state_reported = run_button_started;
        printf("run,%u\r\n", (unsigned int)run_state_reported);
      }
    }

    /* 直角弯事件：控制环只登记，打印放在主循环，避免阻塞 10 ms 控制中断。 */
    if (corner_event != 0U)
    {
      uint8_t what = corner_event;
      corner_event = 0U;
      printf("corner,%s,%ld\r\n", (what == 1U) ? "start" : "done",
             (long)(corner_event_angle * 10.0f));
    }

    /* 内轮反转自检：只报一次，提示方向脚没有真正换向。 */
    if (rev_fail_flag != 0U)
    {
      rev_fail_flag = 0U;
      printf("revfail\r\n");
    }

    /* 故障由控制环置位并已停机，这里只负责报告一次；故障清除后可再次报告。 */
    {
      static uint8_t fault_reported;
      if (control_fault == FAULT_NONE)
      {
        fault_reported = FAULT_NONE;
      }
      else if (fault_reported != control_fault)
      {
        fault_reported = control_fault;
        /* 堵转故障附带现场数据：spdL/spdR 单位 mm/s，CCR 为实际写入值。 */
        if (control_fault == FAULT_STALL)
        {
          printf("fault,%u,%ld,%ld,%u,%u\r\n", (unsigned int)control_fault,
                 (long)fault_speed_l_mmps, (long)fault_speed_r_mmps,
                 (unsigned int)fault_ccr_l, (unsigned int)fault_ccr_r);
        }
        else
        {
          printf("fault,%u\r\n", (unsigned int)control_fault);
        }
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
      if ((debug_print_on != 0U) && (YawTest_Active() == 0U) &&
          ((HAL_GetTick() - line_print_tick) >= LINE_PRINT_MS))
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
    /* 累计计数：标定“每米计数”用，按一次发车键清零。 */
    {
      static uint32_t odom_print_tick;
      if ((debug_print_on != 0U) &&
          ((HAL_GetTick() - odom_print_tick) >= ODOM_PRINT_MS))
      {
        odom_print_tick = HAL_GetTick();
        printf("odom,%ld,%ld\r\n",
               (long)odom_left, (long)odom_right);
      }
    }
    {
      static uint32_t speed_print_tick;
      if ((debug_print_on != 0U) && (YawTest_Active() == 0U) &&
          ((HAL_GetTick() - speed_print_tick) >= SPEED_PRINT_MS))
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
      if ((debug_print_on != 0U) &&
          ((HAL_GetTick() - control_print_tick) >= SPEED_PRINT_MS))
      {
        control_print_tick = HAL_GetTick();
        printf("ctrl,%u,%u,%u,%ld,%ld,%ld,%u,%u,%ld,%ld,%ld\r\n",
               (unsigned int)run_button_started,
               (unsigned int)line_sensor_valid,
               (unsigned int)imu_ready,
               (long)(control_target_yaw_rate_dps * 1000.0f),
               (long)(control_turn_pwm * 1000.0f),
               (long)(control_base_pwm * 1000.0f),
               (unsigned int)control_left_pwm,
               (unsigned int)control_right_pwm,
               (long)(control_measured_yaw_rate_dps * 1000.0f),
               (long)(control_yaw_diff_mps * 1000.0f),
               (long)(control_yaw_angle * 10.0f));
      }
    }
#endif
    // 以下为陀螺仪使用示例
    dodo_BMI270_get_data();                         // 调用此函数会更新陀螺仪数据
    gyro_sample_tick = HAL_GetTick();               // 控制环用它判断角速度数据是否过期
    {
      /* 原始 Z 轴角速度先参与零偏采样；扣除零偏并按安装方向修正后才送控制环。 */
      float raw_gyro_z = BMI270_gyro_transition(BMI270_gyro_z);
      GyroBias_Update(raw_gyro_z);
      /* gyro_z 保持原始方向（未乘 YAW_RATE_SIGN），这样人工转车检查符号时看到的
         就是传感器真实方向；方向修正只在控制环内部应用。 */
      gyro_z = raw_gyro_z - gyro_z_offset + tune_gyro_trim;
    }
    gyro_x = BMI270_gyro_transition(BMI270_gyro_x); // 将原始陀螺仪数据转换为物理值，单位为度每秒
    gyro_y = BMI270_gyro_transition(BMI270_gyro_y);
    accel_x = BMI270_acc_transition(BMI270_accel_x); // 将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
    accel_y = BMI270_acc_transition(BMI270_accel_y);
    accel_z = BMI270_acc_transition(BMI270_accel_z);
    if ((GYRO_DEBUG) && (debug_print_on != 0U))
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
  odom_left  += left;
  odom_right += right;
  speed_left_mps  = (float)enc_left_sum  * ENC_COUNTS_TO_MPS;
  speed_right_mps = (float)enc_right_sum * ENC_COUNTS_TO_MPS;
}

#if TUNE_ENABLE
/* USART3 接收完成回调：只做入队和重新挂接收，解析交给主循环。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    const uint16_t next = (uint16_t)((tune_rx_head + 1U) % TUNE_RX_BUFFER_SIZE);

    if (next != tune_rx_tail)
    {
      tune_rx_buffer[tune_rx_head] = tune_rx_byte;
      tune_rx_head = next;
    }
    HAL_UART_Receive_IT(&huart3, &tune_rx_byte, 1U);
  }
}

/* 无线串口容易因干扰产生溢出错误，HAL 会停止接收；这里清标志并重新挂接收，
   否则调参链路会静默失效。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart3, &tune_rx_byte, 1U);
  }
}
#endif

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
