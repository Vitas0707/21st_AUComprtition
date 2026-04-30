#include "Motor.h"
#include "main.h"
#include <math.h>
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* 运行时可配置的起动助力参数（默认为头文件中的宏） */
static uint16_t g_start_threshold_percent = MOTOR_START_THRESHOLD_PERCENT;
static uint16_t g_start_boost_percent = MOTOR_START_BOOST_PERCENT;
static uint32_t g_start_boost_ms = MOTOR_START_BOOST_MS;
/**
 * @file Motor.c
 * @brief Motor 模块实现（PWM、方向、编码器与 PID）
 *
 * 依赖：HAL 定时器与 GPIO 已由 CubeMX 或用户代码正确初始化。
 */

/**
 * @brief 启动 TIM Base 与对应 PWM 通道
 * @param[in] htim 要启动的 TIM 句柄
 * @param[in] Channel PWM 通道（例如 TIM_CHANNEL_1）
 * @note 在使用 PWM 输出前调用一次，或由 Motor_WheelInit 内部调用
 * @code
 * Motor_Init(&htim3, TIM_CHANNEL_1);
 * @endcode
 */
void Motor_Init(TIM_HandleTypeDef* htim, uint32_t Channel) {
    if (htim == NULL) return;
    HAL_TIM_Base_Start(htim);
    HAL_TIM_PWM_Init(htim);
    HAL_TIM_PWM_Start(htim, Channel);
}

/**
 * @brief 设置 PWM 占空比（百分比）并写入比较寄存器
 * @param[in] htim PWM 定时器句柄
 * @param[in] Channel PWM 通道
 * @param[in] pwm_percent 占空比（0..100）
 * @note 占空比按定时器的 ARR 映射到 CCR
 * @code
 * Motor_SetPWM(&htim3, TIM_CHANNEL_1, 75);
 * @endcode
 */
void Motor_SetPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t pwm_percent) {
    if (htim == NULL) return;
    if (pwm_percent > 100) pwm_percent = 100;
    uint32_t arr = (uint32_t)(htim->Instance->ARR);
    uint32_t ccr = (uint32_t)((((uint32_t)pwm_percent) * (arr + 1U)) / 100U);
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}

/**
 * @brief 设置电机方向
 * @param[in] GPIOx IN1/IN2 所在 GPIO 端口
 * @param[in] GPIO_Pin1 IN1 引脚
 * @param[in] GPIO_Pin2 IN2 引脚
 * @param[in] direction 方向标志（0: IN1=高、IN2=低; 非0: IN1=低、IN2=高）
 * @code
 * Motor_SetDirection(GPIOB, GPIO_PIN_0, GPIO_PIN_1, 0);
 * @endcode
 */
void Motor_SetDirection(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin1,uint16_t GPIO_Pin2,uint8_t direction) {
    if (direction == 0) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin2, GPIO_PIN_SET);
    }
}

/**
 * @brief 初始化轮子封装并启动相关外设（若提供）
 * @param[in,out] m 要初始化的 Motor_t 结构体指针
 * @param[in] htim_pwm PWM 定时器句柄
 * @param[in] pwm_channel PWM 通道
 * @param[in] in_port 方向引脚所在 GPIO 端口
 * @param[in] in_pin1 IN1 引脚
 * @param[in] in_pin2 IN2 引脚
 * @param[in] htim_encoder 编码器 TIM（Encoder 模式），可为 NULL
 * @param[in] wheel_diameter_m 轮子直径（米）
 * @param[in] encoder_ppr 编码器机械 PPR（每转脉冲数），代码按 4x 计数处理
 * @param[in] gear_ratio 齿比 motor_rot / wheel_rot
 * @note 如果提供了编码器句柄, 会使用 HAL_TIM_Encoder_Start 启动
 */
void Motor_WheelInit(Motor_t* m,
                     TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
                     GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
                     TIM_HandleTypeDef* htim_encoder,
                     float wheel_diameter_m, uint32_t encoder_ppr, float gear_ratio) {
    if (!m) return;
    m->htim_pwm = htim_pwm;
    m->pwm_channel = pwm_channel;
    m->in_port = in_port;
    m->in_pin1 = in_pin1;
    m->in_pin2 = in_pin2;
    m->htim_encoder = htim_encoder;
    m->encoder_ppr = encoder_ppr;
    m->wheel_diameter_m = wheel_diameter_m;
    m->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->last_encoder_count = 0;
    m->total_counts = 0;
    m->speed_m_s = 0.0f;
    m->rpm = 0.0f;
    m->last_tick_ms = 0;
    m->last_pwm_percent = 0;
    m->boost_end_ms = 0;
    m->invert_direction = false;
    m->pid.kp = 0.0f; m->pid.ki = 0.0f; m->pid.kd = 0.0f;
    m->pid.integrator = 0.0f; m->pid.prev_error = 0.0f;
    m->pid.integrator_limit = 1000.0f; m->pid.output_limit = 100.0f;

    if (htim_pwm) {
        Motor_Init(htim_pwm, pwm_channel);
        Motor_SetPWM(htim_pwm, pwm_channel, 0);
    }
    if (htim_encoder) {
        HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL);
        m->last_encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(htim_encoder);
        /* keep last_tick_ms == 0 so first Motor_UpdateEncoder call sets baseline */
    }
}

/**
 * @brief 设置方向并将 PWM 设置为指定占空比，启动电机
 * @param[in,out] m 目标 Motor_t
 * @param[in] pwm_percent 占空比 0..100
 * @param[in] direction 方向标志（0 或 1，见 Motor_SetDirection）
 * @code
 * Motor_StartWheel(&wheel, 60, 0); // 60% 正向
 * @endcode
 */
void Motor_StartWheel(Motor_t* m, uint16_t pwm_percent, uint8_t direction) {
    if (!m) return;
    if (pwm_percent > 100) pwm_percent = 100;
    uint32_t now = HAL_GetTick();

    /* 决定实际写入的占空比：起动助力逻辑（使用可配置的全局参数） */
    uint16_t effective_pwm = pwm_percent;
    /* 若请求非零且上次为 0（刚从停止状态开始），且请求低于阈值，则开启短时助力 */
    if (pwm_percent > 0 && m->last_pwm_percent == 0) {
        if (pwm_percent < g_start_threshold_percent) {
            m->boost_end_ms = now + g_start_boost_ms;
            effective_pwm = g_start_boost_percent;
        }
    }
    /* 如果处在助力期，则继续使用助力占空比 */
    if (m->boost_end_ms != 0 && (int32_t)(m->boost_end_ms - now) > 0) {
        effective_pwm = g_start_boost_percent;
    }

    /* 应用 per-wheel 方向反转（若配置） */
    uint8_t actual_dir = (uint8_t)(direction ^ (m->invert_direction ? 1U : 0U));
    Motor_SetDirection(m->in_port, m->in_pin1, m->in_pin2, actual_dir);
    Motor_SetPWM(m->htim_pwm, m->pwm_channel, effective_pwm);

    /* 保存请求的占空比（用于下一次判断是否从停止开始） */
    m->last_pwm_percent = pwm_percent;
}

/**
 * @brief 停止电机并将方向引脚拉低
 * @param[in,out] m 目标 Motor_t
 */
void Motor_StopWheel(Motor_t* m) {
    if (!m) return;
    Motor_SetPWM(m->htim_pwm, m->pwm_channel, 0);
    HAL_GPIO_WritePin(m->in_port, m->in_pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in_port, m->in_pin2, GPIO_PIN_RESET);
    /* 停止时清除助力相关状态 */
    m->last_pwm_percent = 0;
    m->boost_end_ms = 0;
}

void Motor_SetDirectionInverted(Motor_t* m, bool inverted) {
    if (!m) return;
    m->invert_direction = inverted ? true : false;
}

/**
 * @brief 更新编码器计数并计算速度与里程
 * @param[in,out] m 目标 Motor_t
 * @note 读取 TIM CNT，计算差值并处理计数器回绕，按 encoder_ppr*gear_ratio*4 换算为转数
        首次调用若 m->last_tick_ms == 0，将建立基准（仅采样并返回），随后调用才进行速度计算
+ * @attention 建议 10~100 ms 周期调用
 */
void Motor_UpdateEncoder(Motor_t* m) {
    if (!m || !m->htim_encoder) return;
    uint32_t now = HAL_GetTick();

    uint32_t arr = (uint32_t)(m->htim_encoder->Instance->ARR);
    uint32_t cnt = __HAL_TIM_GET_COUNTER(m->htim_encoder);

    if (m->last_tick_ms == 0) {
        m->last_encoder_count = (int32_t)cnt;
        m->last_tick_ms = now;
        return;
    }

    uint32_t dt_ms = now - m->last_tick_ms;
    if (dt_ms == 0) return;

    int32_t last = m->last_encoder_count;
    int32_t delta = (int32_t)cnt - last;
    int32_t arr_plus = (int32_t)arr + 1;
    if (delta > (arr_plus / 2)) delta -= arr_plus;
    else if (delta < -(arr_plus / 2)) delta += arr_plus;

    m->last_encoder_count = (int32_t)cnt;
    m->total_counts += (int64_t)delta;

    float dt_s = ((float)dt_ms) / 1000.0f;
    float counts_per_rev = (float)m->encoder_ppr * m->gear_ratio * MG310_QUADRATURE_MULT;//计算每转脉冲数（输出轴转动一圈对应的编码器计数）
    if (counts_per_rev <= 0.0f) counts_per_rev = 1.0f;
    float revolutions = ((float)delta) / counts_per_rev;//计算增量转数
    float revs_per_sec = revolutions / dt_s;//计算转速（转/秒）
    m->rpm = revs_per_sec * 60.0f;//更新转速（RPM）（转/分钟）
    float circumference = M_PI * m->wheel_diameter_m;
    m->speed_m_s = revs_per_sec * circumference;//更新线速度（米/秒）

    m->last_tick_ms = now;
}

/**
 * @brief 获取速度（米/秒）
 * @param[in] m 目标 Motor_t
 * @return 最近一次计算得到的速度（米/秒）
 */
float Motor_GetSpeedMps(Motor_t* m) {
    if (!m) return 0.0f;
    return m->speed_m_s;
}

/**
 * @brief 获取累计里程（米）
 * @param[in] m 目标 Motor_t
 * @return 累计里程（米）
 */
float Motor_GetDistanceM(Motor_t* m) {
    if (!m) return 0.0f;
    float counts_per_rev = (float)m->encoder_ppr * m->gear_ratio * MG310_QUADRATURE_MULT;//输出轴转动一圈对应的编码器计数
    if (counts_per_rev <= 0.0f) return 0.0f;
    float revolutions = ((float)m->total_counts) / counts_per_rev;
    float circumference = M_PI * m->wheel_diameter_m;
    return revolutions * circumference;
}

/**
 * @brief 重置累计里程计数
 * @param[in,out] m 目标 Motor_t
 */
void Motor_ResetDistance(Motor_t* m) {
    if (!m) return;
    m->total_counts = 0;
}

/**
 * @brief 设置 PID 参数并重置内部状态
 * @param[in,out] m 目标 Motor_t
 * @param[in] kp 比例系数
 * @param[in] ki 积分系数
 * @param[in] kd 微分系数
 * @param[in] integrator_limit 积分器饱和限制（绝对值）
 * @param[in] output_limit 输出限制（例如 pwm 百分比最大值）
 */
void Motor_PIDSetParams(Motor_t* m, float kp, float ki, float kd, float integrator_limit, float output_limit) {
    if (!m) return;
    m->pid.kp = kp;
    m->pid.ki = ki;
    m->pid.kd = kd;
    m->pid.integrator = 0.0f;
    m->pid.prev_error = 0.0f;
    m->pid.integrator_limit = integrator_limit;
    m->pid.output_limit = output_limit;
}

/**
 * @brief 重置 PID 内部状态（积分器和前次误差）
 * @param[in,out] m 目标 Motor_t
 */
void Motor_PIDReset(Motor_t* m) {
    if (!m) return;
    m->pid.integrator = 0.0f;
    m->pid.prev_error = 0.0f;
}

/**
 * @brief 计算 PID 输出
 * @param[in,out] m 目标 Motor_t
 * @param[in] target_speed_m_s 目标速度（m/s）
 * @param[in] dt_s 控制周期（秒），如果 <=0 将使用 m->last_tick_ms 估算
 * @return 带符号控制量（-output_limit .. +output_limit），符号表示方向
 * @code
 * float ctrl = Motor_PIDCompute(&wheel, 0.5f, dt);
 * if (ctrl >= 0) Motor_StartWheel(&wheel, (uint16_t)ctrl, 0);
 * else           Motor_StartWheel(&wheel, (uint16_t)(-ctrl), 1);
 * @endcode
 */
float Motor_PIDCompute(Motor_t* m, float target_speed_m_s, float dt_s) {
    if (!m) return 0.0f;
    if (dt_s <= 0.0f) {
        uint32_t now = HAL_GetTick();
        dt_s = ((float)(now - m->last_tick_ms)) / 1000.0f;//保证时间不为0或者负数
        if (dt_s <= 0.0f) dt_s = 1e-3f;
    }
    
    float error = target_speed_m_s - m->speed_m_s;//误差
    m->pid.integrator += error * dt_s;//积分项
    if (m->pid.integrator > m->pid.integrator_limit) m->pid.integrator = m->pid.integrator_limit;
    if (m->pid.integrator < -m->pid.integrator_limit) m->pid.integrator = -m->pid.integrator_limit;
    float derivative = (error - m->pid.prev_error) / dt_s;//微分项
    float output = m->pid.kp * error + m->pid.ki * m->pid.integrator + m->pid.kd * derivative;//计算输出控制量
    m->pid.prev_error = error;//更新前次误差
    if (output > m->pid.output_limit) output = m->pid.output_limit;
    if (output < -m->pid.output_limit) output = -m->pid.output_limit;
    return output;
}

/* Helper: 将距离（米）换算为编码器脉冲数（带符号） */
static int64_t Motor_DistanceToCounts(Motor_t* m, float distance_m) {
    if (!m) return 0;
    float counts_per_rev = (float)m->encoder_ppr * m->gear_ratio * MG310_QUADRATURE_MULT;//转一圈的脉冲数
    float circumference = M_PI * m->wheel_diameter_m;//轮子周长
    float revs = distance_m / circumference;//需要的转数
    float counts_f = revs * counts_per_rev;//对应的脉冲数（浮点）
    int64_t counts = (int64_t)(counts_f >= 0.0f ? counts_f + 0.5f : counts_f - 0.5f);//四舍五入取整
    return counts;
}

/**
 * @brief 初始化一个基于 MG310 的轮子（便捷封装）
 */
void Motor_InitMG310Wheel(Motor_t* m,
                          TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
                          GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
                          TIM_HandleTypeDef* htim_encoder) {
    Motor_WheelInit(m, htim_pwm, pwm_channel, in_port, in_pin1, in_pin2,
                    htim_encoder, MG310_WHEEL_DIAMETER_M, MG310_ENCODER_PPR, MG310_GEAR_RATIO);
    /* Apply sensible defaults for MG310 wheels */
    Motor_PIDSetParams(m, 1.0f, 0.1f, 0.01f, 1000.0f, 100.0f);
    Motor_PIDReset(m);
    Motor_ResetDistance(m);
    Motor_SetDirectionInverted(m, false);
    /* Ensure start-boost params match header defaults (no-op if already default) */
    Motor_SetStartBoostParams(MOTOR_START_THRESHOLD_PERCENT, MOTOR_START_BOOST_PERCENT, MOTOR_START_BOOST_MS);
}

/**
 * @brief 初始化底盘结构
 */
void RobotDrive_Init(RobotDrive_t* r, Motor_t* fl, Motor_t* fr, Motor_t* rl, Motor_t* rr,
                     float track_width_m, float wheel_base_m) {
    if (!r) return;
    r->fl = fl;
    r->fr = fr;
    r->rl = rl;
    r->rr = rr;
    r->track_width_m = track_width_m;
    r->wheel_base_m = wheel_base_m;
}

/**
 * @brief 阻塞式移动：按指定距离前进/后退（米）
 */
int Robot_MoveDistanceBlocking(RobotDrive_t* r, float distance_m, float speed_m_s) {
    if (!r || !r->fl || !r->fr || !r->rl || !r->rr) return -1;
    Motor_t* wheels[4] = { r->fl, r->fr, r->rl, r->rr };
    int64_t target_counts = Motor_DistanceToCounts(wheels[0], distance_m);//将目标距离转换为目标编码器计数增量
    if (target_counts == 0) return 0;
    int sign = (distance_m >= 0.0f) ? 1 : -1;

    int64_t start_counts[4];
    for (int i = 0; i < 4; ++i) start_counts[i] = wheels[i]->total_counts;

    const uint32_t control_ms = 10;
    const float control_s = control_ms / 1000.0f;
    uint32_t t0 = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)((fabsf(distance_m) / fmaxf(fabsf(speed_m_s), 0.01f)) * 1000.0f) + 2000U;

    while (1) {
        uint32_t now = HAL_GetTick();
        // 更新编码器
        for (int i = 0; i < 4; ++i) Motor_UpdateEncoder(wheels[i]);

        // 计算平均已行进距离
        float sum_m = 0.0f;
        for (int i = 0; i < 4; ++i) {
            int64_t delta = wheels[i]->total_counts - start_counts[i];
            float counts_per_rev = (float)wheels[i]->encoder_ppr * wheels[i]->gear_ratio * MG310_QUADRATURE_MULT;
            float revs = (float)delta / counts_per_rev;
            float travelled = revs * (M_PI * wheels[i]->wheel_diameter_m);
            sum_m += travelled;
        }
        float avg_m = sum_m / 4.0f;

        // 判断是否达到目标
        if (sign > 0) {
            if (avg_m >= distance_m - 0.002f) break;
        } else {
            if (avg_m <= distance_m + 0.002f) break;
        }

        // 超时保护
        if ((now - t0) > timeout_ms) break;

        /* 位置优先：外环按剩余距离生成速度参考，速度为次要目标 */
        {
            float pos_kp = MOTOR_POS_KP;//位置控制比例系数
            float min_speed = MOTOR_POS_MIN_SPEED_M_S;//位置控制的最小速度（m/s），确保在接近目标时仍有足够动力克服静摩擦
            float tol = MOTOR_POS_TOLERANCE_M;
            float default_max_speed = 5.0f;
            float max_speed = (speed_m_s > 0.0f) ? fabsf(speed_m_s) : default_max_speed;

            float remaining = distance_m - avg_m; /* 带符号的剩余距离 */
            float abs_rem = fabsf(remaining);
            float desired_speed = fminf(max_speed, pos_kp * abs_rem);
            if (abs_rem > tol && desired_speed < min_speed) desired_speed = min_speed;
            int move_sign = (remaining >= 0.0f) ? 1 : -1;


            for (int i = 0; i < 4; ++i) {
                Motor_t* w = wheels[i];
                float pid_out = Motor_PIDCompute(w, desired_speed * (float)move_sign, control_s);
                
                if (fabsf(pid_out) > 0 && fabsf(pid_out) < MIN_PWM_THRESHOLD) {
                pid_out = (pid_out >= 0) ? MIN_PWM_THRESHOLD : -MIN_PWM_THRESHOLD;
                }

                uint16_t pwm = (uint16_t)fminf(fabsf(pid_out), 100.0f);
                uint8_t dir = (pid_out >= 0.0f) ? 0 : 1;
                Motor_StartWheel(w, pwm, dir);
            }
        }

        HAL_Delay(control_ms);
    }

    // 停止所有轮子
    for (int i = 0; i < 4; ++i) Motor_StopWheel(wheels[i]);
    return 0;
}

/**
 * @brief 阻塞式自转到指定角度（单位度）
 */
int Robot_TurnAngleBlocking(RobotDrive_t* r, float angle_deg, float angular_speed_deg_s) {
    if (!r || !r->fl || !r->fr || !r->rl || !r->rr) return -1;
    // 自转时左右轮线速度大小相等方向相反
    float theta = angle_deg * (M_PI / 180.0f);
    float half_track = r->track_width_m * 0.5f;
    float wheel_s = half_track * theta; // 单侧轮子需行走的弧长（m），左为 -wheel_s, 右为 +wheel_s（对于正角度）

    // 目标线速度（每个轮子）
    float ang_rad_s = angular_speed_deg_s * (M_PI / 180.0f);
    float wheel_speed = ang_rad_s * half_track;

    Motor_t* lefts[2] = { r->fl, r->rl };
    Motor_t* rights[2] = { r->fr, r->rr };

    // 记录起始计数
    int64_t start_l[2], start_r[2];
    for (int i = 0; i < 2; ++i) { start_l[i] = lefts[i]->total_counts; start_r[i] = rights[i]->total_counts; }

    /* target counts not used; removed to avoid unused-variable warnings */

    const uint32_t control_ms = 10;
    const float control_s = control_ms / 1000.0f;
    uint32_t t0 = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)((fabsf(angle_deg) / fmaxf(fabsf(angular_speed_deg_s), 1.0f)) * 1000.0f) + 2000U;

    while (1) {
        uint32_t now = HAL_GetTick();
        // 更新编码器
        for (int i = 0; i < 2; ++i) { Motor_UpdateEncoder(lefts[i]); Motor_UpdateEncoder(rights[i]); }

        // 判断是否到位（按平均进度）
        float sum_done = 0.0f;
        for (int i = 0; i < 2; ++i) {
            int64_t dl = lefts[i]->total_counts - start_l[i];
            int64_t dr = rights[i]->total_counts - start_r[i];
            float counts_per_rev_l = (float)lefts[i]->encoder_ppr * lefts[i]->gear_ratio * MG310_QUADRATURE_MULT;
            float counts_per_rev_r = (float)rights[i]->encoder_ppr * rights[i]->gear_ratio * MG310_QUADRATURE_MULT;
            float travelled_l = (float)dl / counts_per_rev_l * (M_PI * lefts[i]->wheel_diameter_m);
            float travelled_r = (float)dr / counts_per_rev_r * (M_PI * rights[i]->wheel_diameter_m);
            sum_done += (-travelled_l + travelled_r) * 0.5f; // normalized toward target wheel_s
        }
        float avg_done = sum_done / 2.0f;
        if (angle_deg >= 0) {
            if (avg_done >= wheel_s - 0.002f) break;
        } else {
            if (avg_done <= wheel_s + 0.002f) break;
        }

        if ((now - t0) > timeout_ms) break;

        /* 位置优先：外环按剩余角度（转换为单侧弧长）生成速度参考 */
        {
            float pos_kp = MOTOR_POS_KP;
            float min_speed = MOTOR_POS_MIN_SPEED_M_S;
            float tol = MOTOR_POS_TOLERANCE_M;
            float default_max_speed = (wheel_speed > 0.0f) ? fabsf(wheel_speed) : 5.0f;
            float max_speed = default_max_speed;

            /* 带符号的剩余弧长（m） */
            float remaining = wheel_s - avg_done;
            float abs_rem = fabsf(remaining);
            float desired_speed = fminf(max_speed, pos_kp * abs_rem);
            if (abs_rem > tol && desired_speed < min_speed) desired_speed = min_speed;
            int angle_sign = (angle_deg >= 0.0f) ? 1 : -1;

            for (int i = 0; i < 2; ++i) {
                float target_l = -((float)angle_sign) * desired_speed;
                float target_r = ((float)angle_sign) * desired_speed;
                float pid_l = Motor_PIDCompute(lefts[i], target_l, control_s);
                float pid_r = Motor_PIDCompute(rights[i], target_r, control_s);
                uint16_t pwm_l = (uint16_t)fminf(fabsf(pid_l), 100.0f);
                uint16_t pwm_r = (uint16_t)fminf(fabsf(pid_r), 100.0f);
                uint8_t dir_l = (pid_l >= 0.0f) ? 0 : 1;
                uint8_t dir_r = (pid_r >= 0.0f) ? 0 : 1;
                Motor_StartWheel(lefts[i], pwm_l, dir_l);
                Motor_StartWheel(rights[i], pwm_r, dir_r);
            }
        }

        HAL_Delay(control_ms);
    }

    // 停止
    for (int i = 0; i < 2; ++i) { Motor_StopWheel(lefts[i]); Motor_StopWheel(rights[i]); }
    return 0;
}

/**
 * @brief 阻塞式按格子数量移动（每格默认 300mm，中间线宽默认 7mm）
 */
int Robot_MoveGridCellsBlocking(RobotDrive_t* r, int cells, float speed_m_s) {
    if (!r) return -1;
    int sign = (cells >= 0) ? 1 : -1;
    int n = (cells >= 0) ? cells : -cells;
    if (n == 0) return 0;
    // 计算距离：n 个格子中心间距 = n * cell_size + (n - 1) * line_width
    float cell = GRID_DEFAULT_CELL_M;
    float linew = GRID_DEFAULT_LINE_WIDTH_M;
    float distance = (float)n * cell + fmaxf(0.0f, (float)(n - 1)) * linew;
    distance *= sign;
    return Robot_MoveDistanceBlocking(r, distance, speed_m_s);
}

/**
 * @brief 旋转单个轮子指定圈数（阻塞）
 */
int Motor_RotateRevolutionsBlocking(Motor_t* m, float revolutions, float speed_m_s) {
    if (!m) return -1;
    float circumference = M_PI * m->wheel_diameter_m;
    float distance = revolutions * circumference;
    // 使用单轮的距离控制：将目标距离转换为 counts 并使用 PID 控制单轮
    // 我们可以复用 Motor_WheelInit / Robot_MoveDistanceBlocking 的思路，但这里只对单轮实现
    int64_t start = m->total_counts;
    int64_t target = start + Motor_DistanceToCounts(m, distance);

    const uint32_t control_ms = 10;
    const float control_s = control_ms / 1000.0f;
    uint32_t t0 = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)((fabsf(distance) / fmaxf(fabsf(speed_m_s), 0.01f)) * 1000.0f) + 2000U;

    while (1) {
        Motor_UpdateEncoder(m);
        int64_t cur = m->total_counts;
        if (distance >= 0) {
            if (cur - start >= target - start - 1) break;
        } else {
            if (cur - start <= target - start + 1) break;
        }//判断是否达到目标（允许1个计数的误差）

        if ((HAL_GetTick() - t0) > timeout_ms) break;//超时退出

        /* 位置优先：按剩余距离生成速度参考，再走速度 PID */
        {
            float circumference = M_PI * m->wheel_diameter_m;
            float counts_per_rev = (float)m->encoder_ppr * m->gear_ratio * MG310_QUADRATURE_MULT;
            float travelled_revs = ((float)(cur - start)) / counts_per_rev;
            float travelled_m = travelled_revs * circumference;
            float remaining = distance - travelled_m; /* 带符号 */
            float abs_rem = fabsf(remaining);
            float max_speed = (speed_m_s > 0.0f) ? fabsf(speed_m_s) : 5.0f;
            float desired_speed = fminf(max_speed, MOTOR_POS_KP * abs_rem);
            if (abs_rem > MOTOR_POS_TOLERANCE_M && desired_speed < MOTOR_POS_MIN_SPEED_M_S) desired_speed = MOTOR_POS_MIN_SPEED_M_S;
            int move_sign = (remaining >= 0.0f) ? 1 : -1;

            float pid = Motor_PIDCompute(m, desired_speed * (float)move_sign, control_s);
            uint16_t pwm = (uint16_t)fminf(fabsf(pid), 100.0f);
            uint8_t dir = (pid >= 0.0f) ? 0 : 1;
            Motor_StartWheel(m, pwm, dir);
        }
        HAL_Delay(control_ms);
    }

    Motor_StopWheel(m);
    return 0;
}

/**
 * @brief 使整个底盘按轮子圈数移动（每个轮子转相同圈数，阻塞）
 */
int Robot_RotateWheelsRevolutionsBlocking(RobotDrive_t* r, float revolutions, float speed_m_s) {
    if (!r) return -1;
    float distance = revolutions * M_PI * r->fl->wheel_diameter_m;
    return Robot_MoveDistanceBlocking(r, distance, speed_m_s);
}

/**
 * @brief 根据当前 PID 状态预测控制输出对应的 PWM 百分比（绝对值），不修改内部状态
 */
float Motor_PredictPWMPercent(Motor_t* m, float target_speed_m_s, float dt_s) {
    if (!m) return 0.0f;
    if (dt_s <= 0.0f) dt_s = 0.02f; /* 默认 20 ms */
    float error = target_speed_m_s - m->speed_m_s;
    float integrator = m->pid.integrator + error * dt_s;
    if (integrator > m->pid.integrator_limit) integrator = m->pid.integrator_limit;
    if (integrator < -m->pid.integrator_limit) integrator = -m->pid.integrator_limit;
    float derivative = (error - m->pid.prev_error) / dt_s;
    float output = m->pid.kp * error + m->pid.ki * integrator + m->pid.kd * derivative;
    if (output > m->pid.output_limit) output = m->pid.output_limit;
    if (output < -m->pid.output_limit) output = -m->pid.output_limit;
    return fabsf(output);
}

/**
 * @brief 判断预测 PWM 是否低于阈值（用于在调用前给出提醒）
 */
bool Motor_IsPredictedPWMBelow(Motor_t* m, float target_speed_m_s, float dt_s, uint16_t threshold_percent) {
    float predicted = Motor_PredictPWMPercent(m, target_speed_m_s, dt_s);
    return (predicted < (float)threshold_percent);
}

/**
 * @brief 设置起动助力参数（运行时修改）
 * @param[in] threshold_percent 触发助力的请求占空比阈值（0..100）
 * @param[in] boost_percent 助力期间使用的占空比（0..100）
 * @param[in] boost_ms 助力持续时间（毫秒）
 */
void Motor_SetStartBoostParams(uint16_t threshold_percent, uint16_t boost_percent, uint32_t boost_ms) {
    if (threshold_percent > 100) threshold_percent = 100;
    if (boost_percent > 100) boost_percent = 100;
    g_start_threshold_percent = threshold_percent;
    g_start_boost_percent = boost_percent;
    g_start_boost_ms = boost_ms;
}

/**
 * @brief 获取当前起动助力参数
 */
void Motor_GetStartBoostParams(uint16_t* threshold_percent, uint16_t* boost_percent, uint32_t* boost_ms) {
    if (threshold_percent) *threshold_percent = g_start_threshold_percent;
    if (boost_percent) *boost_percent = g_start_boost_percent;
    if (boost_ms) *boost_ms = g_start_boost_ms;
}