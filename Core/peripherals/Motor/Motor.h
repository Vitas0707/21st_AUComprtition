#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file Motor.h
 * @brief 电机驱动模块接口（PWM、方向、编码器与 PID 框架）
 *
 * 功能：
 *  - 低级 PWM 与方向控制
 *  - 轮子封装：编码器测速、里程计算
 *  - PID 速度闭环框架
 *
 * 使用示例见下：
 * @code
 * Motor_t wheel;
 * Motor_WheelInit(&wheel,
 *                 M1_EN_TIM, M1_EN_CHANNEL,
 *                 GPIOB, GPIO_PIN_0, GPIO_PIN_1,
 *                 M1_Encoder_TIM,
 *                 0.06f, 500, 1.0f);
 * Motor_PIDSetParams(&wheel, 1.0f, 0.1f, 0.01f, 1000.0f, 100.0f);
 * // 主循环或定时器：
 * Motor_UpdateEncoder(&wheel);
 * float dt = 0.02f;
 * float ctrl = Motor_PIDCompute(&wheel, target_speed_m_s, dt);
 * if (ctrl >= 0) Motor_StartWheel(&wheel, (uint16_t)ctrl, 0);
 * else           Motor_StartWheel(&wheel, (uint16_t)(-ctrl), 1);
 * @endcode
 */

/** 电机定时器与通道（在 CubeMX/其他文件中定义对应的 htimX） */
#define M1_EN_TIM (&htim3)
#define M1_EN_CHANNEL TIM_CHANNEL_1
#define M1_Encoder_TIM (&htim5)

#define M2_EN_TIM (&htim3)
#define M2_EN_CHANNEL TIM_CHANNEL_2
#define M2_Encoder_TIM (&htim2)

#define M3_EN_TIM (&htim4)
#define M3_EN_CHANNEL TIM_CHANNEL_1
#define M3_Encoder_TIM (&htim1)

#define M4_EN_TIM (&htim4)
#define M4_EN_CHANNEL TIM_CHANNEL_2
#define M4_Encoder_TIM (&htim8)

/**
 * @brief 电机轮子封装结构
 */
typedef struct {
	TIM_HandleTypeDef* htim_pwm;   /**< PWM 输出使用的定时器句柄 */
	uint32_t pwm_channel;          /**< PWM 通道 */
	GPIO_TypeDef* in_port;         /**< IN1/IN2 所在的 GPIO 端口 */
	uint16_t in_pin1;              /**< IN1 引脚 */
	uint16_t in_pin2;              /**< IN2 引脚 */
	TIM_HandleTypeDef* htim_encoder; /**< 编码器使用的定时器句柄（Encoder 模式） */
	uint32_t encoder_ppr;          /**< 编码器每转脉冲数（机械 PPR），代码按 4x 计算 */
	float wheel_diameter_m;        /**< 轮子直径（米） */
	float gear_ratio;              /**< 齿比：motor_rotations / wheel_rotations */
	int32_t last_encoder_count;    /**< 上次原始计数值（计数器寄存器） */
	int64_t total_counts;          /**< 累计的脉冲数（带符号）用于里程计算 */
	float speed_m_s;               /**< 最近一次计算得到的速度（米/秒） */
	float rpm;                     /**< 最近一次计算得到的电机轴 RPM */
	uint32_t last_tick_ms;         /**< 上次更新时间（毫秒） */
    uint16_t last_pwm_percent;     /**< 上次请求的 PWM 占空比（0..100） */
    uint32_t boost_end_ms;         /**< 助力截止时间（HAL_GetTick 毫秒），0 表示未处于助力期 */
	bool invert_direction;         /**< 若为 true，则对传入的方向取反（用于安装方向差异） */
	struct {
		float kp;                  /**< PID 比例系数 */
		float ki;                  /**< PID 积分系数 */
		float kd;                  /**< PID 微分系数 */
		float integrator;          /**< 积分器累计值 */
		float prev_error;          /**< 上次误差，用于计算微分项 */
		float integrator_limit;    /**< 积分器限制（绝对值） */
		float output_limit;        /**< 输出限制（百分比等量级） */
	} pid; /**< PID 状态 */
} Motor_t;

/** MG310 和麦轮 默认参数 */
#define MG310_ENCODER_PPR 13U//线数
#define MG310_GEAR_RATIO 20.0f//减速比
#define MG310_WHEEL_DIAMETER_M 0.048f//麦轮直径
#define MG310_QUADRATURE_MULT 4.0f//倍频，实际每转脉冲数 = 线数 * 减速比 * 倍频

/* 起动助力配置（可按需调整） */
#define MOTOR_START_THRESHOLD_PERCENT 20U  /**< 若请求占空比低于此值且电机先前为停止，则触发短时助力 */
#define MOTOR_START_BOOST_PERCENT 60U      /**< 助力期间使用的占空比（短时） */
#define MOTOR_START_BOOST_MS 80U           /**< 助力持续时长（毫秒） */
#define MIN_PWM_THRESHOLD 50.0f 			/**< 最小 PWM 阈值，低于此值可能无法克服静摩擦 */

/** 底盘默认参数 */
#define ROBOT_TRACK_WIDTH_M 0.166f//左右轮中心距离
#define ROBOT_WHEEL_BASE_M 0.125f//前后轮中心距离

/** 网格默认参数（可在调用时覆盖） */
#define GRID_DEFAULT_CELL_M 0.300f//格子长度
#define GRID_DEFAULT_LINE_WIDTH_M 0.007f//线条宽度
#define GRID_DEFAULT_TOLERANCE_M 0.010f//容差

/* 位置优先控制参数（外环）
 * - 用于把“到位”作为第一目标，速度为第二目标。
 * - 这些默认值可通过 Motor_SetStartBoostParams()/Motor_GetStartBoostParams
 *   或将来添加的 setter/getter 调整。
 */
#define MOTOR_POS_KP 2.0f               /**< 位置到速度的比例系数（m -> m/s） */
#define MOTOR_POS_MIN_SPEED_M_S 2.0f   /**< 外环允许的最小速度（m/s），避免低速无法起动 */
#define MOTOR_POS_TOLERANCE_M 0.002f    /**< 位置收敛容差（米） */

/**
 * @brief 初始化一个基于 MG310 的轮子（便捷封装）
 * @param[in,out] m 目标 Motor_t
 * @param[in] htim_pwm PWM 定时器句柄
 * @param[in] pwm_channel PWM 通道
 * @param[in] in_port 方向引脚所在 GPIO 端口
 * @param[in] in_pin1 IN1 引脚
 * @param[in] in_pin2 IN2 引脚
 * @param[in] htim_encoder 编码器 TIM（Encoder 模式）
 */
void Motor_InitMG310Wheel(Motor_t* m,
						  TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
						  GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
						  TIM_HandleTypeDef* htim_encoder);

/**
 * @brief 旋转单个轮子指定圈数（阻塞）
 * @param[in,out] m 目标 Motor_t
 * @param[in] revolutions 要旋转的圈数（正为正向）
 * @param[in] speed_m_s 目标线速度（米/秒），用于 PID 参考
 * @return 0 成功，非0 失败
 */
int Motor_RotateRevolutionsBlocking(Motor_t* m, float revolutions, float speed_m_s);

/**
 * @brief 机器人底盘封装（四轮）
 */
typedef struct {
	Motor_t* fl; /**< 前左 */
	Motor_t* fr; /**< 前右 */
	Motor_t* rl; /**< 后左 */
	Motor_t* rr; /**< 后右 */
	float track_width_m; /**< 左右轮中心间距（m），用于自转角度换算 */
	float wheel_base_m;  /**< 前后轮中心间距（m），预留 */
} RobotDrive_t;

/**
 * @brief 使整个底盘按轮子圈数移动（每个轮子转相同圈数，阻塞）
 * @param[in] r 底盘
 * @param[in] revolutions 每个轮子要转的圈数（正为前进）
 * @param[in] speed_m_s 目标线速度（米/秒）
 * @return 0 成功，非0 失败
 */
int Robot_RotateWheelsRevolutionsBlocking(RobotDrive_t* r, float revolutions, float speed_m_s);

/**
 * @brief 初始化底盘结构
 */
void RobotDrive_Init(RobotDrive_t* r, Motor_t* fl, Motor_t* fr, Motor_t* rl, Motor_t* rr,
					 float track_width_m, float wheel_base_m);

/**
 * @brief 阻塞式移动：按指定距离前进/后退（米）
 * @param[in] r 底盘
 * @param[in] distance_m 正值前进，负值后退
 * @param[in] speed_m_s 期望线速度（米/秒），用于 PID 参考值
 * @return 0 成功，非0 失败
 */
int Robot_MoveDistanceBlocking(RobotDrive_t* r, float distance_m, float speed_m_s);

/**
 * @brief 阻塞式自转到指定角度（相对当前朝向，单位度）
 * @param[in] r 底盘
 * @param[in] angle_deg 目标相对转角，正数为逆时针（左转）
 * @param[in] angular_speed_deg_s 目标角速度（度/秒），用于 PID 参考值
 * @return 0 成功，非0 失败
 */
int Robot_TurnAngleBlocking(RobotDrive_t* r, float angle_deg, float angular_speed_deg_s);

/**
 * @brief 阻塞式按格子数量移动（每格默认 300mm，可使用 GRID_DEFAULT_* 覆盖）
 * @param[in] r 底盘
 * @param[in] cells 要移动的格子数量（正为前进，负为后退）
 * @param[in] speed_m_s 期望线速度（米/秒）
 * @return 0 成功，非0 失败
 */
int Robot_MoveGridCellsBlocking(RobotDrive_t* r, int cells, float speed_m_s);


/**
 * @brief 启动 TIM Base 与对应 PWM 通道
 * @param[in] htim 要启动的 TIM 句柄
 * @param[in] Channel PWM 通道（例如 TIM_CHANNEL_1）
 * @note 在使用 PWM 输出前调用一次，或由 Motor_WheelInit 内部调用
 */
void Motor_Init(TIM_HandleTypeDef* htim, uint32_t Channel);

/**
 * @brief 设置 PWM 占空比（百分比）
 * @param[in] htim PWM 定时器句柄
 * @param[in] Channel PWM 通道
 * @param[in] pwm_percent 占空比（0..100）
 * @note 占空比按定时器的 ARR 映射到 CCR
 */
void Motor_SetPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t pwm_percent);

/**
 * @brief 设置电机方向
 * @param[in] GPIOx IN1/IN2 所在 GPIO 端口
 * @param[in] GPIO_Pin1 IN1 引脚
 * @param[in] GPIO_Pin2 IN2 引脚
 * @param[in] direction 方向标志（0: IN1=高、IN2=低; 非0: IN1=低、IN2=高）
 */
void Motor_SetDirection(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin1,uint16_t GPIO_Pin2,uint8_t direction);

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
 */
void Motor_WheelInit(Motor_t* m,
					 TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
					 GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
					 TIM_HandleTypeDef* htim_encoder,
					 float wheel_diameter_m, uint32_t encoder_ppr, float gear_ratio);
/**
 * @note 初始化时会将 m->last_tick_ms 置为 0；首次调用 @ref Motor_UpdateEncoder
 *       会采样编码器并建立时间基准，避免初始化后长时间未启动造成的 dt 爆发。
 */

/**
 * @brief 设置方向并将 PWM 设置为指定占空比，启动电机
 *
 * 行为说明：
 * - 为了克服静摩擦与起动扭矩，库实现了“起动助力”策略：当请求的 `pwm_percent`
 *   大于 0 且小于 `MOTOR_START_THRESHOLD_PERCENT` 且电机先前处于停止（上次 PWM==0），
 *   驱动会在短时间内（`MOTOR_START_BOOST_MS` 毫秒）将占空比提升到
 *   `MOTOR_START_BOOST_PERCENT` 以保证能起动，然后再回落到请求值。
 * - 助力期间由 `m->boost_end_ms` 跟踪（内部管理），用户无需手动清理。
 * - 若你想在调用前检测某个目标速度是否会产生低于指定阈值（例如 50%）的 PWM，
 *   可使用 `Motor_PredictPWMPercent()` 或 `Motor_IsPredictedPWMBelow()` 来做提醒。
 *
 * @param[in,out] m 目标 Motor_t
 * @param[in] pwm_percent 占空比 0..100（请求值）
 * @param[in] direction 方向标志（0 或 1，见 Motor_SetDirection）
 */
void Motor_StartWheel(Motor_t* m, uint16_t pwm_percent, uint8_t direction);

/**
 * @brief 根据当前 PID 状态与速度预测控制输出对应的 PWM 百分比（绝对值）
 * @param[in] m 目标 Motor_t
 * @param[in] target_speed_m_s 目标速度（m/s）
 * @param[in] dt_s 估计控制周期（秒），传入 <=0 时使用 0.02s 作为默认值
 * @return 预测的 PWM 百分比（0..output_limit），不改变 `m` 内部 PID 状态
 */
float Motor_PredictPWMPercent(Motor_t* m, float target_speed_m_s, float dt_s);

/**
 * @brief 检查预测得到的 PWM 是否低于指定阈值（用于在调用前提醒）
 * @param[in] m 目标 Motor_t
 * @param[in] target_speed_m_s 目标速度（m/s）
 * @param[in] dt_s 估计控制周期（秒），传入 <=0 时使用 0.02s 作为默认值
 * @param[in] threshold_percent 要比较的阈值（例如 50）
 * @return true 如果预测 PWM 百分比 < threshold_percent
 */
bool Motor_IsPredictedPWMBelow(Motor_t* m, float target_speed_m_s, float dt_s, uint16_t threshold_percent);

/**
 * @brief 运行时设置起动助力参数
 * @param[in] threshold_percent 触发助力的请求占空比阈值（0..100）
 * @param[in] boost_percent 助力期间使用的占空比（0..100）
 * @param[in] boost_ms 助力持续时间（毫秒）
 */
void Motor_SetStartBoostParams(uint16_t threshold_percent, uint16_t boost_percent, uint32_t boost_ms);

/**
 * @brief 获取当前起动助力参数
 */
void Motor_GetStartBoostParams(uint16_t* threshold_percent, uint16_t* boost_percent, uint32_t* boost_ms);

/**
 * @brief 停止电机并将方向引脚拉低
 * @param[in,out] m 目标 Motor_t
 */
void Motor_StopWheel(Motor_t* m);

/**
 * @brief 运行时设置单轮方向是否反向
 * @param[in] m 目标 Motor_t
 * @param[in] inverted true 则将传入的方向位反转
 */
void Motor_SetDirectionInverted(Motor_t* m, bool inverted);

/**
 * @brief 更新编码器计数并计算速度与里程
 * @param[in,out] m 目标 Motor_t
 * @note 必须周期性调用（建议 10~100 ms）以获得稳定的速度/里程计算
 */
void Motor_UpdateEncoder(Motor_t* m);

/**
 * @brief 获取速度（米/秒）
 * @param[in] m 目标 Motor_t
 * @return 最近一次计算得到的速度（米/秒）
 */
float Motor_GetSpeedMps(Motor_t* m);

/**
 * @brief 获取累计里程（米）
 * @param[in] m 目标 Motor_t
 * @return 累计里程（米）
 */
float Motor_GetDistanceM(Motor_t* m);

/**
 * @brief 重置累计里程计数
 * @param[in,out] m 目标 Motor_t
 */
void Motor_ResetDistance(Motor_t* m);

/**
 * @brief 设置 PID 参数并重置内部状态
 * @param[in,out] m 目标 Motor_t
 * @param[in] kp 比例系数
 * @param[in] ki 积分系数
 * @param[in] kd 微分系数
 * @param[in] integrator_limit 积分器饱和限制（绝对值）
 * @param[in] output_limit 输出限制（例如 pwm 百分比最大值）
 */
void Motor_PIDSetParams(Motor_t* m, float kp, float ki, float kd,
						float integrator_limit, float output_limit);

/**
 * @brief 基于当前速度计算 PID 输出
 * @param[in,out] m 目标 Motor_t
 * @param[in] target_speed_m_s 目标速度（米/秒）
 * @param[in] dt_s 控制周期（秒）。如果传入 <=0，函数内部会使用 `HAL_GetTick()` 和 `m->last_tick_ms` 估算
 * @note 函数会对估算出的 dt 做上下限保护（防止异常大 dt 导致积分瞬间增大）。
 * @return 带符号控制量（-output_limit .. +output_limit），符号表示方向
 */
float Motor_PIDCompute(Motor_t* m, float target_speed_m_s, float dt_s);

/**
 * @brief 重置 PID 内部状态（积分器和前次误差）
 * @param[in,out] m 目标 Motor_t
 */
void Motor_PIDReset(Motor_t* m);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_H
