#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
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

/** MG310 默认参数 */
#define MG310_ENCODER_PPR 13U
#define MG310_GEAR_RATIO 20.0f
#define MG310_WHEEL_DIAMETER_M 0.048f
#define MG310_QUADRATURE_MULT 4.0f

/** 网格默认参数（可在调用时覆盖） */
#define GRID_DEFAULT_CELL_M 0.300f
#define GRID_DEFAULT_LINE_WIDTH_M 0.007f
#define GRID_DEFAULT_TOLERANCE_M 0.010f

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
 * @brief 使整个底盘按轮子圈数移动（每个轮子转相同圈数，阻塞）
 * @param[in] r 底盘
 * @param[in] revolutions 每个轮子要转的圈数（正为前进）
 * @param[in] speed_m_s 目标线速度（米/秒）
 * @return 0 成功，非0 失败
 */
int Robot_RotateWheelsRevolutionsBlocking(RobotDrive_t* r, float revolutions, float speed_m_s);

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
 * @brief 设置方向并将 PWM 设置为指定占空比，启动电机
 * @param[in,out] m 目标 Motor_t
 * @param[in] pwm_percent 占空比 0..100
 * @param[in] direction 方向标志（0 或 1，见 Motor_SetDirection）
 */
void Motor_StartWheel(Motor_t* m, uint16_t pwm_percent, uint8_t direction);

/**
 * @brief 停止电机并将方向引脚拉低
 * @param[in,out] m 目标 Motor_t
 */
void Motor_StopWheel(Motor_t* m);

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
