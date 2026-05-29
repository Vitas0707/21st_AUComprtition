#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_PPR 13
#define ENCODER_GEAR_RATIO 20
#define FREQUENCY_MULT 4
#define ENCODER_CPR (ENCODER_PPR * ENCODER_GEAR_RATIO * FREQUENCY_MULT)
#define WHEEL_DIAMETER_M 0.048f
#define TRACK_WIDTH_M 0.165f


/* 电机控制通道和对应编码器通道 */
#define FL_TIM_HANDLE htim4
#define FL_TIM_CHANNEL TIM_CHANNEL_3
#define FL_IN_PORT GPIOB
#define FL_IN_PIN1 GPIO_PIN_0
#define FL_IN_PIN2 GPIO_PIN_1
#define FL_ENCODER_TIM htim5

#define FR_TIM_HANDLE htim4
#define FR_TIM_CHANNEL TIM_CHANNEL_4
#define FR_IN_PORT GPIOC
#define FR_IN_PIN1 GPIO_PIN_0
#define FR_IN_PIN2 GPIO_PIN_1
#define FR_ENCODER_TIM htim3

#define RL_TIM_HANDLE htim4
#define RL_TIM_CHANNEL TIM_CHANNEL_1
#define RL_IN_PORT GPIOD
#define RL_IN_PIN1 GPIO_PIN_14
#define RL_IN_PIN2 GPIO_PIN_15
#define RL_ENCODER_TIM htim1

#define RR_TIM_HANDLE htim4
#define RR_TIM_CHANNEL TIM_CHANNEL_2
#define RR_IN_PORT GPIOD
#define RR_IN_PIN1 GPIO_PIN_10
#define RR_IN_PIN2 GPIO_PIN_11
#define RR_ENCODER_TIM htim8


#define MIN_PWM_THRESHOLD 50U
#define MAX_PWM_THRESHOLD 65U
#define FEED_FORWARD_PWM 50U
#define DEFAULT_KP 30.0f
#define DEFAULT_KI 1.0f
#define DEFAULT_KD 0.0f
#define DEFAULT_ERROR_LPF_ALPHA 0.1f
#define PID_OUTPUT_LIMIT 100.0f
#define PID_CONTROL_PERIOD_MS 10U
#define POS_KP 0.5f
#define POS_KD 0.01f

typedef struct {
    TIM_HandleTypeDef* htim_pwm;
    uint32_t pwm_channel;
    GPIO_TypeDef* in_port;
    uint16_t in_pin1;
    uint16_t in_pin2;
    TIM_HandleTypeDef* htim_encoder;
    uint32_t encoder_ppr;
    float wheel_diameter_m;
    float gear_ratio;
    int32_t last_encoder_count;
    int64_t total_counts;
    float speed_m_s;
    float rpm;
    uint32_t last_tick_ms;       /* 编码器更新时间戳（ms） */
    uint32_t pid_last_tick_ms;   /* 上次 PID 控制时间戳（ms） */
    bool invert_direction;
    struct {
        float kp;
        float ki;
        float kd;
        float prev_error;
        float prev_prev_error;
        float integral;
        float output;
        float output_limit;
        float lpf_alpha;
    } pid;
} Motor_t;

typedef struct {
    Motor_t* fl;
    Motor_t* fr;
    Motor_t* rl;
    Motor_t* rr;
    float track_width_m;
    float pos_prev_error;
    float lpf_alpha;
} Car_t;


typedef enum {
    CAR_DIR_FORWARD = 0,
    CAR_DIR_BACKWARD,
    CAR_DIR_LEFT,
    CAR_DIR_RIGHT,
    CAR_DIR_CW,//顺时针
    CAR_DIR_CCW,//逆时针
} CarDir_t;




void Motor_WheelInit(Motor_t* m,
                     TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
                     GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
                     TIM_HandleTypeDef* htim_encoder,
                     float wheel_diameter_m, uint32_t encoder_ppr, float gear_ratio);

void Car_Init(Car_t* car, Motor_t* fl, Motor_t* fr, Motor_t* rl, Motor_t* rr, float track_width_m);





void Motor_StartWheel(Motor_t* m, uint16_t pwm_percent, uint8_t direction);
void Motor_StopWheel(Motor_t* m);
void Motor_SetDirectionInverted(Motor_t* m, bool inverted);

void Motor_UpdateEncoder(Motor_t* m);
void Motor_PIDInit(Motor_t* m, float kp, float ki, float kd, float output_limit) ;
void Motor_PIDSetParams(Motor_t* m, float kp, float ki, float kd);
float Motor_PIDIncrementalUpdate(Motor_t* m, float target_speed_m_s,float dt_s);

void Motor_ControlWheel(Motor_t* m, float target_speed_m_s) ;

int Car_DriveDistance(Car_t* car, CarDir_t dir, float distance_m);
int Car_StrafeDistance(Car_t* car, CarDir_t dir, float distance_m, float speed_m_s);
int Car_RotateAngle(Car_t* car, CarDir_t dir, float angle_deg) ;


#ifdef __cplusplus
}
#endif

#endif // __MOTOR_H
