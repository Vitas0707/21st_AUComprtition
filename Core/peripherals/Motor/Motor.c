#include "Motor.h"
#include "main.h"
#include <math.h>
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "Bluetooth.h"
#include "host.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h> 

#ifndef M_PI
#define M_PI 3.1415926f
#endif

/* internal functions:set pwm and direction */
static void Motor_SetPWM_Internal(TIM_HandleTypeDef* htim, uint32_t Channel, float pwm_percent) {
    if (htim == NULL) return;
    if (pwm_percent > 100.0f) pwm_percent = 100.0f;
    uint32_t arr = (uint32_t)(htim->Instance->ARR);
    uint32_t ccr = (uint32_t)(((pwm_percent * (arr + 1U)) / 100U)+0.5f);
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}

static void Motor_SetDirection_Internal(GPIO_TypeDef* GPIOx, uint16_t Pin1, uint16_t Pin2, uint8_t direction) {
    if (direction == 0) {
        HAL_GPIO_WritePin(GPIOx, Pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOx, Pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOx, Pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOx, Pin2, GPIO_PIN_SET);
    }
}

/* init motor(including reverse direction), init car,set pid params */
void Motor_WheelInit(Motor_t* m,TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,GPIO_TypeDef* in_port, uint16_t in_pin1, 
    uint16_t in_pin2,TIM_HandleTypeDef* htim_encoder,float wheel_diameter_m, uint32_t encoder_ppr, float gear_ratio) {
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
    m->pid_last_tick_ms = 0;
    m->invert_direction = false;
    m->pid.kp = DEFAULT_KP; m->pid.ki = DEFAULT_KI; m->pid.kd = DEFAULT_KD;
    m->pid.prev_error = 0.0f; m->pid.prev_prev_error = 0.0f; m->pid.output = 0.0f;
    m->pid.integral = 0.0f;
    m->pid.output_limit = PID_OUTPUT_LIMIT;
    m->pid.lpf_alpha = DEFAULT_ERROR_LPF_ALPHA;

    if (htim_pwm) {
        HAL_TIM_Base_Start(htim_pwm);
        HAL_TIM_PWM_Init(htim_pwm);
        HAL_TIM_PWM_Start(htim_pwm, pwm_channel);
        Motor_SetPWM_Internal(htim_pwm, pwm_channel, 0);
    }
    if (htim_encoder) {
        HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL);
        m->last_encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(htim_encoder);
    }
}

void Motor_SetDirectionInverted(Motor_t* m, bool inverted) {
    if (!m) return;
    m->invert_direction = inverted ? true : false;
}

void Car_Init(Car_t* car, Motor_t* fl, Motor_t* fr, Motor_t* rl, Motor_t* rr, float track_width_m) {
    if (!car) return;
    car->fl = fl; car->fr = fr; car->rl = rl; car->rr = rr; car->track_width_m  = track_width_m;car->lpf_alpha = DEFAULT_ERROR_LPF_ALPHA;
    car->pos_prev_error = 0.0f;
    car->pos_integral = 0.0f;
}

void Motor_PIDInit(Motor_t* m, float kp, float ki, float kd, float output_limit) {
    if (!m) return;
    m->pid.kp = kp;
    m->pid.ki = ki;
    m->pid.kd = kd;
    m->pid.prev_error = 0.0f;
    m->pid.prev_prev_error = 0.0f;
    m->pid.output = 0.0f;
    m->pid.integral = 0.0f;
    if (output_limit > 0.0f) m->pid.output_limit = output_limit;
}

void Motor_PIDSetParams(Motor_t* m, float kp, float ki, float kd) {
    if (!m) return;
    m->pid.kp = kp;
    m->pid.ki = ki;
    m->pid.kd = kd;
}

/* basic application functions:start and stop wheels */
void Motor_StartWheel(Motor_t* m, float pwm_percent, uint8_t direction) {
    if (!m) return;
    if (pwm_percent > 100) pwm_percent = 100;
    uint8_t actual_dir = (uint8_t)(direction ^ (m->invert_direction ? 1U : 0U));
    Motor_SetDirection_Internal(m->in_port, m->in_pin1, m->in_pin2, actual_dir);
    Motor_SetPWM_Internal(m->htim_pwm, m->pwm_channel, pwm_percent);
}

void Motor_StopWheel(Motor_t* m) {
    if (!m) return;
    Motor_SetPWM_Internal(m->htim_pwm, m->pwm_channel, 0);
    HAL_GPIO_WritePin(m->in_port, m->in_pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in_port, m->in_pin2, GPIO_PIN_RESET);
    m->pid.integral = 0.0f;
    m->pid.prev_error = 0.0f;
    m->pid.prev_prev_error = 0.0f;
    m->pid.output = 0.0f;
    m->total_counts = 0;
    m->last_tick_ms = 0;
    m->last_encoder_count = 0;
    m->pid_last_tick_ms = 0;
}



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
    float cpr = (float)ENCODER_CPR;
    float revolutions = ((float)delta) / cpr;
    float revs_per_sec = revolutions / dt_s;
    m->rpm = revs_per_sec * 60.0f;
    float circumference = M_PI * m->wheel_diameter_m;
    m->speed_m_s = revs_per_sec * circumference;

    if (m->invert_direction) {
        m->speed_m_s = -m->speed_m_s;
        m->rpm = -m->rpm;
    }

    m->last_tick_ms = now;
}

/* Reset PID integral to prevent windup during position-loop approach */
void Motor_ResetPIDIntegral(Motor_t* m) {
    if (!m) return;
    m->pid.integral = 0.0f;
    m->pid.prev_error = 0.0f;
    m->pid.prev_prev_error = 0.0f;
}

/* Algorithm function: incremental PID update */
float Motor_PIDIncrementalUpdate(Motor_t* m, float target_speed_m_s, float dt_s) {
    if (!m) return 0.0f;
    if(dt_s <= 0.0f) dt_s = ((float)PID_CONTROL_PERIOD_MS) / 1000.0f;
    

    float raw_error = target_speed_m_s - m->speed_m_s;

    float alpha = m->pid.lpf_alpha;
    if (alpha < 0.0f) alpha = 0.0f;
    else if (alpha > 1.0f) alpha = 1.0f;

    float error = alpha * raw_error + (1.0f - alpha) * m->pid.prev_error;

    float derivative = (error - 2.0f * m->pid.prev_error + m->pid.prev_prev_error) / (dt_s > 1e-6f ? dt_s : 1e-3f);

    float integral = m->pid.integral + error * dt_s;

    if(integral > 200.0f) integral = 200.0f;
    if(integral < -200.0f) integral = -200.0f;

    float delta_output = m->pid.kp * error + m->pid.ki * integral + m->pid.kd * derivative;
    float out = m->pid.output + delta_output;

    if (out > m->pid.output_limit) out = m->pid.output_limit;
    if (out < -m->pid.output_limit) out = -m->pid.output_limit;

    m->pid.prev_prev_error = m->pid.prev_error;
    m->pid.prev_error = error;
    m->pid.output = out;
    m->pid.integral = integral;
    return out;
}


void Motor_ControlWheel(Motor_t* m, float target_speed_m_s) {
    if (!m) return ;

    const uint32_t period_ms = PID_CONTROL_PERIOD_MS;
    uint32_t now = HAL_GetTick();

    uint32_t experienced_ms = (m->pid_last_tick_ms == 0) ? period_ms : (now - m->pid_last_tick_ms);
    if (experienced_ms < period_ms) {
        return ;
    }

    float dt_s = ((float)experienced_ms) / 1000.0f;

    Motor_UpdateEncoder(m);
    float out = Motor_PIDIncrementalUpdate(m, target_speed_m_s, dt_s);


    float pwm = (float)fminf(fabsf(out), MAX_PWM_THRESHOLD);
    pwm = fmaxf(pwm, MIN_PWM_THRESHOLD);
    uint8_t dir = (out >= 0.0f) ? 0 : 1;

    m->pwm_duty = pwm;
    Motor_StartWheel(m, pwm, dir);
    m->pid_last_tick_ms = now;
    return ;
}


/* user application function: rotate in place */

 static float DistanceToCounts(Motor_t* m, float distance_m) {
    if (!m) return 0;
    float circumference = M_PI * m->wheel_diameter_m;
    float revolutions = distance_m / circumference;
    float counts =revolutions * ENCODER_CPR;
    return counts;
}

static void Car_StopAllWheels(Car_t* car) {
    if (!car) return;
    if (car->fl) {
        Motor_StopWheel(car->fl);
    }
    if (car->fr) {
        Motor_StopWheel(car->fr);
    }
    if (car->rl) {
        Motor_StopWheel(car->rl);
    }
    if (car->rr) {
        Motor_StopWheel(car->rr);
    }
    car->pos_prev_error = 0.0f;
    car->pos_integral = 0.0f;
    return;
}

int Car_DriveDistance(Car_t* car, CarDir_t dir, float distance_m) {
    if (!car || !car->fl || !car->fr || !car->rl || !car->rr) return -1;
    if (dir != CAR_DIR_FORWARD && dir != CAR_DIR_BACKWARD) return -1;
    if (distance_m <= 0.0f) return -1;

    float t_move_s = distance_m / 0.20f;
    uint32_t timeout_ms = (uint32_t)(t_move_s * 1000.0f * 2.0f);

    Motor_UpdateEncoder(car->fl);
    Motor_UpdateEncoder(car->fr);
    Motor_UpdateEncoder(car->rl);
    Motor_UpdateEncoder(car->rr);

    int64_t start_counts[4];
    start_counts[0] = car->fl->total_counts;
    start_counts[1] = car->fr->total_counts;
    start_counts[2] = car->rl->total_counts;
    start_counts[3] = car->rr->total_counts;

    float counts_f = DistanceToCounts(car->fl, distance_m);
    int64_t target_counts = (int64_t)roundf(fabsf(counts_f));

    uint32_t start_ms = HAL_GetTick();


    for (;;) {
        int64_t d0 = car->fl->total_counts - start_counts[0]; if (d0 < 0) d0 = -d0;
        int64_t d1 = car->fr->total_counts - start_counts[1]; if (d1 < 0) d1 = -d1;
        int64_t d2 = car->rl->total_counts - start_counts[2]; if (d2 < 0) d2 = -d2;
        int64_t d3 = car->rr->total_counts - start_counts[3]; if (d3 < 0) d3 = -d3;
        int64_t current_counts = (d0 + d1 + d2 + d3) / 4;

        float pos_raw_error = (float)(target_counts - current_counts);
        float pos_error = car->lpf_alpha * pos_raw_error + (1.0f - car->lpf_alpha) * car->pos_prev_error;

        float pos_out = POS_KP * pos_error + POS_KD * ((pos_error - car->pos_prev_error)/(float)PID_CONTROL_PERIOD_MS);
        float wheel_speed = ((dir == CAR_DIR_FORWARD) ? pos_out : -pos_out)*0.0001f ;
        car->pos_prev_error = pos_error;

        /* Reset speed-loop integral when approaching target to prevent overshoot oscillation */
        if (fabsf(pos_error) < POS_INTEGRAL_RESET_THRESHOLD) {
            Motor_ResetPIDIntegral(car->fl);
            Motor_ResetPIDIntegral(car->fr);
            Motor_ResetPIDIntegral(car->rl);
            Motor_ResetPIDIntegral(car->rr);
            car->pos_integral = 0.0f;
        }

        Motor_ControlWheel(car->fl, wheel_speed);
        Motor_ControlWheel(car->fr, wheel_speed);
        Motor_ControlWheel(car->rl, wheel_speed);
        Motor_ControlWheel(car->rr, wheel_speed);

        HAL_Delay(PID_CONTROL_PERIOD_MS);

        Motor_UpdateEncoder(car->fl);
        Motor_UpdateEncoder(car->fr);
        Motor_UpdateEncoder(car->rl);
        Motor_UpdateEncoder(car->rr);

        d0 = car->fl->total_counts - start_counts[0]; if (d0 < 0) d0 = -d0;
        d1 = car->fr->total_counts - start_counts[1]; if (d1 < 0) d1 = -d1;
        d2 = car->rl->total_counts - start_counts[2]; if (d2 < 0) d2 = -d2;
        d3 = car->rr->total_counts - start_counts[3]; if (d3 < 0) d3 = -d3;
        int64_t d_avg = (d0 + d1 + d2 + d3) / 4;
        if (d_avg >= target_counts) {
            Car_StopAllWheels(car);
            return 0;
        }

        if ((HAL_GetTick() - start_ms) > timeout_ms) {
            Car_StopAllWheels(car);
            return -2;
        }


    }
}

int Car_StrafeDistance(Car_t* car, CarDir_t dir, float distance_m, float speed_m_s) {
    if (!car || !car->fl || !car->fr || !car->rl || !car->rr) return -1;
    if (dir != CAR_DIR_LEFT && dir != CAR_DIR_RIGHT) return -1;
    if (distance_m <= 0.0f) return -1;

    /* For mecanum with 45deg rollers, wheel travel = distance * sqrt(2) */
    const float diag_factor = sqrtf(2.0f);
    float effective_distance = distance_m * diag_factor;
    float speed = (speed_m_s > 0.001f) ? speed_m_s : 0.20f;
    float wheel_speed_mag = speed * diag_factor; /* wheel linear speed */

    if (dir == CAR_DIR_LEFT) {
        wheel_speed_mag = -wheel_speed_mag;
       } 

    float t_move_s = distance_m / speed;
    uint32_t timeout_ms = (uint32_t)(t_move_s * 1000.0f * 2.0f);


    Motor_UpdateEncoder(car->fl);
    Motor_UpdateEncoder(car->fr);
    Motor_UpdateEncoder(car->rl);
    Motor_UpdateEncoder(car->rr);


    int64_t start_counts[4];
    start_counts[0] = car->fl->total_counts;
    start_counts[1] = car->fr->total_counts;
    start_counts[2] = car->rl->total_counts;
    start_counts[3] = car->rr->total_counts;

    float counts_f = DistanceToCounts(car->fl, effective_distance);
    int64_t target_counts = (int64_t)roundf(fabsf(counts_f));

    uint32_t start_ms = HAL_GetTick();

    /* Pair-average position-loop state (local variables) */
    /* Pair A = FL & RR (same direction), Pair B = FR & RL (same direction) */
    float pos_err_A_prev = 0.0f;
    float pos_err_B_prev = 0.0f;

    const float lpf = car->lpf_alpha;

    /* base speed direction per pair (sign only, magnitude from pos_out) */
    const float dir_A = (wheel_speed_mag >= 0.0f) ? 1.0f : -1.0f;
    const float dir_B = -dir_A;

    for (;;) {
        /* --- read per-wheel encoder deltas --- */
        int64_t d0 = car->fl->total_counts - start_counts[0]; if (d0 < 0) d0 = -d0;
        int64_t d1 = car->fr->total_counts - start_counts[1]; if (d1 < 0) d1 = -d1;
        int64_t d2 = car->rl->total_counts - start_counts[2]; if (d2 < 0) d2 = -d2;
        int64_t d3 = car->rr->total_counts - start_counts[3]; if (d3 < 0) d3 = -d3;

        /* --- pair-average position error --- */
        float d_avg_A = (float)(d0 + d3) * 0.5f;  /* FL & RR pair */
        float d_avg_B = (float)(d1 + d2) * 0.5f;  /* FR & RL pair */

        float pos_raw_A = (float)(target_counts - (int64_t)d_avg_A);
        float pos_raw_B = (float)(target_counts - (int64_t)d_avg_B);

        /* --- per-pair LPF --- */
        float pos_err_A = lpf * pos_raw_A + (1.0f - lpf) * pos_err_A_prev;
        float pos_err_B = lpf * pos_raw_B + (1.0f - lpf) * pos_err_B_prev;

        /* --- per-pair PD + I --- */
        float pos_out_A = POS_KP * pos_err_A + POS_KD * ((pos_err_A - pos_err_A_prev) / (float)PID_CONTROL_PERIOD_MS);
        float pos_out_B = POS_KP * pos_err_B + POS_KD * ((pos_err_B - pos_err_B_prev) / (float)PID_CONTROL_PERIOD_MS);

        pos_err_A_prev = pos_err_A;
        pos_err_B_prev = pos_err_B;

        /* --- same-direction pair shares one speed command --- */
        float speed_A = pos_out_A * dir_A;  /* FL & RR */
        float speed_B = pos_out_B * dir_B;  /* FR & RL */

        Motor_ControlWheel(car->fl, speed_A);
        Motor_ControlWheel(car->rr, speed_A);
        Motor_ControlWheel(car->fr, speed_B);
        Motor_ControlWheel(car->rl, speed_B);

        HAL_Delay(PID_CONTROL_PERIOD_MS);

        Motor_UpdateEncoder(car->fl);
        Motor_UpdateEncoder(car->fr);
        Motor_UpdateEncoder(car->rl);
        Motor_UpdateEncoder(car->rr);

        /* re-read deltas for reach check */
        d0 = car->fl->total_counts - start_counts[0]; if (d0 < 0) d0 = -d0;
        d1 = car->fr->total_counts - start_counts[1]; if (d1 < 0) d1 = -d1;
        d2 = car->rl->total_counts - start_counts[2]; if (d2 < 0) d2 = -d2;
        d3 = car->rr->total_counts - start_counts[3]; if (d3 < 0) d3 = -d3;

        d_avg_A = (float)(d0 + d3) * 0.5f;
        d_avg_B = (float)(d1 + d2) * 0.5f;



        /* both pair averages must reach target */
        if ((int64_t)d_avg_A >= target_counts && (int64_t)d_avg_B >= target_counts) {
            Car_StopAllWheels(car);
            return 0;
        }

        if ((HAL_GetTick() - start_ms) > timeout_ms) {
            Car_StopAllWheels(car);
            return -2;
        }
    }
}

int Car_RotateAngle(Car_t* car, CarDir_t dir, float angle_deg) {
    if (!car || !car->fl || !car->fr || !car->rl || !car->rr) return -1;
    if (dir != CAR_DIR_CW && dir != CAR_DIR_CCW) return -1;
    if (angle_deg <= 0.0f) return -1;

    float theta_rad = angle_deg * (M_PI / 180.0f);

    float r_m = car->track_width_m / 2.0f;

    /* left and right linear displacements */
    float displacement_m = theta_rad * r_m;

    /* target counts for left and right wheels */
    float counts_f = DistanceToCounts(car->fl, fabsf(displacement_m));
    int64_t target_counts = (int64_t)roundf(fabsf(counts_f));


    float t_move_s = fabsf(displacement_m) / 0.20f;
    uint32_t timeout_ms = (uint32_t)(t_move_s * 1000.0f * 2.0f);

    Motor_UpdateEncoder(car->fl);
    Motor_UpdateEncoder(car->fr);
    Motor_UpdateEncoder(car->rl);
    Motor_UpdateEncoder(car->rr);

    int64_t start_counts[4];
    start_counts[0] = car->fl->total_counts;
    start_counts[1] = car->fr->total_counts;
    start_counts[2] = car->rl->total_counts;
    start_counts[3] = car->rr->total_counts;

    uint32_t start_ms = HAL_GetTick();


    /* Per-wheel position-loop state (local variables, not stored in Car_t) */
    float pos_error_fl_prev = 0.0f;
    float pos_error_fr_prev = 0.0f;
    float pos_error_rl_prev = 0.0f;
    float pos_error_rr_prev = 0.0f;

    const float lpf_alpha = car->lpf_alpha;

    for (;;) {
        /* --- read per-wheel encoder deltas --- */
        int64_t dl0 = car->fl->total_counts - start_counts[0]; if (dl0 < 0) dl0 = -dl0;
        int64_t dr1 = car->fr->total_counts - start_counts[1]; if (dr1 < 0) dr1 = -dr1;
        int64_t dl2 = car->rl->total_counts - start_counts[2]; if (dl2 < 0) dl2 = -dl2;
        int64_t dr3 = car->rr->total_counts - start_counts[3]; if (dr3 < 0) dr3 = -dr3;

        /* --- per-wheel position errors (raw) --- */
        float pos_raw_fl = (float)(target_counts - dl0);
        float pos_raw_fr = (float)(target_counts - dr1);
        float pos_raw_rl = (float)(target_counts - dl2);
        float pos_raw_rr = (float)(target_counts - dr3);

        /* --- per-wheel LPF --- */
        float pos_err_fl = lpf_alpha * pos_raw_fl + (1.0f - lpf_alpha) * pos_error_fl_prev;
        float pos_err_fr = lpf_alpha * pos_raw_fr + (1.0f - lpf_alpha) * pos_error_fr_prev;
        float pos_err_rl = lpf_alpha * pos_raw_rl + (1.0f - lpf_alpha) * pos_error_rl_prev;
        float pos_err_rr = lpf_alpha * pos_raw_rr + (1.0f - lpf_alpha) * pos_error_rr_prev;

        /* --- per-wheel PD + I = pos_out --- */
        float pos_out_fl = POS_KP * pos_err_fl + POS_KD * ((pos_err_fl - pos_error_fl_prev) / (float)PID_CONTROL_PERIOD_MS);
        float pos_out_fr = POS_KP * pos_err_fr + POS_KD * ((pos_err_fr - pos_error_fr_prev) / (float)PID_CONTROL_PERIOD_MS);
        float pos_out_rl = POS_KP * pos_err_rl + POS_KD * ((pos_err_rl - pos_error_rl_prev) / (float)PID_CONTROL_PERIOD_MS);
        float pos_out_rr = POS_KP * pos_err_rr + POS_KD * ((pos_err_rr - pos_error_rr_prev) / (float)PID_CONTROL_PERIOD_MS);

        /* save filtered error for next derivative */
        pos_error_fl_prev = pos_err_fl;
        pos_error_fr_prev = pos_err_fr;
        pos_error_rl_prev = pos_err_rl;
        pos_error_rr_prev = pos_err_rr;

        /* --- per-wheel speed commands (Mecanum rotation: FL/RL same, FR/RR opposite) --- */
        float speed_fl = pos_out_fl;
        float speed_rl = pos_out_rl;
        float speed_fr = -pos_out_fr;
        float speed_rr = -pos_out_rr;

        if (dir == CAR_DIR_CCW) {
            speed_fl = -speed_fl;
            speed_fr = -speed_fr;
            speed_rl = -speed_rl;
            speed_rr = -speed_rr;
        }

        Motor_ControlWheel(car->fl, speed_fl);
        Motor_ControlWheel(car->fr, speed_fr);
        Motor_ControlWheel(car->rl, speed_rl);
        Motor_ControlWheel(car->rr, speed_rr);

        HAL_Delay(PID_CONTROL_PERIOD_MS);

        Motor_UpdateEncoder(car->fl);
        Motor_UpdateEncoder(car->fr);
        Motor_UpdateEncoder(car->rl);
        Motor_UpdateEncoder(car->rr);

        /* re-read deltas for reach check and debug */
        dl0 = car->fl->total_counts - start_counts[0]; if (dl0 < 0) dl0 = -dl0;
        dr1 = car->fr->total_counts - start_counts[1]; if (dr1 < 0) dr1 = -dr1;
        dl2 = car->rl->total_counts - start_counts[2]; if (dl2 < 0) dl2 = -dl2;
        dr3 = car->rr->total_counts - start_counts[3]; if (dr3 < 0) dr3 = -dr3;

        /* all four wheels must reach target */
        if (dl0 >= target_counts && dr1 >= target_counts && dl2 >= target_counts && dr3 >= target_counts) {
            Car_StopAllWheels(car);
            return 0;
        }

        if ((HAL_GetTick() - start_ms) > timeout_ms) {
            Car_StopAllWheels(car);
            return -2;
        }
    }
}