# include "Motor.h"
#include "main.h"
# include "math.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>


//电机初始化
void Motor_Init(TIM_HandleTypeDef* htim, uint32_t Channel) {
    HAL_TIM_PWM_Start(htim, Channel);
}


//设置电机PWM占空比
void Motor_SetPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t pwm) {
    if (pwm > 100) pwm = 100; // 限制PWM值在0-100范围内
    __HAL_TIM_SET_COMPARE(htim, Channel, pwm);
}

//设置电机转动方向
void Motor_SetDirection(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin1,uint16_t GPIO_Pin2,uint8_t direction) {

    if (direction == 0) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin2, GPIO_PIN_SET);
    }
}