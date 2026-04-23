#ifndef __MOTOR_H
#define __MOTOR_H

//电机1
# define M1TIM &htim3
# define M1CHANNEL TIM_CHANNEL_1

//电机2
# define M2TIM &htim3
# define M2CHANNEL TIM_CHANNEL_2

//电机3
# define M3TIM &htim4
# define M3CHANNEL TIM_CHANNEL_1

//电机4
# define M4TIM &htim4
# define M4CHANNEL TIM_CHANNEL_2



void Motor_Init(void);
void Motor_SetPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t pwm);
void Motor_SetDirection(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin1,uint16_t GPIO_Pin2,uint8_t direction); 

#endif // __MOTOR_H
