/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "font.h"
#include "Motor.h"
#include "Camera.h"
#include "Bluetooth.h"
#include <stdbool.h>
#include <stdint.h>
#include "host.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor_t FL, FR, RL, RR;
Car_t car;
char cmd[1] = {'1'};
char check[] ="接收成功";
// extern SetPID_t PID_Params;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

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
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //电机初始化
  Motor_WheelInit(&FL, &FL_TIM_HANDLE, FL_TIM_CHANNEL, FL_IN_PORT, FL_IN_PIN1, FL_IN_PIN2, &FL_ENCODER_TIM, WHEEL_DIAMETER_M, ENCODER_PPR, ENCODER_GEAR_RATIO);
  Motor_WheelInit(&FR, &FR_TIM_HANDLE, FR_TIM_CHANNEL, FR_IN_PORT, FR_IN_PIN1, FR_IN_PIN2, &FR_ENCODER_TIM, WHEEL_DIAMETER_M, ENCODER_PPR, ENCODER_GEAR_RATIO);
  Motor_WheelInit(&RL, &RL_TIM_HANDLE, RL_TIM_CHANNEL, RL_IN_PORT, RL_IN_PIN1, RL_IN_PIN2, &RL_ENCODER_TIM, WHEEL_DIAMETER_M, ENCODER_PPR, ENCODER_GEAR_RATIO);
  Motor_WheelInit(&RR, &RR_TIM_HANDLE, RR_TIM_CHANNEL, RR_IN_PORT, RR_IN_PIN1, RR_IN_PIN2, &RR_ENCODER_TIM, WHEEL_DIAMETER_M, ENCODER_PPR, ENCODER_GEAR_RATIO);
  Motor_SetDirectionInverted(&FL, true);
  Motor_SetDirectionInverted(&RL, true);
  Motor_SetDirectionInverted(&RR, true);
  Car_Init(&car, &FL, &FR, &RL, &RR, TRACK_WIDTH_M);
  //OLED初始化
  HAL_Delay(20);
  OLED_Init();
  //摄像头初始化
  CAMERA_Init();
  //蓝牙初始化
  BLUETOOTH_Init();
  //主机通信初始化
  Host_Init();

  Motor_PIDInit(&FL,4.27f, 2.94f, 9.43f, PID_OUTPUT_LIMIT);
  // Motor_PIDInit(&FR,2.62f, 1.59f, 0.00f, PID_OUTPUT_LIMIT);
  Motor_PIDInit(&RL,4.27f, 2.94f, 9.43f, PID_OUTPUT_LIMIT);
  // Motor_PIDInit(&RR,2.62f, 1.59f, 0.00f, PID_OUTPUT_LIMIT);
  Motor_PIDInit(&FR, 4.27f, 2.94f, 9.43f, PID_OUTPUT_LIMIT);
  Motor_PIDInit(&RR,4.27f, 2.94f, 9.43f, PID_OUTPUT_LIMIT);


  // Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.30f);
  // Car_RotateAngle(&car, CAR_DIR_CCW, 95);
  // Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.90f);
  // Car_DriveDistance(&car, CAR_DIR_BACKWARD, 0.01f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Motor_ControlWheel(&FL, 0.50f);
    // Motor_ControlWheel(&FR, 0.50f);
    // Motor_ControlWheel(&RL, 0.50f);
    // Motor_ControlWheel(&RR, 0.50f);
    // printf("para:%.2f,%.2f,%.2f,%.2f\r\n", FL.speed_m_s, FR.speed_m_s, RL.speed_m_s, RR.speed_m_s);
    // printf("para: %.2f,%.2f,%d,%.2f,%.2f\r\n", 0.50f,RR.speed_m_s,(int)RR.invert_direction,RR.pwm_duty,RR.pid.integral);
    // Motor_PIDSetParams(&RR, PID_Params.Kp, PID_Params.Ki, PID_Params.Kd);

    /*  去程  */
    if(Move_permission == true){
      Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.3f);
      switch(Steps[Step_Index].direction){
        case 1:{//直行
          Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.30f);
          Car_DriveDistance(&car, CAR_DIR_FORWARD, (Steps[Step_Index].steps-1)*0.3f);
          Car_DriveDistance(&car, CAR_DIR_BACKWARD, 0.01f);
          HAL_UART_Transmit(&CAMERA_UART_HANDLE, (uint8_t*)&cmd, sizeof(cmd), HAL_MAX_DELAY);
          break;
        }
        case 2:{//右转
          Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.30f);
          Car_RotateAngle(&car, CAR_DIR_CW, 100);
          Car_DriveDistance(&car, CAR_DIR_FORWARD,(Steps[Step_Index].steps-1)*0.3f);
          Car_DriveDistance(&car, CAR_DIR_BACKWARD, 0.01f);
          HAL_UART_Transmit(&CAMERA_UART_HANDLE, (uint8_t*)&cmd, sizeof(cmd), HAL_MAX_DELAY);
          break;
        }
        case 3:{//左转
          Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.30f);
          Car_RotateAngle(&car, CAR_DIR_CCW, 100);
          Car_DriveDistance(&car, CAR_DIR_FORWARD, (Steps[Step_Index].steps-1)*0.3f);
          Car_DriveDistance(&car, CAR_DIR_BACKWARD, 0.01f);
          HAL_UART_Transmit(&CAMERA_UART_HANDLE, (uint8_t*)&cmd, sizeof(cmd), HAL_MAX_DELAY);
          break;
        }
        case 4:{//到达终点
          Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.30f);
          break;
        }
        default:break;
      }
      Move_permission = false;
    }
    /*  显示内容   */
    else if(Show_permission == true){
      OLED_NewFrame();
      if (Camera_Data.str_index == '0') {
      OLED_PrintString(1, 1, "动物", &font16x16, OLED_COLOR_NORMAL);
      }else if (Camera_Data.str_index == '1') {
      OLED_PrintString(1, 1, "人类", &font16x16, OLED_COLOR_NORMAL);
      }else if (Camera_Data.str_index == '2') {
      OLED_PrintString(1, 1, "水果", &font16x16, OLED_COLOR_NORMAL);
      }
      else {
      OLED_PrintString(1, 1, "数据有误", &font16x16, OLED_COLOR_NORMAL);
      }
      OLED_ShowFrame();
      Show_permission = false;//显示完成
    }
    /*  转圈   */
    else if(Revolve_permission == true){
      for(int i = 0;i<Camera_Data.value;i++){
        Car_RotateAngle(&car, CAR_DIR_CCW, 370);
        HAL_Delay(300);
      }
      Revolve_permission = false;//转圈完成
    }
    /*  回退   */
    else if(Back_permission == true){
      switch(Steps[Step_Index].direction){
        case 1:{//直行
          Car_DriveDistance(&car, CAR_DIR_FORWARD, Steps[Step_Index].steps*0.3f);
          break;
        }
        case 2:{//左转
          Car_DriveDistance(&car, CAR_DIR_FORWARD,Steps[Step_Index].steps*0.3f);
          Car_RotateAngle(&car, CAR_DIR_CW, 90);
          break;
        }
        case 3:{//右转
          Car_DriveDistance(&car, CAR_DIR_FORWARD, Steps[Step_Index].steps*0.3f);
          Car_RotateAngle(&car, CAR_DIR_CCW, 90);
          break;
        }
        default:break;
      }
      if(Step_Index == 0){
        Back_permission = false;//回退完成
      }
      Step_Index--;
    }

    HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  if (huart == &BLUETOOTH_UART_HANDLE) {
    if (Bluetooth_Rx_Buffer[0] == 'M') {
      HAL_UART_Transmit_IT(&CAMERA_UART_HANDLE, (uint8_t*)&cmd, sizeof(cmd));
      HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)check, sizeof(check), HAL_MAX_DELAY);
    } else if (Bluetooth_Rx_Buffer[0] == 'B') {
      Back_permission = true;
      HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)check, sizeof(check), HAL_MAX_DELAY);
    }
    HAL_UART_Receive_IT(&BLUETOOTH_UART_HANDLE, Bluetooth_Rx_Buffer, sizeof(Bluetooth_Rx_Buffer));
  }

  else if (huart == &CAMERA_UART_HANDLE) {
    CAMERA_UART_Receive_Process(cam_rx_byte);
    HAL_UART_Receive_IT(&CAMERA_UART_HANDLE, &cam_rx_byte, 1);
  }

  else if(huart == &HOST_UART_HANDLE) {
    Host_dataparser(Host_Rx_byte);
    HAL_UART_Receive_IT(&HOST_UART_HANDLE, &Host_Rx_byte, 1);
  }
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
