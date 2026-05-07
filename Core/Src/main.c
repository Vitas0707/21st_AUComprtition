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
#include "i2c.h"
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
// cam_rx_byte is defined in Camera.c
Motor_t wheelM1_fl, wheelM2_fr, wheelM3_rl, wheelM4_rr;
RobotDrive_t robot_drive;
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
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  
  // 电机与底盘初始化
  Motor_InitMG310Wheel(&wheelM1_fl, M1_EN_TIM, M1_EN_CHANNEL, GPIOB, GPIO_PIN_0, GPIO_PIN_1, M1_Encoder_TIM);
  Motor_InitMG310Wheel(&wheelM2_fr, M2_EN_TIM, M2_EN_CHANNEL, GPIOC, GPIO_PIN_0, GPIO_PIN_1, M2_Encoder_TIM);
  Motor_InitMG310Wheel(&wheelM3_rl, M3_EN_TIM, M3_EN_CHANNEL, GPIOD, GPIO_PIN_14, GPIO_PIN_15, M3_Encoder_TIM);
  Motor_InitMG310Wheel(&wheelM4_rr, M4_EN_TIM, M4_EN_CHANNEL, GPIOD, GPIO_PIN_10, GPIO_PIN_11, M4_Encoder_TIM);
  Motor_SetDirectionInverted(&wheelM1_fl, true);
  Motor_SetDirectionInverted(&wheelM3_rl, true);
  RobotDrive_Init(&robot_drive, &wheelM1_fl, &wheelM2_fr, &wheelM3_rl, &wheelM4_rr, ROBOT_TRACK_WIDTH_M, ROBOT_WHEEL_BASE_M);
  //电机PID参数设置
  // 左轮（前左和后左）设置较低的Kp以减速
  Motor_PIDSetParams(&wheelM1_fl, 25.0f, 15.0f, 3.0f, 1000.0f, 100.0f);  // 降低Kp从35到25
  Motor_PIDSetParams(&wheelM3_rl, 25.0f, 15.0f, 3.0f, 1000.0f, 100.0f);  // 降低Kp从35到25
  // 右轮保持原参数
  Motor_PIDSetParams(&wheelM2_fr, 35.0f, 15.0f, 3.0f, 1000.0f, 100.0f);
  Motor_PIDSetParams(&wheelM4_rr, 35.0f, 15.0f, 3.0f, 1000.0f, 100.0f);

  //OLED初始化
  HAL_Delay(20);
  OLED_Init();
  //摄像头初始化
  CAMERA_Init();
  //蓝牙初始化
  BLUETOOTH_Init();


  // Motor_StartWheel(&wheelM1_fl, 60, 0); 
  // Motor_RotateRevolutionsBlocking(&wheelM1_fl, 5, 2.0f);
  // Motor_RotateRevolutionsBlocking(&wheelM2_fr, 5, 0.2F);
  // Motor_RotateRevolutionsBlocking(&wheelM3_rl, 5, 0.2F);
  // Motor_RotateRevolutionsBlocking(&wheelM4_rr, 5, 0.2F);
  Robot_RotateWheelsRevolutionsBlocking(&robot_drive, 1, 2.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //OLED测试
    // OLED_PrintString(1, 1, "动物", &font16x16, OLED_COLOR_NORMAL);
    // OLED_ShowFrame();

    //电机测试：每个轮子转 5 圈，线速度 0.2 m/s

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    // __HAL_TIM_SET_COMPARE(M1_EN_TIM, TIM_CHANNEL_1, 100); 
    // HAL_TIM_PWM_Start(M1_EN_TIM, M1_EN_CHANNEL);


    if(Show_permission == 1){
      
      OLED_NewFrame();
      
      if (Camera_Data.str_index == 0) {
        OLED_PrintString(1, 1, "动物", &font16x16, OLED_COLOR_NORMAL);
        }else if (Camera_Data.str_index == 1) {
        OLED_PrintString(1, 1, "人类", &font16x16, OLED_COLOR_NORMAL);
        }else if (Camera_Data.str_index == 2) {
        OLED_PrintString(1, 1, "水果", &font16x16, OLED_COLOR_NORMAL);
        }else if (Camera_Data.str_index == 3) {
        OLED_PrintString(1, 1, "枪械", &font16x16, OLED_COLOR_NORMAL);
        }else {
        OLED_PrintString(1, 1, "数据有误", &font16x16, OLED_COLOR_NORMAL);
        }

      OLED_ShowFrame();//一直显示在屏幕上，直到下一次更新
      Show_permission = 0;//显示完成
    }
    

    HAL_Delay(10);//适当延时，避免过度占用CPU资源
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
  if (huart == BLUETOOTH_UART_HANDLE) {
    extern uint8_t Bluetooth_Rx_Buffer[BLUETOOTH_RX_BUFFER_SIZE]; // 蓝牙接收缓冲区
    if (Bluetooth_Rx_Buffer[0] == 'M' || Bluetooth_Rx_Buffer[0] == 'R' || Bluetooth_Rx_Buffer[0] == 'G') {
      HAL_UART_Transmit_IT(CAMERA_UART_HANDLE, Bluetooth_Rx_Buffer, sizeof(Bluetooth_Rx_Buffer));
    } else if (Bluetooth_Rx_Buffer[0] == 'B') {
      // 控制小车回去的逻辑（用户实现）
    }
    HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, Bluetooth_Rx_Buffer, sizeof(Bluetooth_Rx_Buffer));
  }
  else if (huart == CAMERA_UART_HANDLE) {
    // 逐字节交给摄像头解析模块处理
    CAMERA_UART_Receive_Process(cam_rx_byte);
    // 开启下一次接收
    HAL_UART_Receive_IT(CAMERA_UART_HANDLE, &cam_rx_byte, 1);
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
