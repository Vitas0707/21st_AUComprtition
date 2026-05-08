#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f4xx_hal.h" // 根据你的单片机型号修改，如 stm32f4xx_hal.h
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

// 定义使用的串口句柄
#define CAMERA_UART_HANDLE (&huart1)
//定义步数存储长度
#define MAX_STEPS 100
//定义状态
extern uint8_t Show_permission;//是否允许显示屏显示内容
extern uint8_t Move_permission;//是否在移动
extern uint8_t Revolve_permission;//是否在转圈

// --- 解析后的数据结构 ---
typedef struct {
    // uint8_t type;          // 数据类型 ('R'：字符串，传入索引即可, 'G'：数字)
    uint8_t value;         // 数值（类型 'G' 有效）
    uint8_t str_index;     // 字符串索引（类型 'R' 有效）
} CameraData_t;

typedef struct {
    // uint8_t type;            // 数据类型 ('M')
    uint8_t direction;       // 移动方向
    uint8_t steps;           // 移动距离
} step_t;

// 存储 M 类型的步骤数组（在 Camera.c 中定义）
extern step_t Steps[MAX_STEPS];
// 最近解析出的 R/G 数据（在 Camera.c 中定义）
extern CameraData_t Camera_Data;


// 用于串口单字节接收的缓冲（在 Camera.c 中定义）
extern uint8_t cam_rx_byte;
// 显示许可标志（在 Camera.c 中定义）
extern uint8_t Show_permission;

// --- 函数声明 ---
void CAMERA_Init(void);

// 用户需要实现的回调函数 (在 main.c 或其他地方实现)
void CAMERA_Data_Received_Callback(CameraData_t *data);

// 内部处理函数 (在中断中调用)
void CAMERA_UART_Receive_Process(uint8_t data);

#endif
