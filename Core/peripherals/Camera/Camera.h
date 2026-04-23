#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f4xx_hal.h" // 根据你的单片机型号修改，如 stm32f4xx_hal.h
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 定义使用的串口句柄
#define CAMERA_UART_HANDLE huart1
//定义步数存储长度
#define MAX_STEPS 100
//导出记录步数的变量
extern uint8_t Step_Index;                      // 当前记录到了第几步
extern uint8_t Is_Recording;                    // 是否在记录模式
extern uint8_t Is_Replaying;                    // 是否在回放模式
extern uint8_t Replay_Index;                    // 回放指针

// --- 1. 指令定义 ---
typedef enum {
    CMD_MOVE = 'M',       // 移动/开始
    CMD_RECOGNIZE = 'R',  // 识别
    CMD_GET_NUM = 'G',    // 获取数字
} CameraCmd_t;

// --- 2. 接收数据类型定义 ---
typedef enum {
    DATA_TYPE_DIR_NUM = 'D', // 方向+数字 (如: 前进3格)
    DATA_TYPE_STRING = 'S',  // 字符数据 (如: "动物")
    DATA_TYPE_NUMBER = 'N'   // 纯数字 (如: 123)
} CameraDataType_t;

// --- 3. 解析后的数据结构 ---
typedef struct {
    uint8_t type;      // 数据类型 ('D', 'S', 'N')
    int8_t direction;  // 方向 (仅类型D有效: 1=前, 2=后, 3=左, 4=右, -1=无效)
    uint8_t steps;     //行动步数 (仅类型D有效)
    int value;         // 数值 (类型D和N有效)
    char str_buf[32];  // 字符串缓冲区 (仅类型S有效)
} CameraData_t;

// --- 4. 函数声明 ---
void CAMERA_Init(void);
void CAMERA_Send_Cmd(CameraCmd_t cmd);

// 用户需要实现的回调函数 (在 main.c 或其他地方实现)
void CAMERA_Data_Received_Callback(CameraData_t *data);

// 内部处理函数 (在中断中调用)
void CAMERA_UART_Receive_Process(uint8_t data);

#endif
