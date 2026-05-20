#include "Camera.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// --- 内部变量 ---
// 状态机：0 = 等待帧头(0xFE), 1 = 收到类型字节, 2 = 接收数据直到帧尾(0xEF)
static uint8_t Rx_State = 0;
static uint8_t Rx_DataType = 0;
static uint8_t Rx_TempBuf[32];
static uint8_t Rx_Cnt = 0;

// 对外存储
    //数据保存部分
step_t Steps[MAX_STEPS];//储存移动类型数据
CameraData_t Camera_Data;//储存识别和转圈数据
uint8_t Step_Index = 0;//当前记录到了第几步
uint8_t cam_rx_byte = 0;
    //单片机状态部分
bool Show_permission = 0;//是否允许显示屏显示内容
bool Move_permission = 0;//是否在移动
bool Revolve_permission = 0;//是否在转圈

// 简单方向解析（根据协议字符映射方向枚举值）
static uint8_t Parse_Direction(char dir_char) {
    switch (dir_char) {
        case 'F': case 'f': return 1; // forward
        case 'L': case 'l': return 2; // left
        case 'R': case 'r': return 3; // right
        default: return 0;           // unknown
    }
}

/**
    * @brief  初始化摄像头串口
    * @note   摄像头串口数据格式：0xFE + 类型 + 数据（f3……） + 0xEF
  */
void CAMERA_Init(void) {
    // 开启摄像头串口接收中断，准备接收数据
    HAL_UART_Receive_IT(&CAMERA_UART_HANDLE, &cam_rx_byte, 1);
    //0xFE和0xEF作为帧头和帧尾,防止数据流中的普通字符被误判为指令
}

/**
  * @brief 串口逐字节处理入口（在 HAL_UART_RxCpltCallback 中调用）
  * @param data 收到的单字节
  *
  * 协议：0xFE [type] [payload...] 0xEF
  * - type = 'R' : payload 为字符串 -> 存入 CameraData_t.str_index 并回调
  * - type = 'G' : payload 为 ASCII 数字 -> 转为整数存入 CameraData_t.value 并回调
  * - type = 'M' : payload 为方向字符 + 数字(可多位) -> 存入 Steps 数组
  */
void CAMERA_UART_Receive_Process(uint8_t data) {
    if (Rx_State == 0) {
        if (data == 0xFE) {
            Rx_State = 1;
            Rx_Cnt = 0;
            Rx_DataType = 0;
        }
        return;
    }

    else if (Rx_State == 1) {
        // 第一个字节是数据类型，不保存到缓冲区
        Rx_DataType = data;
        Rx_State = 2;
        Rx_Cnt = 0;
        return;
    }

    else if (Rx_State == 2) {
        if (data == 0xEF) {
            // 帧尾：根据类型解析已接收的 payload（Rx_TempBuf[0..Rx_Cnt-1]）
            if (Rx_DataType == 'M') {
                char dir_char = (char)Rx_TempBuf[0];
                uint8_t dir = Parse_Direction(dir_char);
                
                step_t s;
                s.direction = dir;
                s.steps = (uint8_t)Rx_TempBuf[1];
                if (Step_Index < MAX_STEPS) {
                    Steps[Step_Index++] = s;
                }
            }
            else if (Rx_DataType == 'R') {
                Rx_TempBuf[Rx_Cnt - 1] = Camera_Data.str_index; 
                Show_permission = true;//允许显示屏显示内容
            } 
            else if (Rx_DataType == 'G') {
                Rx_TempBuf[Rx_Cnt - 1] = Camera_Data.value;
                Revolve_permission = true;//允许转圈
            }
            // 重置状态机
            Rx_State = 0;
            Rx_Cnt = 0;
            Rx_DataType = 0;
        }
        else {
            // 存入 payload 缓冲区（第一个字节为 type，不进入此缓冲区）
            if (Rx_Cnt < sizeof(Rx_TempBuf) - 1) {
                Rx_TempBuf[Rx_Cnt++] = data;
            } 
            else {
                // 溢出：丢弃本帧并重置
                Rx_State = 0;
                Rx_Cnt = 0;
                Rx_DataType = 0;
            }
        }
    }
}