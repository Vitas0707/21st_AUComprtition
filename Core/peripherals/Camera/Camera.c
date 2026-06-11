#include "Camera.h"
#include "Bluetooth.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>
#include <stdio.h>


// 状态机：0 = 等待帧头(0xFE), 1 = 收到类型字节, 2 = 接收数据直到帧尾(0xEF)
static uint8_t Rx_State = 0;
static uint8_t Rx_DataType = 0;
static uint8_t Rx_TempBuf[32];
static uint8_t Rx_Cnt = 0;
static stepbuf_t stepbuf = is_dir;
static char check[64];
static step_t s = {0, 0};

// 对外存储
step_t Steps[MAX_STEPS];
CameraData_t Camera_Data;
uint8_t Step_Index = 0;
uint8_t cam_rx_byte = 0;

bool Show_permission = false;
bool Move_permission = false;
bool Revolve_permission = false;
bool Back_permission = false;

CarState_t car_state = CAR_IDLE;
bool is_running = false;
uint32_t wait_start_tick = 0;



static uint8_t Parse_Direction(char dir_char) {
    switch (dir_char) {
        case 'F': case 'f': return 1; // forward
        case 'R': case 'r': return 2; // right
        case 'L': case 'l': return 3; // left
        case 'B': case 'b': return 4; // backward
        case 'O': case 'o': return 5; // over
        default: return 0;           // invalid
    }
}


void CAMERA_Init(void) {
    HAL_UART_Receive_IT(&CAMERA_UART_HANDLE, &cam_rx_byte, 1);
}

/**
  * @brief 串口逐字节处理入口（在 HAL_UART_RxCpltCallback 中调用）
  * @param data 收到的单字节
  *
  * 协议： <[type][data...]>
  */
void CAMERA_UART_Receive_Process(uint8_t data) {
    if (Rx_State == 0) {
        if (data == '<') {
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
        if (data == '>') {
            // 帧尾：根据类型解析已接收的 payload（Rx_TempBuf[0..Rx_Cnt-1]）
            if (Rx_DataType == 'M') {
                if (Rx_Cnt == 2){
                    char dir_char = (char)Rx_TempBuf[0];
                    uint8_t dir = Parse_Direction(dir_char);
                    s.direction = dir;
                    s.steps = (uint8_t)Rx_TempBuf[1] - '0';
                    if(s.direction != 0 && s.steps != 0){
                    stepbuf = can_move;
                    }
                }                
                else if (Rx_Cnt == 1){
                    /* ---- 箭头和数字分开来的情况 ---- */
                    if(Rx_TempBuf[0] == 'F' || Rx_TempBuf[0] == 'f' || Rx_TempBuf[0] == 'R' || Rx_TempBuf[0] == 'r' || Rx_TempBuf[0] == 'L' || Rx_TempBuf[0] == 'l' || Rx_TempBuf[0] == 'B' || Rx_TempBuf[0] == 'b' || Rx_TempBuf[0] == 'O' || Rx_TempBuf[0] == 'o'){
                        char dir_char = (char)Rx_TempBuf[0];
                        uint8_t dir = Parse_Direction(dir_char);
                        s.direction = dir;
                        stepbuf = is_steps;   /* 已收到方向，等待数字 */
                    }
                    else if (Rx_TempBuf[0] >= '0' && Rx_TempBuf[0] <= '7') {
                        s.steps = (uint8_t)Rx_TempBuf[0] - '0';
                        if (s.direction != 0) {
                            stepbuf = can_move;  /* 方向已有值，凑齐 */
                        }
                    }

                    /* 两半都在了 */
                    if(s.direction != 0 && s.steps != 0){
                        stepbuf = can_move;
                    }
                }
                
                if (stepbuf == can_move && Step_Index < MAX_STEPS) {
                    Steps[Step_Index] = s;
                    int data_len = snprintf(check, sizeof(check), "M%d%d", s.direction, s.steps);
                    HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)check, data_len, HAL_MAX_DELAY);
                    /* 推入后重置累积器，准备下一轮 */
                    s.direction = 0;
                    s.steps = 0;
                    stepbuf = is_dir;
                    Move_permission = true;
                }
            }
            else if (Rx_DataType == 'R') {
                Camera_Data.str_index = Rx_TempBuf[Rx_Cnt - 1] ; 
                Show_permission = true;//允许显示屏显示内容
                int data_len = snprintf(check, sizeof(check), "R%c", Camera_Data.str_index);
                HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)check, data_len, HAL_MAX_DELAY);
            } 
            else if (Rx_DataType == 'G') {
                Camera_Data.value = Rx_TempBuf[Rx_Cnt - 1] - '0';
                Revolve_permission = true;//允许转圈
                int data_len = snprintf(check, sizeof(check), "G%d", Camera_Data.value);
                HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)check, data_len, HAL_MAX_DELAY);
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