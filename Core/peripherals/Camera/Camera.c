#include "camera.h"
#include "Camera.h"



// --- 内部变量 ---
// 接收状态机状态
// 0: 等待帧头, 1: 等待类型, 2: 接收数据, 3: 等待帧尾
static uint8_t Rx_State = 0; 
static uint8_t Rx_DataType = 0;
static uint8_t Rx_TempBuf[32]; // 临时接收缓冲区
static uint8_t Rx_Cnt = 0;

//存储移动数据的数组
CameraData_t Recorded_Steps[MAX_STEPS]; // 开辟内存空间
uint8_t Step_Index = 0;                  // 当前指针
uint8_t Is_Recording = 0;                // 0:不记录, 1:记录中
uint8_t Is_Replaying = 0;                // 0:不回放, 1:回放中
uint8_t Replay_Index = 0;                // 回放指针

// 简单的方向映射 (根据你的实际需求修改)
// 假设摄像头发送 'F'(前), 'L'(左), 'R'(右)
int8_t Parse_Direction(char dir_char) {
    switch(dir_char) {
        case 'F': case 'f': return 1; // 前进
        case 'L': case 'l': return 2; // 左转
        case 'R': case 'r': return 3; // 右转
        default: return -1; // 未知
    }
}

/**
* @brief  初始化摄像头串口
* @notice 摄像头串口数据格式：0xFE + 类型 + 数据（f3……） + 0xEF
  */
void CAMERA_Init(void) {
    // 开启串口接收中断，准备接收第一个字节
    HAL_UART_Receive_IT(CAMERA_UART_HANDLE, (uint8_t*)0, 1); // 这里的指针只是占位，实际在中断回调里处理
    //0xFE和0xEF作为帧头和帧尾,防止数据流中的普通字符被误判为指令
}

/**
  * @brief  向摄像头发送指令
  * @param  cmd: 指令字符 ('M', 'R', 'G')
  */
void CAMERA_Send_Cmd(CameraCmd_t cmd) {
    uint8_t tx_data = (uint8_t)cmd;
    // 发送指令
    HAL_UART_Transmit(CAMERA_UART_HANDLE, &tx_data, 1, 100);
}

/**
  * @brief  串口中断回调函数 (需放在 main.c 的 HAL_UART_RxCpltCallback 中调用，或者在这里实现)
  *         注意：为了代码模块化，建议在 main.c 的中断回调里调用 CAMERA_UART_Receive_Process
  */
void CAMERA_UART_Receive_Process(uint8_t data) {
    CameraData_t parsed_data;
    
    // --- 状态机解析逻辑 ---
    
    // 1. 寻找帧头 0xFE
    if (Rx_State == 0) {
        if (data == 0xFE) {
            Rx_State = 1;
        }
    }
    // 2. 接收数据类型 ('D', 'S', 'N')
    else if (Rx_State == 1) {
        if (data == 'D' || data == 'S' || data == 'N') {
            Rx_DataType = data;
            Rx_Cnt = 0; // 清空计数
            Rx_State = 2;
        } else {
            Rx_State = 0; // 格式错误，重置
        }
    }
    // 3. 接收数据内容
    else if (Rx_State == 2) {
        // 寻找帧尾 0xEF
        if (data == 0xEF) {
            // --- 解析数据 ---
            memset(&parsed_data, 0, sizeof(parsed_data)); // 清空结构体
            parsed_data.type = Rx_DataType;

            if (Rx_DataType == 'D') {
                // 解析 "方向+数字"，假设格式如 "F3" (前3)
                if (Rx_Cnt >= 2) {
                    parsed_data.direction = Parse_Direction(Rx_TempBuf[0]);
                    // 简单的字符转数字 (仅支持个位数，如需多位需改用 atoi)
                    parsed_data.value = Rx_TempBuf[1] - '0'; 
                }
            } 
            else if (Rx_DataType == 'S') {
                // 解析字符串
                if (Rx_Cnt < 32) {
                    memcpy(parsed_data.str_buf, Rx_TempBuf, Rx_Cnt);
                    parsed_data.str_buf[Rx_Cnt] = '\0'; // 添加结束符
                }
            } 
            else if (Rx_DataType == 'N') {
                // 解析纯数字
                if (Rx_Cnt < 32) {
                    Rx_TempBuf[Rx_Cnt] = '\0';
                    parsed_data.value = atoi((char*)Rx_TempBuf);
                }
            }

            // 调用用户回调函数处理数据
            CAMERA_Data_Received_Callback(&parsed_data);

            // 重置状态
            Rx_State = 0;
        } 
        else {
            // 存入临时缓冲区
            if (Rx_Cnt < 32) {
                Rx_TempBuf[Rx_Cnt++] = data;
            } else {
                Rx_State = 0; // 缓冲区溢出，重置
            }
        }
    }
}