#include "Bluetooth.h"
#include "Camera.h"
#include "Motor.h"
#include "stm32f4xx_hal_uart.h"
#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

uint8_t Bluetooth_Rx_Buffer[1]; // 蓝牙接收缓冲区

void BLUETOOTH_Init(void) {
   HAL_UART_Receive_IT(&BLUETOOTH_UART_HANDLE, Bluetooth_Rx_Buffer, sizeof(Bluetooth_Rx_Buffer)); 
}

void Send_Speed_Data(Car_t *car) {
    uint8_t tx_buf[19]; // 定义 19 字节的发送缓冲区

    // 1. 填充包头
    tx_buf[0] = 0xA5;

    // 2. 填充原数据 (4个 float，共 16 字节)
    // 使用 memcpy 一次性拷贝，比逐个赋值更简洁、更高效
    memcpy(&tx_buf[1], &car->fl->speed_m_s, 4);
    memcpy(&tx_buf[5], &car->fr->speed_m_s, 4);
    memcpy(&tx_buf[9], &car->rl->speed_m_s, 4);
    memcpy(&tx_buf[13], &car->rr->speed_m_s, 4);

    // 3. 计算校验和 (原数据所有字节之和的低 8 位)
    uint8_t sum = 0;
    for(int i = 1; i <= 16; i++) {
        sum += tx_buf[i];
    }
    tx_buf[17] = sum;

    // 4. 填充包尾
    tx_buf[18] = 0x5A;

    // 5. 发送数据
   HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
}
