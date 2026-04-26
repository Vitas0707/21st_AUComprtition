#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "usart.h"

#define BLUETOOTH_UART_HANDLE (&huart2) // 根据实际使用的串口修改
#define BLUETOOTH_RX_BUFFER_SIZE 1 // 根据实际指令长度调整

void BLUETOOTH_Init(void) ;


#endif
