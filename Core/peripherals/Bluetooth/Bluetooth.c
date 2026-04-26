#include "Bluetooth.h"
#include "stm32f4xx_hal_uart.h"
#include <sys/types.h>

uint8_t Bluetooth_Rx_Buffer[BLUETOOTH_RX_BUFFER_SIZE]; // 蓝牙接收缓冲区

void BLUETOOTH_Init(void) {
   HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, Bluetooth_Rx_Buffer, sizeof(Bluetooth_Rx_Buffer)); // 开启蓝牙串口接收中断
}


