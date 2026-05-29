#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "usart.h"

#define BLUETOOTH_UART_HANDLE huart2
extern uint8_t Bluetooth_Rx_Buffer[1];
void BLUETOOTH_Init(void) ;


#endif
