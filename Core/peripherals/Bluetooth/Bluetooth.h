#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "Motor.h"
#include "usart.h"

#define BLUETOOTH_UART_HANDLE huart4
extern uint8_t Bluetooth_Rx_Buffer[1];
void BLUETOOTH_Init(void) ;
void Send_Speed_Data(Car_t *car) ;


#endif
