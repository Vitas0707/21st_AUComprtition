#ifndef __HOST_H
#define __HOST_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#define HOST_UART_HANDLE huart1

typedef struct{
    float Kp ;
    float Ki ;
    float Kd ;
}SetPID_t;

typedef enum {
    WaitHead,
    WaitFlag,
    WaitData,
}RxState_t;

typedef enum {
    CMD_Kp,
    CMD_Ki,
    CMD_Kd,
}Command_t;

extern RxState_t Rxstate;
extern Command_t CurrentCMD;


extern uint8_t Host_Rx_byte;
extern uint8_t Host_Rx_Buffer[32]; 
extern uint8_t Host_Rx_Index;
extern SetPID_t PID_Params;


int _write(int file, char *ptr, int len);

void Host_Init() ;
void Host_dataparser(uint8_t data);

#endif // __HOST_H
