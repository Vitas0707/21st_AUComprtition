#include "host.h"
#include "stdio.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "unistd.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>


uint8_t Host_Rx_byte;
uint8_t Host_Rx_Buffer[32];
uint8_t Host_Rx_Index = 0;
SetPID_t PID_Params = {0.0f, 0.0f, 0.0f};
RxState_t Rxstate = WaitHead;
Command_t CurrentCMD = CMD_Kp;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&HOST_UART_HANDLE, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void Host_Init() {
    HAL_UART_Receive_IT(&HOST_UART_HANDLE, &Host_Rx_byte, 1);
}

void Host_dataparser(uint8_t data){

    switch (Rxstate){
        case WaitHead:
            if(data =='K'||data =='k'){
                Rxstate = WaitFlag;
                Host_Rx_Index = 0;
            }
            break;

        case WaitFlag:
            if(data =='p'||data =='P'){
                CurrentCMD = CMD_Kp;
                Rxstate = WaitData;
            }
            else if(data =='i'||data =='I'){
                CurrentCMD = CMD_Ki;
                Rxstate = WaitData;
            }
            else if(data =='d'||data =='D'){
                CurrentCMD = CMD_Kd;
                Rxstate = WaitData;
            }
            else{
                Rxstate = WaitHead;
            }
            Host_Rx_Index = 0;
            break;

        case WaitData:
            if(data == '!'){
                Host_Rx_Buffer[Host_Rx_Index]='\0';
				uint8_t *endptr;
				float NewValue = strtof((char*)Host_Rx_Buffer, (char**)&endptr);
                switch (CurrentCMD){
                    case CMD_Kp:
                        {
                            PID_Params.Kp = NewValue;
                            printf("Received Kp: %.2f\r\n", PID_Params.Kp);
                        }
                        break;
                    case CMD_Ki:
                        {
                            PID_Params.Ki = NewValue;
                            printf("Received Ki: %.2f\r\n", PID_Params.Ki);
                        }
                        break;
                    case CMD_Kd:
                        {
                            PID_Params.Kd = NewValue;
                            printf("Received Kd: %.2f\r\n", PID_Params.Kd);
                        }
                        break;
                }
                Rxstate = WaitHead;
            }
            else{
                if(Host_Rx_Index < sizeof(Host_Rx_Buffer)-1){
                    Host_Rx_Buffer[Host_Rx_Index++] = data;
                }
            }
    }
}
