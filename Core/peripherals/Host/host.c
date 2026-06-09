#include "host.h"
#include "stdio.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "unistd.h"
#include "usart.h"
#include "bluetooth.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>


uint8_t Host_Rx_byte;
uint8_t Host_Rx_Buffer[32];
uint8_t Host_Rx_Index = 0;
SetPID_t PID_Params = {0.0f, 0.0f, 0.0f};
RxState_t Rxstate = WaitHead;
Command_t CurrentCMD = CMD_Kp;

uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};



void send_justfloat(float var1,float var2,float var3,float var4){
    float data[4] = {var1,var2,var3,var4};
    HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, (uint8_t*)data, 4*sizeof(float), HAL_MAX_DELAY);
    HAL_UART_Transmit(&BLUETOOTH_UART_HANDLE, tail, 4*sizeof(float), HAL_MAX_DELAY);
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
