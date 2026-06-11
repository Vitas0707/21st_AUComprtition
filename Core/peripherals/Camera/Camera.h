#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f4xx_hal.h" 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

#define CAMERA_UART_HANDLE huart4
#define MAX_STEPS 100

typedef struct {
    // uint8_t type;       // 数据类型 ('R'：字符串，传入索引即可, 'G'：数字)
    uint8_t value;         // 数值（类型 'G' 有效）
    uint8_t str_index;     // 字符串索引（类型 'R' 有效）
} CameraData_t;

typedef struct {
    // uint8_t type;            // 数据类型 ('M')
    uint8_t direction;       // 移动方向
    uint8_t steps;           // 移动距离
} step_t;

typedef  enum{
    is_dir,
    is_steps,
    can_move,
} stepbuf_t;

typedef enum {
    CAR_IDLE = 0,
    CAR_WAITING_CAMERA,
    CAR_MOVING,
    CAR_RECOVERY,
    CAR_STA
} CarState_t;


extern step_t Steps[MAX_STEPS];
extern uint8_t Step_Index;
extern CameraData_t Camera_Data;
extern uint8_t cam_rx_byte;
extern bool Show_permission;
extern bool Move_permission;
extern bool Revolve_permission;
extern bool Back_permission;

extern CarState_t car_state;
extern bool is_running;
extern uint32_t wait_start_tick;



void CAMERA_Init(void);
void CAMERA_UART_Receive_Process(uint8_t data);

#endif