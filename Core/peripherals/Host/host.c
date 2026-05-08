#include "host.h"
#include "stdio.h"
#include "unistd.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#define HOST_UART_HANDLE (&huart4)


ssize_t _write(int file, char *ptr, int len)
{
    //判断文件描述符
    // file == 1 代表标准输出 (stdout)
    // file == 2 代表标准错误 (stderr)
    // 我们只关心这两个，其他的可以忽略
    if (file == 1 || file == 2) 
    {
        //调用 HAL 库发送数据
        HAL_UART_Transmit(HOST_UART_HANDLE, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        //返回成功发送的长度，告诉 printf 任务完成了
        return len; 
    }
    
    return -1; // 如果是不支持的文件描述符，返回 -1 表示错误
}
