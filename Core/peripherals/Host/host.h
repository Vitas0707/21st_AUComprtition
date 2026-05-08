#ifndef __HOST_H
#define __HOST_H

#include "stm32f4xx_hal.h"
#include "unistd.h"

ssize_t _write(int file, char *ptr, int len);


#endif // __HOST_H
