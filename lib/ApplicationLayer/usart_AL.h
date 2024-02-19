#ifndef __USART_AL_H
#define __USART_AL_H


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "main.h"
#include "usart.h"
#include "macros.h"
#include "structures.h"
#include "dr_devices.h"

void DebugPrint(UART_HandleTypeDef *huart, const char* format, ...);

#endif