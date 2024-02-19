/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with UART Communication.

*/

#include "usart_AL.h"

uart_t serial = {
            .uart1      = &huart1,
            .write      = HAL_UART_Transmit,
            .read       = HAL_UART_Receive,
            .print      = DebugPrint,
            .bufferData = {0},
};

void DebugPrint(UART_HandleTypeDef *huart, const char* format, ...) {
    memset(serial.bufferData, 0x00, sizeof(serial.bufferData));
    va_list args;
    va_start(args, format);
    vsnprintf((char*)serial.bufferData, sizeof(serial.bufferData), format, args);
    serial.write(huart, serial.bufferData, strlen((char*) serial.bufferData), UART_TIMEOUT);
    va_end(args);
}

