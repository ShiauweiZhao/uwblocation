//
// Created by ChenYuanfu on 2018/6/29.
//

#ifndef SERIALPRINT_UPRINT_UTIL_H
#define SERIALPRINT_UPRINT_UTIL_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include "string.h"

void vprint(USART_HandleTypeDef *husart, const char *fmt, va_list argp)
{
    char string[100];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_StatusTypeDef state = HAL_USART_Transmit(husart, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}


void uprintf(USART_HandleTypeDef *husart, const char *fmt, ...) {
    va_list argp;
    va_start(argp, fmt);
    vprint(husart, fmt, argp);
    va_end(argp);
}

void uprintUint8Buffer(USART_HandleTypeDef *husart, uint8_t *buff, uint32_t size){
    for (int ret = 0; ret < size; ret++) {
        if (!(ret % 6))
            puts("");
        uprintf(husart, "%.2X ", buff[ret]);
    }
    uprintf(husart, "\r\n");
    uprintf(husart, "\r\n");

}

static void printUint8BufferChar(USART_HandleTypeDef *husart, uint8_t *buff, uint32_t size){
    for (int ret = 0; ret < size; ret++) {
        if (!(ret % 6))
            puts("");
        //printf("%.2X ", buff[ret]);
        uprintf(husart,"%c ", buff[ret]);
    }

    uprintf(husart, "\r\n");
    uprintf(husart, "\r\n");
}


#endif //SERIALPRINT_UPRINT_UTIL_H
