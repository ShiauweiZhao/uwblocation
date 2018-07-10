//
// Created by zhaoxiaowei on 2018/7/10.
//

#ifndef UWBLOCATION_PORT_H
#define UWBLOCATION_PORT_H

#include "stm32f4xx_hal.h"


void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);
void Sleep(uint32_t Delay);
void reset_DW1000(void);
#endif //UWBLOCATION_PORT_H
