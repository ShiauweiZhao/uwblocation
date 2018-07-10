//
// Created by zhaoxiaowei on 2018/7/10.
//

#include "port.h"


extern SPI_HandleTypeDef hspi4;


void port_set_dw1000_slowrate(void)
{
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(&hspi4);
}

void port_set_dw1000_fastrate(void)
{
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi4);
}



//sleep ms
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}
void deca_sleep(uint32_t time_ms)
{
    Sleep(time_ms);
}


void reset_DW1000(void)
{
    HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
    deca_sleep(2);

}