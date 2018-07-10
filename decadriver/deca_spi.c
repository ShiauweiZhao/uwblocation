//
// Created by zhaoxiaowei on 2018/7/10.
//

#include "deca_spi.h"
#include "deca_device_api.h"
extern SPI_HandleTypeDef hspi4;

int openspi(/*SPI_TypeDef* SPIx*/)
{
    // done by port.c, default SPI used is SPI1

    return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    //while (port_SPIx_busy_sending()); //wait for tx buffer to empty

    //port_SPIx_disable();

    return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{

    while (HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */ //cs pe3

    HAL_SPI_Transmit(&hspi4, (uint8 *)&headerBuffer[0], headerLength, 10);	/* Send header in polling mode */
    HAL_SPI_Transmit(&hspi4, (uint8_t *)&bodyBuffer[0], bodylength, 10);		/* Send data in polling mode */

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    uint8_t spi_TmpBuffer[4096+128];

//    decaIrqStatus_t  stat ;
//    stat = decamutexon() ;

    /* Blocking: Check whether previous transfer has been finished */
    while (HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_TransmitReceive(&hspi4, (uint8 *)headerBuffer, spi_TmpBuffer, (uint16)(headerLength+readlength), 10);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

    memcpy((uint8 *)readBuffer , (uint8 *)&spi_TmpBuffer[headerLength], readlength);


    return 0;
} // end readfromspi()