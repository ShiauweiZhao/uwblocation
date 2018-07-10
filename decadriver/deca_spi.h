//
// Created by zhaoxiaowei on 2018/7/10.
//

#ifndef UWBLOCATION_DECA_SPI_H
#define UWBLOCATION_DECA_SPI_H


#include "deca_types.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define DECA_MAX_SPI_HEADER_LENGTH      (3)                     // max number of bytes in header (for formating & sizing)
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void) ;




#endif //UWBLOCATION_DECA_SPI_H
