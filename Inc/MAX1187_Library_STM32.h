//
//  MAX1187_Library_STM32.h
//  
//
//  Created by Luigi Raiano on 09/07/2020.
//

#ifndef __MAX1187_LIBRARY_STM32_H
#define __MAX1187_LIBRARY_STM32_H

#include "main.h"

// ADC specifications (16 bit adc max 1167)
#define ADC_res 0.00007694 // [V/LSB] (5/2^16)

void MAX1187_SPI_Scan(SPI_HandleTypeDef *hspi, uint8_t ConfByte, uint8_t RxBufferSize, uint8_t *RxData);

#endif /* __MAX1187_LIBRARY_STM32_H */
