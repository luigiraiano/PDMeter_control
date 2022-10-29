//
//  MAX1187_Library_STM32.c
//  
//
//  Created by Luigi Raiano on 09/07/2020.
//

#include "MAX1187_Library_STM32.h"

// Multiple scan

void MAX1187_SPI_Scan(SPI_HandleTypeDef *hspi, uint8_t ConfByte, uint8_t RxBufferSize, uint8_t *RxData)
{
    if(hspi->Instance == SPI2)
    {
        // Start SPI communication with ADC (16-bit)
        HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET); // put PB12 = 0

        // Send configuration byte to ADC
        if(HAL_SPI_Transmit(hspi, (uint8_t *)&ConfByte, 1, 0xFF) != HAL_OK)
        {
            Error_Handler();
        }

        // WAIT FOR THE COMPLETION OF THE CONVERSION (NOT_EOC FALLS TO 0)
        while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))
        {
            // do nothing for waiting the end on conversion
        }

        // Receive converted data from ADC
        if(HAL_SPI_Receive(hspi, (uint8_t *)RxData, RxBufferSize, 0xFF) != HAL_OK)
        {
            Error_Handler();
        }

        // Stop SPI communication
        HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // put PB12 = 1
    }
}
