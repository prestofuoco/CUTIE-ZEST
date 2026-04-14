#include "sx126x_hal.h"
#include "main.h"

// vibe coded this, double check later

// Reference the SPI handle defined in main.c
extern SPI_HandleTypeDef hspi1; 

void sx126x_hal_wait_on_busy( const void* context ) {
    // Uses the labels from your main.h
    while( HAL_GPIO_ReadPin( SX_BUSY_GPIO_Port, SX_BUSY_Pin ) == GPIO_PIN_SET );
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, uint16_t command_length,
                                     const uint8_t* data, uint16_t data_length ) {
    sx126x_hal_wait_on_busy( context );

    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET );

    HAL_SPI_Transmit( &hspi1, ( uint8_t* ) command, command_length, HAL_MAX_DELAY );
    if( data_length > 0 ) {
        HAL_SPI_Transmit( &hspi1, ( uint8_t* ) data, data_length, HAL_MAX_DELAY );
    }

    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, uint16_t command_length,
                                    uint8_t* data, uint16_t data_length ) {
    sx126x_hal_wait_on_busy( context );

    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET );

    // 1. Send the command
    HAL_SPI_Transmit( &hspi1, ( uint8_t* ) command, command_length, HAL_MAX_DELAY );
    
    // 2. The critical "Dummy Byte" for SX126x reads
    uint8_t dummy = 0x00;
    HAL_SPI_Transmit( &hspi1, &dummy, 1, HAL_MAX_DELAY );

    // 3. Receive the data
    HAL_SPI_Receive( &hspi1, data, data_length, HAL_MAX_DELAY );

    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context ) {
    HAL_GPIO_WritePin( SX_RST_GPIO_Port, SX_RST_Pin, GPIO_PIN_RESET );
    HAL_Delay( 20 ); // Semtech recommends ~20ms
    HAL_GPIO_WritePin( SX_RST_GPIO_Port, SX_RST_Pin, GPIO_PIN_SET );
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context ) {
    // Toggling NSS wakes up the SX126x from Sleep mode
    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET );
    HAL_Delay( 2 );
    HAL_GPIO_WritePin( SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET );
    return SX126X_HAL_STATUS_OK;
}