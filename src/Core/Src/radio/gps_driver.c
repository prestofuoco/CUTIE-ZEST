#include "radio/gps_driver.h"
#include "minmea.h"
#include <stdint.h>
#include <string.h>

#define GPS_BUFFER_SIZE 128

static UART_HandleTypeDef *gps_uart;
static GPS_Data_t current_data; // most recent gps data

static char rx_buffer[GPS_BUFFER_SIZE]; // dma/int byte storage
static char processing_buffer[GPS_BUFFER_SIZE]; // buffer for incoming data
static char rx_idx = 0;
static bool done = false;


void GPS_Init(UART_HandleTypeDef *huart) {
    gps_uart = huart;

    // clear memory
    memset(&current_data, 0, sizeof(GPS_Data_t));
    
    rx_index = 0;

    // init rx
    HAL_UART_Receive_IT(gps_uart, &raw_rx_buffer[rx_index], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == gps_uart->Instance) {
        // check for newline
        if (rx_buffer[rx_idx] == '\n' || rx_buffer[rx_idx] == '\r') {
            // zero-terminate
            rx_buffer[rx_idx] = '\0';
            
            // move data to processing buffer
            memcpy(processing_buffer, rx_buffer, GPS_BUFFER_SIZE);

            done = true;
            rx_idx = 0;
        } 
        else {
            // continue
            rx_idx++;
            // prevent overflow
            if (rx_idx >= GPS_BUFFER_SIZE) rx_idx = 0; 
        }
        HAL_UART_Receive_IT(gps_uart, (uint8_t *)&rx_buffer[rx_idx], 1);
    }
}

void GPS_Process(void) {
    bool parsed = minmea_parse_rmc(&frame, buffer);

    if (raw_rx_buffer) 
    // if is a complete sentence in raw rx buffer, pass it to minmea for parse
    // update current_data with the results
}

GPS_Data_t GPS_GetLatest(void) {
    return current_data;
}