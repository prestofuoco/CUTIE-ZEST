#include "radio/gps_driver.h"
#include "minmea.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

static UART_HandleTypeDef *gps_uart;
static GPS_Data_t current_data; // most recent gps data

static uint8_t rx_byte;
static char rx_line_buffer[GPS_BUFFER_SIZE];
static char processing_buffer[GPS_BUFFER_SIZE];
static volatile uint8_t rx_idx = 0;
static volatile bool line_ready = false;

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_uart = huart;

    // clear memory
    memset(&current_data, 0, sizeof(GPS_Data_t));
    memset(rx_line_buffer, 0, GPS_BUFFER_SIZE);    
    rx_idx   = 0;
    line_ready = false;

    // init rx
    HAL_UART_Receive_IT(gps_uart, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == gps_uart->Instance) {
        // end of nmea sentence
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_idx > 0) {
                rx_line_buffer[rx_idx] = '\0'; // terminate
                // copy so the interrupt can stay active
                memcpy(processing_buffer, rx_line_buffer, rx_idx + 1);
                line_ready = true;
                rx_idx = 0;
            }
        } 
        else {
            if (rx_idx < (GPS_BUFFER_SIZE - 1)) {
                rx_line_buffer[rx_idx++] = (char)rx_byte;
            }
        }

        HAL_UART_Receive_IT(gps_uart, &rx_byte, 1);
    }
}

void GPS_Process(void) {
    if (!line_ready) return;    // nothing to do yet
    line_ready = false;         // clear flag before processing

    enum minmea_sentence_id id = minmea_sentence_id(processing_buffer, false); 
    switch (id) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, processing_buffer)) {
                current_data.has_fix          = frame.valid; 
                if (frame.valid) {
                    current_data.latitude     = minmea_tocoord(&frame.latitude);
                    current_data.longitude    = minmea_tocoord(&frame.longitude);
                    current_data.speed_kph    = minmea_tofloat(&frame.speed) * 1.852f; // knots -> kph
                    struct timespec ts;
                    if (minmea_gettime(&ts, &frame.date, &frame.time) == 0) {
                        current_data.utc_time = (uint32_t)ts.tv_sec; }
                }
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, processing_buffer)) {
                current_data.has_fix    = (frame.fix_quality > 0);
                current_data.latitude   = minmea_tocoord(&frame.latitude);
                current_data.longitude  = minmea_tocoord(&frame.longitude);
                current_data.altitude   = minmea_tofloat(&frame.altitude);
                current_data.satellites = (uint8_t)frame.satellites_tracked;
            }
        } break;
        
        // ── sentences we don't need — silently ignore ─────────
        // case MINMEA_SENTENCE_GSV:
        // case MINMEA_SENTENCE_GSA:
        // case MINMEA_SENTENCE_GLL:
        // case MINMEA_INVALID:
        //case MINMEA_UNKNOWN:
        default:
            break;
    }
}

void GPS_LED_Tick(void) {
    static uint32_t last_toggle = 0;

    uint32_t interval = current_data.has_fix ? 1000 : 200;

    if ((HAL_GetTick() - last_toggle) >= interval) {
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        last_toggle = HAL_GetTick();
    }
}

GPS_Data_t GPS_GetLatestData(void) {
    return current_data;
}
