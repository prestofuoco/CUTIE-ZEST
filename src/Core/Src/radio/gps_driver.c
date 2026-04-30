#include "radio/gps_driver.h"
#include "minmea.h"
#include <stdint.h>
#include <string.h>

#define GPS_BUFFER_SIZE 128

static UART_HandleTypeDef *gps_uart;
static GPS_Data_t current_data; // most recent gps data

static uint8_t  raw_rx_buffer[256];         // raw bytes from UART interrupt
static uint16_t rx_index = 0;

static char     nmea_line[128];             // one complete NMEA sentence
static bool     line_ready = false;         // flag: sentence ready to parse

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_uart = huart;

    // clear memory
    memset(&current_data, 0, sizeof(GPS_Data_t));
    memset(raw_rx_buffer, 0, sizeof(raw_rx_buffer));
    rx_index   = 0;
    line_ready = false;

    // init rx
    HAL_UART_Receive_IT(gps_uart, &raw_rx_buffer[rx_index], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != gps_uart) return;   // ignore other UARTs
    uint8_t byte = raw_rx_buffer[rx_index];

    if (byte == '\n') {
        // end of NMEA sentence — copy it and set the flag
        memcpy(nmea_line, raw_rx_buffer, rx_index + 1);
        nmea_line[rx_index] = '\0';
        line_ready = true;
        rx_index = 0;                // reset for next sentence
    } else {
        // keep accumulating, guard against overflow
        rx_index++;
        if (rx_index >= sizeof(raw_rx_buffer))
            rx_index = 0;           // overflow protection, reset
    }

    // re-arm for the next byte — keeps the chain going
    HAL_UART_Receive_IT(gps_uart, &raw_rx_buffer[rx_index], 1);
}

void GPS_Process(void) {
    if (!line_ready) return;    // nothing to do yet
    line_ready = false;         // clear flag before processing

    switch (minmea_sentence_id(nmea_line, false)) {
        // ── position, velocity, time ─────────────────────────
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, nmea_line))
            {
                current_data.has_fix      = frame.valid;
                current_data.latitude     = minmea_tocoord(&frame.latitude);
                current_data.longitude    = minmea_tocoord(&frame.longitude);
                current_data.speed_kph    = minmea_tofloat(&frame.speed) * 1.852f; // knots → kph
            }
        }
        break;

        // ── fix data (altitude + satellites) ─────────────────
        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, nmea_line))
            {
                current_data.has_fix    = (frame.fix_quality > 0);
                current_data.latitude   = minmea_tocoord(&frame.latitude);
                current_data.longitude  = minmea_tocoord(&frame.longitude);
                current_data.altitude   = minmea_tofloat(&frame.altitude);
                current_data.satellites = frame.satellites_tracked;
            }
        }
        break;

        // ── sentences we don't need — silently ignore ─────────
        case MINMEA_SENTENCE_GSV:
        case MINMEA_SENTENCE_GSA:
        case MINMEA_SENTENCE_GLL:
        case MINMEA_INVALID:
        case MINMEA_UNKNOWN:
        default:
            break;
    }
}

GPS_Data_t GPS_GetLatest(void) {
    return current_data;
}