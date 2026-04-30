#ifndef INC_GPS_DRIVER_H_
#define INC_GPS_DRIVER_H_

#include "stm32g4xx_hal.h"
#include "minmea.h"

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed_kph;
    uint8_t satellites;
    uint32_t utc_time;
    bool has_fix;
} GPS_Data_t;

// functions
void GPS_Init(UART_HandleTypeDef *huart); // init gps
void GPS_Process(void); // minmea parsing
GPS_Data_t GPS_GetLatestData(void); // return data

#endif