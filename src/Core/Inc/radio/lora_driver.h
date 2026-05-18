#ifndef INC_LORA_DRIVER_H_
#define INC_LORA_DRIVER_H_

#include "stm32g4xx_hal.h"
#include "sx126x_hal.h"
#include "sx126x.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RF_FREQUENCY 915000000

typedef struct {
    uint16_t syncword;
    uint32_t timestamp;
    float    latitude;
    float    longitude;
    float    altitude;
    uint8_t  battery_percentage;
    uint8_t  satellites;
    bool     has_fix;
    uint16_t crc;
    uint8_t  saved_data[4];
} LORA_Packet_t;

void          LORA_Init(void);
void          LORA_SendPacket(LORA_Packet_t *packet);
void          LORA_Process(void);
bool          LORA_PacketAvailable(void);
LORA_Packet_t LORA_GetLatestPacket(void);
uint32_t      LORA_GetLastSendTick(void); 

#endif