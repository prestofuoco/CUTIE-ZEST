#ifndef INC_LORA_DRIVER_H_
#define INC_LORA_DRIVER_H_

#include "sx126x.h"

#define RF_FREQUENCY 915000000

// prototypes
void LORA_Init(void);
void LORA_SendPacket(uint8_t *payload, uint8_t size);
void LORA_Process(void);

#endif