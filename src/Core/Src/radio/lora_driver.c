#include "radio/lora_driver.h"
#include "minmea.h"
#include <stdint.h>
#include <string.h>

#define LORA_SYNCWORD       0xABCD
#define LORA_TX_POWER_DBM   22      // max power, adjust if needed

static SPI_HandleTypeDef *lora_spi;

static LORA_Packet_t latest_packet;
static bool          packet_available = false;

// ── CS pin helpers ───────────────────────────────────────────────
void sx126x_hal_set_nss(bool state) {
    HAL_GPIO_WritePin(
        SPI1_CS0_GPIO_Port,
        SPI1_CS0_Pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

// ── busy wait ────────────────────────────────────────────────────
static void sx126x_wait_for_busy(void) {
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(SX_BUSY_GPIO_Port, SX_BUSY_Pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= 500)
            return;     // timeout safety — don't hang forever
    }
}

// SPI read/write — sx126x library calls these internally
void sx126x_hal_write(const void *context, const uint8_t *command, uint16_t command_len, const uint8_t *data, uint16_t data_len) {
    sx126x_wait_for_busy();
    sx126x_hal_set_nss(false);
    HAL_SPI_Transmit(lora_spi, (uint8_t *)command, command_len, HAL_MAX_DELAY);
    if (data && data_len > 0)
        HAL_SPI_Transmit(lora_spi, (uint8_t *)data, data_len, HAL_MAX_DELAY);
    sx126x_hal_set_nss(true);
}

void sx126x_hal_read(const void *context, const uint8_t *command, uint16_t command_len, uint8_t *data, uint16_t data_len) {
    sx126x_wait_for_busy();
    sx126x_hal_set_nss(false);
    HAL_SPI_Transmit(lora_spi, (uint8_t *)command, command_len, HAL_MAX_DELAY);
    HAL_SPI_Receive(lora_spi, data, data_len, HAL_MAX_DELAY);
    sx126x_hal_set_nss(true);
}

// ── CRC ─────────────────────────────────────────────────────────
static uint16_t compute_crc(LORA_Packet_t *packet) {
    uint16_t crc = 0;
    uint8_t *bytes = (uint8_t *)packet;
    size_t len = sizeof(LORA_Packet_t) - sizeof(packet->crc) - sizeof(packet->saved_data);
    for (size_t i = 0; i < len; i++)
        crc += bytes[i];
    return crc;
}

// ── init ─────────────────────────────────────────────────────────
void LORA_Init(SPI_HandleTypeDef *hspi) {
    lora_spi = hspi;

    memset(&latest_packet, 0, sizeof(LORA_Packet_t));
    packet_available = false;

    // reset and wake chip
    sx126x_reset(NULL);
    sx126x_wakeup(NULL);

    // standby mode before config
    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);

    // set LoRa packet type
    sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_LORA);

    // 915 MHz
    sx126x_set_rf_freq(NULL, RF_FREQUENCY);

    // SF12, 125kHz bandwidth, coding rate 4/5, low data rate optimize on
    sx126x_lora_mod_params_t mod_params = {
        .sf   = SX126X_LORA_SF7,
        .bw   = SX126X_LORA_BW_125,
        .cr   = SX126X_LORA_CR_4_5,
        .ldro = 0
    };
    sx126x_set_lora_mod_params(NULL, &mod_params);

    // packet params — fixed length, our struct size
    sx126x_lora_pkt_params_t pkt_params = {
        .preamble_len_in_symb = 8,
        .header_type          = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes     = sizeof(LORA_Packet_t),
        .crc_is_on            = true,
        .invert_iq_is_on      = false
    };
    sx126x_set_lora_pkt_params(NULL, &pkt_params);

    // syncword — 0x3444 is public, 0x1424 is private; use private
    sx126x_set_lora_sync_word(NULL, 0x1424);

    // tx power 22dBm, 200us ramp
    sx126x_set_tx_params(NULL, LORA_TX_POWER_DBM, SX126X_RAMP_200_US);

    // PA config for SX1262 — max power
    sx126x_pa_cfg_params_t pa_cfg = {
        .pa_duty_cycle = 0x04,
        .hp_max        = 0x07,
        .device_sel    = 0x00,
        .pa_lut        = 0x01
    };
    sx126x_set_pa_cfg(NULL, &pa_cfg);

    // enable RX done and TX done IRQs on DIO1
    sx126x_set_dio_irq_params(NULL,
        SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_CRC_ERROR,
        SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_CRC_ERROR,
        SX126X_IRQ_NONE,
        SX126X_IRQ_NONE
    );

    // start listening
    sx126x_set_rx(NULL, 0);     // 0 = continuous RX
}

// ── send ─────────────────────────────────────────────────────────
void LORA_SendPacket(LORA_Packet_t *packet) {
    packet->syncword  = LORA_SYNCWORD;
    packet->timestamp = HAL_GetTick();
    packet->crc       = compute_crc(packet);

    // write payload into SX126x buffer starting at offset 0
    sx126x_write_buffer(NULL, 0, (uint8_t *)packet, sizeof(LORA_Packet_t));

    // trigger transmit — 0 timeout means return to standby when done
    sx126x_set_tx(NULL, 0);

    // wait for TX done IRQ (simple blocking wait — improve with interrupt later)
    sx126x_irq_mask_t irq;
    uint32_t tx_start = HAL_GetTick();

    do {
        sx126x_get_irq_status(NULL, &irq);

        if ((HAL_GetTick() - tx_start) >= 500)
        {
            sx126x_clear_irq_status(NULL, SX126X_IRQ_TX_DONE);
            sx126x_set_rx(NULL, 0);
            return;
        }

    } while (!(irq & SX126X_IRQ_TX_DONE));

    sx126x_clear_irq_status(NULL, SX126X_IRQ_TX_DONE);
    // go back to RX after sending
    sx126x_set_rx(NULL, 0);
}

// ── process ──────────────────────────────────────────────────────
void LORA_Process(void) {
    sx126x_irq_mask_t irq;
    sx126x_get_irq_status(NULL, &irq);

    // CRC error — bad packet, clear and move on
    if (irq & SX126X_IRQ_CRC_ERROR)
    {
        sx126x_clear_irq_status(NULL, SX126X_IRQ_CRC_ERROR);
        return;
    }

    if (!(irq & SX126X_IRQ_RX_DONE)) return;   // nothing received yet
    sx126x_clear_irq_status(NULL, SX126X_IRQ_RX_DONE);

    // find out where in the buffer the packet landed and how long it is
    sx126x_rx_buffer_status_t buf_status;
    sx126x_get_rx_buffer_status(NULL, &buf_status);

    // only accept packets that match our struct size exactly
    if (buf_status.pld_len_in_bytes != sizeof(LORA_Packet_t)) return;

    // read raw bytes out of SX126x buffer
    uint8_t raw[sizeof(LORA_Packet_t)];
    sx126x_read_buffer(NULL, buf_status.buffer_start_pointer, raw, sizeof(LORA_Packet_t));

    LORA_Packet_t *incoming = (LORA_Packet_t *)raw;

    // validate syncword
    if (incoming->syncword != LORA_SYNCWORD) return;

    // validate CRC
    if (incoming->crc != compute_crc(incoming)) return;

    // all good
    memcpy(&latest_packet, incoming, sizeof(LORA_Packet_t));
    packet_available = true;
}

// ── getters ──────────────────────────────────────────────────────
bool LORA_PacketAvailable(void) {
    return packet_available;
}

LORA_Packet_t LORA_GetLatestPacket(void) {
    packet_available = false;   // clear flag on read
    return latest_packet;
}