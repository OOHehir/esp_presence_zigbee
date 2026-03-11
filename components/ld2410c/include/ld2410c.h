#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

typedef struct {
    bool     moving_target;
    bool     stationary_target;
    uint8_t  move_energy;
    uint8_t  static_energy;
    uint16_t target_distance_cm;
} ld2410c_data_t;

/**
 * Initialise the LD2410C UART interface.
 * @param port  UART port number
 * @param tx_pin  GPIO for ESP TX → LD2410C RX
 * @param rx_pin  GPIO for ESP RX ← LD2410C TX
 */
esp_err_t ld2410c_init(uart_port_t port, int tx_pin, int rx_pin);

/**
 * Read one reporting frame from the LD2410C.
 * Blocks until a valid frame is received or timeout (1000ms).
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no valid frame received
 */
esp_err_t ld2410c_read(ld2410c_data_t *out);

/**
 * Shut down the LD2410C UART interface.
 */
void ld2410c_deinit(void);

/* --- Internal frame parsing (exposed for unit testing) --- */

/** Size of a standard (non-engineering) reporting frame */
#define LD2410C_FRAME_MAX_LEN 64

/**
 * Parse a complete reporting frame buffer into ld2410c_data_t.
 * @param buf   Buffer containing a complete frame (header through footer)
 * @param len   Length of data in buffer
 * @param out   Parsed output
 * @return ESP_OK if valid, ESP_ERR_INVALID_ARG if frame is malformed
 */
esp_err_t ld2410c_parse_frame(const uint8_t *buf, size_t len, ld2410c_data_t *out);
