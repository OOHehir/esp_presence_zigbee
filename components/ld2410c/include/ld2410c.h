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

/**
 * Configure gate sensitivity and max detection range.
 * Must be called after ld2410c_init(). Enters config mode, sets parameters,
 * and returns to normal reporting mode.
 * @param max_move_gate   Max moving detection gate (0-8, each gate = 75cm)
 * @param max_still_gate  Max stationary detection gate (0-8)
 * @param move_thresh     Per-gate moving sensitivity thresholds (array of max_move_gate+1)
 *                        Lower = more sensitive (0-100). NULL for defaults.
 * @param still_thresh    Per-gate stationary sensitivity thresholds (array of max_still_gate+1)
 *                        Lower = more sensitive (0-100). NULL for defaults.
 * @param no_one_timeout  Seconds to wait before reporting "no target" (0 = immediate)
 */
esp_err_t ld2410c_configure(uint8_t max_move_gate, uint8_t max_still_gate,
                            const uint8_t *move_thresh, const uint8_t *still_thresh,
                            uint16_t no_one_timeout);

/** Number of distance gates (0-8) */
#define LD2410C_MAX_GATES  9

/** Distance per gate in cm */
#define LD2410C_GATE_CM    75

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

/* --- Command frame building/parsing (exposed for unit testing) --- */

/** Command frame header/footer */
#define LD2410C_CMD_HEADER_0 0xFD
#define LD2410C_CMD_HEADER_1 0xFC
#define LD2410C_CMD_HEADER_2 0xFB
#define LD2410C_CMD_HEADER_3 0xFA
#define LD2410C_CMD_FOOTER_0 0x04
#define LD2410C_CMD_FOOTER_1 0x03
#define LD2410C_CMD_FOOTER_2 0x02
#define LD2410C_CMD_FOOTER_3 0x01

/** Command codes */
#define LD2410C_CMD_ENABLE_CONFIG  0x00FF
#define LD2410C_CMD_END_CONFIG     0x00FE
#define LD2410C_CMD_SET_MAX_GATE   0x0060
#define LD2410C_CMD_SET_GATE_SENS  0x0064

/** Build a command frame into buf. Returns total frame length.
 *  @param buf       Output buffer (must be at least 4+2+2+payload_len+4 bytes)
 *  @param cmd       Command code (little-endian uint16)
 *  @param payload   Payload bytes (may be NULL if payload_len == 0)
 *  @param payload_len  Number of payload bytes
 */
size_t ld2410c_build_cmd(uint8_t *buf, uint16_t cmd,
                         const uint8_t *payload, size_t payload_len);

/** Parse command ACK frame. Returns ESP_OK if valid ACK with status==0.
 *  @param buf   Buffer containing ACK frame
 *  @param len   Length of buffer
 *  @param cmd   Expected command code (checked against ACK echo)
 */
esp_err_t ld2410c_parse_ack(const uint8_t *buf, size_t len, uint16_t cmd);
