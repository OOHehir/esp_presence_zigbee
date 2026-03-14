#include "ld2410c.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ld2410c";

/* Frame markers */
static const uint8_t FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t FRAME_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};
#define HEADER_LEN    4
#define FOOTER_LEN    4

/* Byte offsets within a complete frame (from start of header) */
#define OFF_DATA_LEN_LO  4
#define OFF_DATA_LEN_HI  5
#define OFF_DATA_TYPE    6
#define OFF_HEAD         7  /* 0xAA */
#define OFF_TARGET_STATE 8
#define OFF_MOVE_DIST_LO 9
#define OFF_MOVE_DIST_HI 10
#define OFF_MOVE_ENERGY  11
#define OFF_STILL_DIST_LO 12
#define OFF_STILL_DIST_HI 13
#define OFF_STILL_ENERGY 14
#define OFF_DET_DIST_LO  15
#define OFF_DET_DIST_HI  16
/* Basic mode: byte 17 = tail (0x55), byte 18 = check (0x00), then footer */

/* Engineering mode additional offsets (after byte 16) */
#define OFF_ENG_MAX_MOVE_GATE   17
#define OFF_ENG_MAX_STILL_GATE  18
/* Per-gate energy arrays follow at byte 19 */

#define BASIC_DATA_TYPE  0x02
#define ENG_DATA_TYPE    0x01
#define INNER_HEAD       0xAA
#define INNER_TAIL       0x55
#define CHECK_BYTE       0x00

/* Minimum frame length: 4 header + 2 len + 13 data + 4 footer = 23 */
#define MIN_FRAME_LEN    23

/* UART config */
#define LD2410C_BAUD     256000
#define UART_BUF_SIZE    256
#define READ_TIMEOUT_MS  1000

static uart_port_t s_port = -1;
static uint32_t s_malformed_count = 0;

esp_err_t ld2410c_parse_frame(const uint8_t *buf, size_t len, ld2410c_data_t *out)
{
    if (!buf || !out || len < MIN_FRAME_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate header */
    if (memcmp(buf, FRAME_HEADER, HEADER_LEN) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate footer */
    if (memcmp(buf + len - FOOTER_LEN, FRAME_FOOTER, FOOTER_LEN) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Data length field */
    uint16_t data_len = buf[OFF_DATA_LEN_LO] | (buf[OFF_DATA_LEN_HI] << 8);

    /* Frame total = header(4) + len_field(2) + data(data_len) + footer(4) */
    size_t expected_len = HEADER_LEN + 2 + data_len + FOOTER_LEN;
    if (len < expected_len) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check data type — accept both basic and engineering */
    uint8_t data_type = buf[OFF_DATA_TYPE];
    if (data_type != BASIC_DATA_TYPE && data_type != ENG_DATA_TYPE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate inner head byte */
    if (buf[OFF_HEAD] != INNER_HEAD) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate tail and check bytes (relative to end of data, before footer) */
    size_t tail_off = HEADER_LEN + 2 + data_len - 2;
    size_t check_off = tail_off + 1;
    if (buf[tail_off] != INNER_TAIL || buf[check_off] != CHECK_BYTE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Zero-init the output */
    memset(out, 0, sizeof(*out));

    /* Parse target state */
    uint8_t state = buf[OFF_TARGET_STATE];
    out->moving_target     = (state & 0x01) != 0;
    out->stationary_target = (state & 0x02) != 0;

    /* Parse distances and energy */
    out->move_energy  = buf[OFF_MOVE_ENERGY];
    out->static_energy = buf[OFF_STILL_ENERGY];

    /* Use detection distance as the reported target distance */
    out->target_distance_cm = buf[OFF_DET_DIST_LO] | (buf[OFF_DET_DIST_HI] << 8);

    /* Engineering mode: parse per-gate energy values */
    if (data_type == ENG_DATA_TYPE) {
        out->engineering_mode = true;
        out->max_move_gate = buf[OFF_ENG_MAX_MOVE_GATE];
        out->max_still_gate = buf[OFF_ENG_MAX_STILL_GATE];

        /* Move gate energy values start at byte 19 */
        size_t move_count = out->max_move_gate + 1;
        if (move_count > LD2410C_MAX_GATES) move_count = LD2410C_MAX_GATES;
        size_t move_start = OFF_ENG_MAX_STILL_GATE + 1;  /* byte 19 */

        for (size_t i = 0; i < move_count && (move_start + i) < (expected_len - FOOTER_LEN - 2); i++) {
            out->move_gate_energy[i] = buf[move_start + i];
        }

        /* Still gate energy values follow after move gates */
        size_t still_count = out->max_still_gate + 1;
        if (still_count > LD2410C_MAX_GATES) still_count = LD2410C_MAX_GATES;
        size_t still_start = move_start + move_count;

        for (size_t i = 0; i < still_count && (still_start + i) < (expected_len - FOOTER_LEN - 2); i++) {
            out->still_gate_energy[i] = buf[still_start + i];
        }
    }

    return ESP_OK;
}

/* --- Command protocol --- */

static const uint8_t CMD_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FOOTER[] = {0x04, 0x03, 0x02, 0x01};
#define CMD_HEADER_LEN 4
#define CMD_FOOTER_LEN 4
#define CMD_MIN_ACK_LEN 10  /* header(4) + len(2) + cmd(2) + status(2) + footer(4) = 14, but min usable is 10 */
#define CMD_ACK_TIMEOUT_MS 2000

size_t ld2410c_build_cmd(uint8_t *buf, uint16_t cmd,
                         const uint8_t *payload, size_t payload_len)
{
    size_t i = 0;
    /* Header */
    buf[i++] = 0xFD; buf[i++] = 0xFC; buf[i++] = 0xFB; buf[i++] = 0xFA;
    /* Length = 2 (cmd) + payload */
    uint16_t len = 2 + payload_len;
    buf[i++] = len & 0xFF;
    buf[i++] = (len >> 8) & 0xFF;
    /* Command (little-endian) */
    buf[i++] = cmd & 0xFF;
    buf[i++] = (cmd >> 8) & 0xFF;
    /* Payload */
    if (payload && payload_len > 0) {
        memcpy(buf + i, payload, payload_len);
        i += payload_len;
    }
    /* Footer */
    buf[i++] = 0x04; buf[i++] = 0x03; buf[i++] = 0x02; buf[i++] = 0x01;
    return i;
}

esp_err_t ld2410c_parse_ack(const uint8_t *buf, size_t len, uint16_t cmd)
{
    if (!buf || len < 10) return ESP_ERR_INVALID_ARG;

    /* Validate header */
    if (memcmp(buf, CMD_HEADER, CMD_HEADER_LEN) != 0) return ESP_ERR_INVALID_ARG;

    /* Validate footer */
    if (memcmp(buf + len - CMD_FOOTER_LEN, CMD_FOOTER, CMD_FOOTER_LEN) != 0)
        return ESP_ERR_INVALID_ARG;

    /* ACK echoes command with bit 8 set: response cmd = cmd | 0x0100 */
    uint16_t ack_cmd = buf[6] | (buf[7] << 8);
    if (ack_cmd != (cmd | 0x0100)) return ESP_ERR_INVALID_RESPONSE;

    /* Status: 0x0000 = success */
    uint16_t status = buf[8] | (buf[9] << 8);
    if (status != 0) return ESP_FAIL;

    return ESP_OK;
}

/* Send command and wait for ACK */
static esp_err_t ld2410c_send_cmd(uint16_t cmd, const uint8_t *payload, size_t payload_len)
{
    uint8_t tx_buf[64];
    size_t tx_len = ld2410c_build_cmd(tx_buf, cmd, payload, payload_len);

    /* Flush any pending RX data */
    uart_flush_input(s_port);

    /* Send command */
    int written = uart_write_bytes(s_port, tx_buf, tx_len);
    if (written != (int)tx_len) return ESP_FAIL;

    /* Read ACK — scan for command header */
    uint8_t rx_buf[64];
    int rx_pos = 0;
    int hdr_matched = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(CMD_ACK_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        uint8_t byte;
        int rd = uart_read_bytes(s_port, &byte, 1, pdMS_TO_TICKS(100));
        if (rd <= 0) continue;

        if (hdr_matched < CMD_HEADER_LEN) {
            if (byte == CMD_HEADER[hdr_matched]) {
                rx_buf[hdr_matched] = byte;
                hdr_matched++;
                if (hdr_matched == CMD_HEADER_LEN) rx_pos = CMD_HEADER_LEN;
            } else {
                hdr_matched = (byte == CMD_HEADER[0]) ? 1 : 0;
                if (hdr_matched) rx_buf[0] = byte;
            }
            continue;
        }

        if (rx_pos < (int)sizeof(rx_buf)) {
            rx_buf[rx_pos++] = byte;
        } else {
            return ESP_ERR_INVALID_SIZE;
        }

        /* Check for footer */
        if (rx_pos >= 10 &&
            rx_buf[rx_pos - 4] == CMD_FOOTER[0] &&
            rx_buf[rx_pos - 3] == CMD_FOOTER[1] &&
            rx_buf[rx_pos - 2] == CMD_FOOTER[2] &&
            rx_buf[rx_pos - 1] == CMD_FOOTER[3]) {
            return ld2410c_parse_ack(rx_buf, rx_pos, cmd);
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t ld2410c_configure(uint8_t max_move_gate, uint8_t max_still_gate,
                            const uint8_t *move_thresh, const uint8_t *still_thresh,
                            uint16_t no_one_timeout, bool engineering)
{
    if (s_port < 0) return ESP_ERR_INVALID_STATE;
    if (max_move_gate > 8 || max_still_gate > 8) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;

    /* 1. Enter config mode */
    uint8_t enable_payload[] = {0x01, 0x00};
    ret = ld2410c_send_cmd(LD2410C_CMD_ENABLE_CONFIG, enable_payload, sizeof(enable_payload));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Enable config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Config mode enabled");

    /* 2. Enable/disable engineering mode first */
    {
        uint16_t eng_cmd = engineering ? LD2410C_CMD_ENG_MODE_ON : LD2410C_CMD_ENG_MODE_OFF;
        ret = ld2410c_send_cmd(eng_cmd, NULL, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Engineering mode %s failed: %s",
                     engineering ? "enable" : "disable", esp_err_to_name(ret));
            goto end_config;
        }
        ESP_LOGI(TAG, "Engineering mode %s", engineering ? "enabled" : "disabled");
    }

    /* 3. Set max gate distances and no-one timeout */
    {
        uint8_t gate_payload[] = {
            0x00, 0x00, max_move_gate, 0x00, 0x00, 0x00,
            0x01, 0x00, max_still_gate, 0x00, 0x00, 0x00,
            0x02, 0x00, (uint8_t)(no_one_timeout & 0xFF),
                        (uint8_t)(no_one_timeout >> 8), 0x00, 0x00,
        };
        ret = ld2410c_send_cmd(LD2410C_CMD_SET_MAX_GATE, gate_payload, sizeof(gate_payload));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Set max gate failed: %s", esp_err_to_name(ret));
            goto end_config;
        }
        ESP_LOGI(TAG, "Max gates: move=%u still=%u timeout=%us",
                 max_move_gate, max_still_gate, no_one_timeout);
    }

    /* 4. Set per-gate sensitivity */
    for (uint8_t g = 0; g <= (max_move_gate > max_still_gate ? max_move_gate : max_still_gate); g++) {
        uint8_t mv = (move_thresh && g <= max_move_gate) ? move_thresh[g] : 50;
        uint8_t st = (still_thresh && g <= max_still_gate) ? still_thresh[g] : 50;
        uint8_t sens_payload[] = {
            0x00, 0x00, g, 0x00, 0x00, 0x00,
            0x01, 0x00, mv, 0x00, 0x00, 0x00,
            0x02, 0x00, st, 0x00, 0x00, 0x00,
        };
        ret = ld2410c_send_cmd(LD2410C_CMD_SET_GATE_SENS, sens_payload, sizeof(sens_payload));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Set gate %u sensitivity failed: %s", g, esp_err_to_name(ret));
            goto end_config;
        }
        ESP_LOGI(TAG, "Gate %u: move=%u still=%u", g, mv, st);
    }
    ESP_LOGI(TAG, "Gate sensitivity configured");

end_config:
    /* 5. Exit config mode (always attempt even on error) */
    {
        esp_err_t end_ret = ld2410c_send_cmd(LD2410C_CMD_END_CONFIG, NULL, 0);
        if (end_ret != ESP_OK) {
            ESP_LOGE(TAG, "End config failed: %s", esp_err_to_name(end_ret));
            if (ret == ESP_OK) ret = end_ret;
        } else {
            ESP_LOGI(TAG, "Config mode ended");
        }
    }

    return ret;
}

esp_err_t ld2410c_init(uart_port_t port, int tx_pin, int rx_pin)
{
    uart_config_t uart_cfg = {
        .baud_rate  = LD2410C_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(port, &uart_cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    ret = uart_driver_install(port, UART_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    s_port = port;
    s_malformed_count = 0;
    ESP_LOGI(TAG, "Initialised on UART%d (TX=%d, RX=%d) at %d baud",
             port, tx_pin, rx_pin, LD2410C_BAUD);
    return ESP_OK;
}

esp_err_t ld2410c_read(ld2410c_data_t *out)
{
    if (s_port < 0 || !out) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[LD2410C_FRAME_MAX_LEN];
    int buf_pos = 0;
    int header_matched = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(READ_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        uint8_t byte;
        int read = uart_read_bytes(s_port, &byte, 1,
                                   pdMS_TO_TICKS(100));
        if (read <= 0) continue;

        /* Look for header */
        if (header_matched < HEADER_LEN) {
            if (byte == FRAME_HEADER[header_matched]) {
                buf[header_matched] = byte;
                header_matched++;
                if (header_matched == HEADER_LEN) {
                    buf_pos = HEADER_LEN;
                }
            } else {
                header_matched = (byte == FRAME_HEADER[0]) ? 1 : 0;
                if (header_matched) buf[0] = byte;
            }
            continue;
        }

        /* Accumulate data */
        if (buf_pos < LD2410C_FRAME_MAX_LEN) {
            buf[buf_pos++] = byte;
        } else {
            /* Frame too long — reset */
            header_matched = 0;
            buf_pos = 0;
            s_malformed_count++;
            ESP_LOGD(TAG, "Frame overflow (malformed: %lu)", (unsigned long)s_malformed_count);
            continue;
        }

        /* Check for footer once we have enough data */
        if (buf_pos >= MIN_FRAME_LEN &&
            buf[buf_pos - 4] == FRAME_FOOTER[0] &&
            buf[buf_pos - 3] == FRAME_FOOTER[1] &&
            buf[buf_pos - 2] == FRAME_FOOTER[2] &&
            buf[buf_pos - 1] == FRAME_FOOTER[3]) {

            /* Log first frame's raw hex for debugging frame type */
            static bool s_first_frame = true;
            if (s_first_frame) {
                char hex[LD2410C_FRAME_MAX_LEN * 3 + 1];
                int pos = 0;
                for (int i = 0; i < buf_pos && pos < (int)sizeof(hex) - 3; i++) {
                    pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[i]);
                }
                ESP_LOGI(TAG, "First frame (%d bytes, type=0x%02X): %s",
                         buf_pos, buf_pos > OFF_DATA_TYPE ? buf[OFF_DATA_TYPE] : 0, hex);
                s_first_frame = false;
            }

            esp_err_t ret = ld2410c_parse_frame(buf, buf_pos, out);
            if (ret == ESP_OK) {
                return ESP_OK;
            }

            s_malformed_count++;
            ESP_LOGD(TAG, "Bad frame (malformed: %lu)", (unsigned long)s_malformed_count);
            header_matched = 0;
            buf_pos = 0;
        }
    }

    return ESP_ERR_TIMEOUT;
}

void ld2410c_deinit(void)
{
    if (s_port >= 0) {
        uart_driver_delete(s_port);
        s_port = -1;
        ESP_LOGI(TAG, "Deinitialised");
    }
}
