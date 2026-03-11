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
/* Byte 17 = tail (0x55), byte 18 = check (0x00), then footer */

#define REPORT_DATA_TYPE 0x02
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

    /* Check data type is reporting frame */
    if (buf[OFF_DATA_TYPE] != REPORT_DATA_TYPE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate inner head byte */
    if (buf[OFF_HEAD] != INNER_HEAD) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Data length field */
    uint16_t data_len = buf[OFF_DATA_LEN_LO] | (buf[OFF_DATA_LEN_HI] << 8);

    /* Frame total = header(4) + len_field(2) + data(data_len) + footer(4) */
    size_t expected_len = HEADER_LEN + 2 + data_len + FOOTER_LEN;
    if (len < expected_len) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate tail and check bytes (relative to end of data, before footer) */
    size_t tail_off = HEADER_LEN + 2 + data_len - 2;  /* tail is 2 bytes before end of data */
    size_t check_off = tail_off + 1;
    if (buf[tail_off] != INNER_TAIL || buf[check_off] != CHECK_BYTE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Parse target state */
    uint8_t state = buf[OFF_TARGET_STATE];
    out->moving_target     = (state & 0x01) != 0;
    out->stationary_target = (state & 0x02) != 0;

    /* Parse distances and energy */
    out->move_energy  = buf[OFF_MOVE_ENERGY];
    out->static_energy = buf[OFF_STILL_ENERGY];

    /* Use detection distance as the reported target distance */
    out->target_distance_cm = buf[OFF_DET_DIST_LO] | (buf[OFF_DET_DIST_HI] << 8);

    return ESP_OK;
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
