#include "vl53l0x.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "vl53l0x";

/* VL53L0X I2C address (7-bit) */
#define VL53L0X_ADDR              0x29

/* Key registers */
#define REG_IDENTIFICATION_MODEL_ID           0xC0
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV  0x89
#define REG_MSRC_CONFIG_CONTROL               0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_SYSTEM_SEQUENCE_CONFIG            0x01
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET  0x4F
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO      0x0A
#define REG_GPIO_HV_MUX_ACTIVE_HIGH          0x84
#define REG_SYSTEM_INTERRUPT_CLEAR            0x0B
#define REG_RESULT_INTERRUPT_STATUS           0x13
#define REG_SYSRANGE_START                    0x00
#define REG_RESULT_RANGE_STATUS               0x14
#define REG_I2C_SLAVE_DEVICE_ADDRESS          0x8A

/* Expected model ID */
#define VL53L0X_MODEL_ID 0xEE

/* Range status values from the device (upper nibble of status register >> 3) */
#define RANGE_STATUS_VALID         0
#define RANGE_STATUS_SIGMA_FAIL    1
#define RANGE_STATUS_SIGNAL_FAIL   2
#define RANGE_STATUS_MIN_RANGE     3
#define RANGE_STATUS_PHASE_FAIL    4
#define RANGE_STATUS_HW_FAIL       5

/* I2C timeout */
#define I2C_TIMEOUT_MS  100

static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL;
static gpio_num_t s_xshut_pin = GPIO_NUM_NC;
static uint32_t s_timeout_count = 0;
static uint32_t s_reinit_count = 0;
#define TIMEOUT_REINIT_THRESHOLD  5   /* re-init after this many consecutive timeouts */
#define REINIT_MAX_ATTEMPTS       3   /* back off after this many consecutive failures */
#define REINIT_COOLDOWN_CYCLES   30   /* after max attempts, wait this many cycles before retrying */

/* --- Low-level I2C helpers --- */

static esp_err_t vl53l0x_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev_handle, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t vl53l0x_write_reg16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_transmit(s_dev_handle, buf, 3, I2C_TIMEOUT_MS);
}

static esp_err_t vl53l0x_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev_handle, &reg, 1, val, 1, I2C_TIMEOUT_MS);
}

static esp_err_t vl53l0x_read_reg16(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &reg, 1, buf, 2, I2C_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *val = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

static esp_err_t vl53l0x_read_multi(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev_handle, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

/* --- Status mapping --- */

uint8_t vl53l0x_map_status(uint8_t device_status)
{
    switch (device_status) {
    case RANGE_STATUS_VALID:
    case RANGE_STATUS_MIN_RANGE:
        return 0;   /* valid */
    case RANGE_STATUS_SIGMA_FAIL:
        return 1;
    case RANGE_STATUS_SIGNAL_FAIL:
        return 2;
    case RANGE_STATUS_PHASE_FAIL:
    case RANGE_STATUS_HW_FAIL:
        return 255; /* no target / error */
    default:
        return 0;   /* unknown status — treat as valid if range is plausible */
    }
}

/* --- Initialisation sequence ---
 * Based on the well-documented VL53L0X register init sequence used in
 * Pololu, Adafruit, and ST reference implementations.
 */

static esp_err_t vl53l0x_sensor_init(void)
{
    uint8_t model_id;
    esp_err_t ret;

    /* Verify model ID */
    ret = vl53l0x_read_reg(REG_IDENTIFICATION_MODEL_ID, &model_id);
    if (ret != ESP_OK) return ret;
    if (model_id != VL53L0X_MODEL_ID) {
        ESP_LOGE(TAG, "Wrong model ID: 0x%02X (expected 0x%02X)", model_id, VL53L0X_MODEL_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "VL53L0X detected (model ID: 0x%02X)", model_id);

    /* Set 2.8V I2C mode if needed */
    uint8_t vhv;
    ret = vl53l0x_read_reg(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv | 0x01);
    if (ret != ESP_OK) return ret;

    /* Standard I2C mode */
    ret = vl53l0x_write_reg(0x88, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(0x80, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(0xFF, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(0x00, 0x00);
    if (ret != ESP_OK) return ret;

    uint8_t stop_variable;
    ret = vl53l0x_read_reg(0x91, &stop_variable);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg(0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(0xFF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(0x80, 0x00);
    if (ret != ESP_OK) return ret;

    /* Set MSRC (minimum signal rate check) to not limit */
    uint8_t msrc;
    ret = vl53l0x_read_reg(REG_MSRC_CONFIG_CONTROL, &msrc);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(REG_MSRC_CONFIG_CONTROL, msrc | 0x12);
    if (ret != ESP_OK) return ret;

    /* Set signal rate limit to 0.25 MCPS (default) */
    ret = vl53l0x_write_reg16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    if (ret != ESP_OK) return ret;

    /* Configure interrupt: new sample ready */
    ret = vl53l0x_write_reg(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (ret != ESP_OK) return ret;

    uint8_t gpio_hv;
    ret = vl53l0x_read_reg(REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv);
    if (ret != ESP_OK) return ret;
    ret = vl53l0x_write_reg(REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv & ~0x10);
    if (ret != ESP_OK) return ret;

    ret = vl53l0x_write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Sensor initialised successfully");
    return ESP_OK;
}

esp_err_t vl53l0x_init(i2c_port_t port, int sda_pin, int scl_pin, gpio_num_t xshut_pin)
{
    /* Hardware reset via XSHUT if pin is provided */
    if (xshut_pin != GPIO_NUM_NC) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = 1ULL << xshut_pin,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io_cfg);
        gpio_set_level(xshut_pin, 0);    /* hold in shutdown */
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(xshut_pin, 1);    /* release — sensor boots */
        vTaskDelay(pdMS_TO_TICKS(2));     /* wait for boot (1.2ms typical) */
        ESP_LOGI(TAG, "XSHUT reset on GPIO%d", xshut_pin);
    }

    /* Create I2C master bus */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Add VL53L0X device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDR,
        .scl_speed_hz = 400000,
    };

    ret = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
        return ret;
    }

    /* Sensor-specific initialisation */
    ret = vl53l0x_sensor_init();
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(s_dev_handle);
        i2c_del_master_bus(s_bus_handle);
        s_dev_handle = NULL;
        s_bus_handle = NULL;
        return ret;
    }

    s_xshut_pin = xshut_pin;
    ESP_LOGI(TAG, "Initialised on I2C%d (SDA=%d, SCL=%d)", port, sda_pin, scl_pin);
    return ESP_OK;
}

esp_err_t vl53l0x_read(vl53l0x_data_t *out)
{
    if (!s_dev_handle || !out) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    /* Start single-shot measurement */
    ret = vl53l0x_write_reg(REG_SYSRANGE_START, 0x01);
    if (ret != ESP_OK) return ret;

    /* Wait for measurement complete (poll interrupt status) */
    uint8_t status;
    int timeout = 100; /* 100 x 10ms = 1s max */
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = vl53l0x_read_reg(REG_RESULT_INTERRUPT_STATUS, &status);
        if (ret != ESP_OK) return ret;
    } while (!(status & 0x07) && --timeout > 0);

    if (timeout == 0) {
        s_timeout_count++;
        if (s_timeout_count == 1 || (s_timeout_count % TIMEOUT_REINIT_THRESHOLD) == 0) {
            ESP_LOGW(TAG, "Measurement timed out (status=0x%02X, count=%lu)",
                     status, (unsigned long)s_timeout_count);
        }
        /* Re-init sensor after consecutive timeouts; back off then retry */
        if (s_timeout_count >= TIMEOUT_REINIT_THRESHOLD) {
            bool should_reinit;
            if (s_reinit_count < REINIT_MAX_ATTEMPTS) {
                should_reinit = true;
            } else {
                /* After max rapid attempts, retry every COOLDOWN_CYCLES timeouts */
                should_reinit = (s_timeout_count % (TIMEOUT_REINIT_THRESHOLD * REINIT_COOLDOWN_CYCLES)) == 0;
            }
            if (should_reinit) {
                s_reinit_count++;
                ESP_LOGW(TAG, "Re-initialising sensor (attempt %lu)",
                         (unsigned long)s_reinit_count);
                if (s_xshut_pin != GPIO_NUM_NC) {
                    gpio_set_level(s_xshut_pin, 0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    gpio_set_level(s_xshut_pin, 1);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                esp_err_t rinit = vl53l0x_sensor_init();
                if (rinit == ESP_OK) {
                    /* Discard first reading after re-init (often bogus) */
                    vl53l0x_write_reg(REG_SYSRANGE_START, 0x01);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    vl53l0x_write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
                }
                s_timeout_count = 0;
            }
        }
        out->range_cm = VL53L0X_OUT_OF_RANGE_CM;
        out->status = 255;
        return ESP_OK;
    }

    s_timeout_count = 0;
    s_reinit_count = 0;

    /* Read result (12 bytes from 0x14) */
    uint8_t result[12];
    ret = vl53l0x_read_multi(REG_RESULT_RANGE_STATUS, result, 12);
    if (ret != ESP_OK) return ret;

    /* Clear interrupt */
    ret = vl53l0x_write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (ret != ESP_OK) return ret;

    /* Extract range status (upper nibble of byte 0, shifted right by 3) */
    uint8_t range_status = (result[0] >> 3) & 0x0F;
    out->status = vl53l0x_map_status(range_status);

    /* Extract range in mm (bytes 10-11, big-endian) */
    uint16_t range = ((uint16_t)result[10] << 8) | result[11];

    ESP_LOGD(TAG, "Raw: range_status=%u mapped=%u range=%u mm", range_status, out->status, range);

    if (out->status != 0 || range > VL53L0X_MAX_RANGE_MM) {
        out->range_cm = VL53L0X_OUT_OF_RANGE_CM;
    } else {
        out->range_cm = (range + 5) / 10;  /* mm → cm, rounded */
    }

    return ESP_OK;
}

void vl53l0x_deinit(void)
{
    if (s_dev_handle) {
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = NULL;
    }
    if (s_bus_handle) {
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
    }
    ESP_LOGI(TAG, "Deinitialised");
}
