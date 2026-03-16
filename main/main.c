#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "ld2410c.h"
#include "vl53l0x.h"
#include "zigbee_node.h"

static const char *TAG = "main";

/* Pin assignments */
#define LD2410C_TX_PIN  GPIO_NUM_4
#define LD2410C_RX_PIN  GPIO_NUM_5
#define VL53L0X_SDA_PIN   GPIO_NUM_6
#define VL53L0X_SCL_PIN   GPIO_NUM_7
#define VL53L0X_XSHUT_PIN GPIO_NUM_8
#define ZB_RESET_GPIO      GPIO_NUM_22  /* Pull-down; HIGH on boot = factory reset */

/* Sensor polling intervals */
#define LD_POLL_INTERVAL_MS  1000
#define VL_POLL_INTERVAL_MS  2000

/* ── LD2410C task (UART — lightweight, must not be blocked) ── */
static void ld2410c_task(void *arg)
{
    bool prev_presence = false;
    uint8_t prev_static_energy = UINT8_MAX;

    while (1) {
        ld2410c_data_t data;
        esp_err_t err = ld2410c_read(&data);

        if (err == ESP_OK) {
            bool presence = data.moving_target || data.stationary_target;
            if (presence != prev_presence) {
                ESP_LOGI(TAG, "LD2410C: %s (move=%d still=%d e=%u/%u dist=%ucm)",
                         presence ? "PRESENT" : "CLEAR",
                         data.moving_target, data.stationary_target,
                         data.move_energy, data.static_energy,
                         data.target_distance_cm);
                prev_presence = presence;
                zigbee_node_update_ld2410c(&data);
            }
            if (data.static_energy != prev_static_energy) {
                ESP_LOGI(TAG, "LD2410C: static_energy=%u", data.static_energy);
                prev_static_energy = data.static_energy;
                zigbee_node_update_static_energy(data.static_energy);
            }
        } else {
            ESP_LOGW(TAG, "LD2410C read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(LD_POLL_INTERVAL_MS));
    }
}

/* ── VL53L0X task (I2C — can block up to 1s on measurement) ── */
static void vl53l0x_task(void *arg)
{
    uint16_t prev_range = 0;
    uint8_t prev_status = 0;
    uint32_t read_count = 0;

    while (1) {
        vl53l0x_data_t data;
        esp_err_t err = vl53l0x_read(&data);
        read_count++;

        if (err == ESP_OK) {
            if (data.range_cm != prev_range || data.status != prev_status) {
                ESP_LOGI(TAG, "VL53L0X: range=%ucm status=%u", data.range_cm, data.status);
                prev_range = data.range_cm;
                prev_status = data.status;
                zigbee_node_update_vl53l0x(&data);
            } else if ((read_count % 30) == 0) {
                ESP_LOGI(TAG, "VL53L0X: alive range=%ucm status=%u (reads=%lu)",
                         data.range_cm, data.status, (unsigned long)read_count);
            }
        } else {
            ESP_LOGW(TAG, "VL53L0X read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(VL_POLL_INTERVAL_MS));
    }
}

/* ── Entry point launched by Zigbee signal handler ── */
static void sensor_report_task(void *arg)
{
    xTaskCreate(vl53l0x_task, "vl53l0x", 3072, NULL, 4, NULL);
    ld2410c_task(arg);  /* run LD2410C in this task */
}

/* ── Zigbee factory reset check ──────────────────────────────── */
static bool check_zigbee_reset(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ZB_RESET_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(50));  /* let level settle */

    if (gpio_get_level(ZB_RESET_GPIO) == 1) {
        ESP_LOGW(TAG, "GPIO %d HIGH — erasing Zigbee storage + NVS for factory reset", ZB_RESET_GPIO);
        const esp_partition_t *zb_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
        if (zb_part) {
            esp_partition_erase_range(zb_part, 0, zb_part->size);
        }
        nvs_flash_erase();
        return true;
    }
    return false;
}

void app_main(void)
{
    /* 1. Check for factory reset (GPIO 22 HIGH = reset) */
    bool factory_reset = check_zigbee_reset();

    /* 2. Initialise NVS — required by Zigbee stack */
    esp_err_t ret = nvs_flash_init();
    if (factory_reset || ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Initialise LD2410C (UART) */
    ret = ld2410c_init(UART_NUM_1, LD2410C_TX_PIN, LD2410C_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LD2410C init failed: %s — restarting", esp_err_to_name(ret));
        esp_restart();
    }

    /* Configure LD2410C for bench-up-at-ceiling deployment (~2m to ceiling).
     * Gate map: 0=0-75cm (bench clutter), 1=75-150cm (person upper body),
     *           2=150-225cm (ceiling ~200cm), 3=225-300cm (multipath).
     * Gate 0: disabled (100) — prototype board reflections
     * Gate 1: moderate (40) — primary person detection zone
     * Gate 2: sensitive move (30), high still (60) — ceiling is constant reflector
     * Gate 3: low sensitivity (80) — only strong multipath bounces
     * no_one_timeout=10s — debounce periodic false positives
     * Engineering mode OFF — OUT pin only works in basic mode.
     * All settings applied in a single config session. */
    static const uint8_t move_sens[]  = {100, 40, 30, 80};
    static const uint8_t still_sens[] = {100, 40, 60, 80};
    ret = ld2410c_configure(3, 3, move_sens, still_sens, 10, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LD2410C configure failed: %s — using defaults", esp_err_to_name(ret));
    }

    /* 3. Initialise VL53L0X (I2C) */
    ret = vl53l0x_init(I2C_NUM_0, VL53L0X_SDA_PIN, VL53L0X_SCL_PIN, VL53L0X_XSHUT_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X init failed: %s — restarting", esp_err_to_name(ret));
        esp_restart();
    }

    /* 4. Initialise Zigbee stack */
    ret = zigbee_node_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Zigbee init failed: %s — restarting", esp_err_to_name(ret));
        esp_restart();
    }

    /* 5. Start Zigbee main loop — does not return.
     * Sensor task is started from zigbee signal handler once network is up. */
    zigbee_node_start(sensor_report_task);
}
