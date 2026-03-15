#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
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

/* Sensor polling interval */
#define SENSOR_REPORT_INTERVAL_MS 1000

static void sensor_report_task(void *arg)
{
    bool prev_ld_presence = false;
    uint8_t prev_ld_static_energy = UINT8_MAX;
    uint16_t prev_vl_range = 0;

    while (1) {
        ld2410c_data_t ld_data;
        vl53l0x_data_t vl_data;

        esp_err_t ld_err = ld2410c_read(&ld_data);
        esp_err_t vl_err = vl53l0x_read(&vl_data);

        if (ld_err == ESP_OK) {
            bool presence = ld_data.moving_target || ld_data.stationary_target;
            if (presence != prev_ld_presence) {
                ESP_LOGI(TAG, "LD2410C: %s (move=%d still=%d e=%u/%u dist=%ucm)",
                         presence ? "PRESENT" : "CLEAR",
                         ld_data.moving_target, ld_data.stationary_target,
                         ld_data.move_energy, ld_data.static_energy,
                         ld_data.target_distance_cm);
                prev_ld_presence = presence;
                zigbee_node_update_ld2410c(&ld_data);
            }
            if (ld_data.static_energy != prev_ld_static_energy) {
                ESP_LOGI(TAG, "LD2410C: static_energy=%u", ld_data.static_energy);
                prev_ld_static_energy = ld_data.static_energy;
                zigbee_node_update_static_energy(ld_data.static_energy);
            }
        } else {
            ESP_LOGW(TAG, "LD2410C read failed: %s", esp_err_to_name(ld_err));
        }

        if (vl_err == ESP_OK) {
            if (vl_data.range_cm != prev_vl_range) {
                ESP_LOGI(TAG, "VL53L0X: range=%ucm", vl_data.range_cm);
                prev_vl_range = vl_data.range_cm;
                zigbee_node_update_vl53l0x(&vl_data);
            }
        } else {
            ESP_LOGW(TAG, "VL53L0X read failed: %s", esp_err_to_name(vl_err));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_REPORT_INTERVAL_MS));
    }
}

void app_main(void)
{
    /* 1. Initialise NVS — required by Zigbee stack */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
