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
#define LD2410C_TX_PIN  GPIO_NUM_16
#define LD2410C_RX_PIN  GPIO_NUM_17
#define VL53L0X_SDA_PIN GPIO_NUM_6
#define VL53L0X_SCL_PIN GPIO_NUM_7

/* Sensor polling interval */
#define SENSOR_REPORT_INTERVAL_MS 5000

/* Thread safety: this task is the sole consumer of both sensors — no mutex needed */
static void sensor_report_task(void *arg)
{
    while (1) {
        ld2410c_data_t ld_data;
        vl53l0x_data_t vl_data;

        esp_err_t ld_err = ld2410c_read(&ld_data);
        esp_err_t vl_err = vl53l0x_read(&vl_data);

        if (ld_err == ESP_OK) {
            ESP_LOGD(TAG, "LD2410C: move=%d still=%d move_e=%u still_e=%u dist=%ucm",
                     ld_data.moving_target, ld_data.stationary_target,
                     ld_data.move_energy, ld_data.static_energy,
                     ld_data.target_distance_cm);
            zigbee_node_update_ld2410c(&ld_data);
        } else {
            ESP_LOGW(TAG, "LD2410C read failed: %s", esp_err_to_name(ld_err));
        }

        if (vl_err == ESP_OK) {
            ESP_LOGD(TAG, "VL53L0X: range=%umm status=%u",
                     vl_data.range_mm, vl_data.status);
            zigbee_node_update_vl53l0x(&vl_data);
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

    /* 3. Initialise VL53L0X (I2C) */
    ret = vl53l0x_init(I2C_NUM_0, VL53L0X_SDA_PIN, VL53L0X_SCL_PIN);
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

    /* 5. Start sensor reporting task */
    xTaskCreate(sensor_report_task, "sensor_report", 4096, NULL, 5, NULL);

    /* 6. Start Zigbee main loop — does not return */
    zigbee_node_start();
}
